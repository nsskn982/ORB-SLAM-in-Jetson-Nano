#!/usr/bin/env python3
"""
@file collision_node.py
@brief ROS node that runs CNN-based collision inference on incoming images.
@author Srikanth Nanda
@date 2026-05-08
"""

import math
import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class CollisionNode(object):
    def __init__(self):
        """@brief Initialize node state, ROS interfaces, and model."""
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.model_path = rospy.get_param("~model_path", "")

        self.input_width = int(rospy.get_param("~input_width", 224))
        self.input_height = int(rospy.get_param("~input_height", 224))
        self.expected_image_width = int(rospy.get_param("~expected_image_width", 640))
        self.expected_image_height = int(rospy.get_param("~expected_image_height", 480))
        self.inference_hz = float(rospy.get_param("~inference_hz", 10.0))
        self.queue_size = int(rospy.get_param("~queue_size", 1))

        self.collision_threshold = float(rospy.get_param("~collision_threshold", 0.60))
        self.collision_class_index = int(rospy.get_param("~collision_class_index", 1))
        self.fail_safe_probability = float(rospy.get_param("~fail_safe_probability", 1.0))

        self.swap_rb = self._as_bool(rospy.get_param("~swap_rb", True))
        mean_default = [0.485, 0.456, 0.406]
        std_default = [0.229, 0.224, 0.225]
        self.mean = self._to_float_list(rospy.get_param("~mean", mean_default), 3)
        self.std = self._to_float_list(rospy.get_param("~std", std_default), 3)

        self.mean_arr = np.array(self.mean, dtype=np.float32).reshape((1, 1, 3))
        std_arr = np.array(self.std, dtype=np.float32)
        std_arr[std_arr < 1e-6] = 1e-6
        self.std_arr = std_arr.reshape((1, 1, 3))

        self.period_sec = 1.0 / max(1e-6, self.inference_hz)
        self.last_run_time = rospy.Time(0)
        self.supported_encodings = set(["bgr8", "rgb8", "mono8"])

        self.net = None
        self.net_lock = threading.Lock()

        self.prob_pub = rospy.Publisher("/collision/probability", Float32, queue_size=self.queue_size)
        self.risky_pub = rospy.Publisher("/collision/risky", Bool, queue_size=self.queue_size)

        self._load_model()

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=self.queue_size,
            buff_size=2 ** 20,
        )

        rospy.loginfo(
            "collision_node initialized image_topic=%s model_path=%s",
            self.image_topic,
            self.model_path if self.model_path else "<empty>",
        )

    def _as_bool(self, value):
        """@brief Convert common truthy values to bool.
        @param value Input value.
        @return True if value is truthy, otherwise False.
        """
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _to_float_list(self, value, expected_len):
        """@brief Convert input to a fixed-length list of floats.
        @param value Scalar or list/tuple of values.
        @param expected_len Required length of the list.
        @return List of floats with expected length.
        """
        if isinstance(value, (list, tuple)):
            result = [float(v) for v in value]
        else:
            result = [float(value)]
        if len(result) != expected_len:
            raise ValueError("Expected list length %d, got %d" % (expected_len, len(result)))
        return result

    def _load_model(self):
        """@brief Load the ONNX collision model using OpenCV DNN."""
        if not self.model_path:
            rospy.logwarn("No CNN model configured. Using fail-safe probability output.")
            return
        try:
            self.net = cv2.dnn.readNetFromONNX(self.model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            rospy.loginfo("Loaded CNN model from %s", self.model_path)
        except Exception as exc:
            self.net = None
            rospy.logwarn("Failed to load CNN model from %s: %s", self.model_path, str(exc))

    def _image_callback(self, msg):
        """@brief Handle incoming image messages and publish collision probability.
        @param msg ROS Image message.
        """
        now = rospy.Time.now()
        if (now - self.last_run_time).to_sec() < self.period_sec:
            return
        self.last_run_time = now

        if msg.encoding not in self.supported_encodings:
            rospy.logwarn_throttle(2.0, "Skipping unsupported image encoding: %s", msg.encoding)
            return

        if (
            self.expected_image_width > 0 and self.expected_image_height > 0 and
            (msg.width != self.expected_image_width or msg.height != self.expected_image_height)
        ):
            rospy.logwarn_throttle(
                5.0,
                "Input image size is %dx%d, expected %dx%d",
                msg.width,
                msg.height,
                self.expected_image_width,
                self.expected_image_height,
            )

        try:
            bgr_image = self._ros_image_to_bgr(msg)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Skipping frame due to image conversion error: %s", str(exc))
            return

        if self.net is None:
            self._publish_probability(self.fail_safe_probability)
            return

        try:
            blob = self._preprocess(bgr_image)
            with self.net_lock:
                self.net.setInput(blob)
                output = self.net.forward()
            probability = self._extract_collision_probability(output)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "CNN inference failed: %s", str(exc))
            probability = self.fail_safe_probability

        self._publish_probability(probability)

    def _ros_image_to_bgr(self, msg):
        """@brief Convert ROS image message into a BGR numpy array.
        @param msg ROS Image message.
        @return BGR image as numpy array.
        """
        if msg.height <= 0 or msg.width <= 0:
            raise ValueError("Invalid image size")

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        h = int(msg.height)
        w = int(msg.width)

        if msg.encoding in ("bgr8", "rgb8"):
            needed = h * w * 3
            if raw.size < needed:
                raise ValueError("Insufficient RGB/BGR data")
            image = raw[:needed].reshape((h, w, 3))
            if msg.encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image

        if msg.encoding == "mono8":
            needed = h * w
            if raw.size < needed:
                raise ValueError("Insufficient mono data")
            mono = raw[:needed].reshape((h, w))
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

        raise ValueError("Unsupported image encoding: %s" % msg.encoding)

    def _preprocess(self, bgr_image):
        """@brief Resize and normalize an image for CNN inference.
        @param bgr_image Input BGR image.
        @return NCHW float32 blob.
        """
        resized = cv2.resize(
            bgr_image,
            (self.input_width, self.input_height),
            interpolation=cv2.INTER_AREA,
        )

        if self.swap_rb:
            resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        image = resized.astype(np.float32) / 255.0
        image = (image - self.mean_arr) / self.std_arr

        chw = np.transpose(image, (2, 0, 1))
        return np.expand_dims(chw, axis=0)

    def _extract_collision_probability(self, output):
        """@brief Interpret network output as a collision probability.
        @param output Raw network output.
        @return Probability in [0, 1].
        """
        vector = np.array(output, dtype=np.float32).reshape(-1)
        if vector.size == 0:
            return self.fail_safe_probability

        if vector.size == 1:
            value = float(vector[0])
            if 0.0 <= value <= 1.0:
                return value
            if value >= 50.0:
                return 1.0
            if value <= -50.0:
                return 0.0
            return 1.0 / (1.0 + math.exp(-value))

        index = max(0, min(self.collision_class_index, vector.size - 1))

        if np.all(vector >= 0.0) and np.all(vector <= 1.0):
            total = float(np.sum(vector))
            if abs(total - 1.0) <= 0.05:
                return float(vector[index])

        max_logit = float(np.max(vector))
        exps = np.exp(vector - max_logit)
        probs = exps / np.sum(exps)
        return float(probs[index])

    def _publish_probability(self, probability):
        """@brief Publish collision probability and risky flag.
        @param probability Probability value.
        """
        prob = max(0.0, min(1.0, float(probability)))
        self.prob_pub.publish(Float32(data=prob))
        self.risky_pub.publish(Bool(data=(prob >= self.collision_threshold)))


def main():
    """@brief ROS node entrypoint."""
    rospy.init_node("collision_node")
    CollisionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
