#!/usr/bin/env python3
"""
@file cnn_dev_to_deploy.py
@brief Validate CNN model on samples and deploy it for runtime.
@author Srikanth Nanda
@date 2026-05-16
"""

import argparse
import os
import time

import cv2
import numpy as np

import cnn_deploy_model


VALID_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp")


def list_images(sample_dir):
    """@brief List image files in a directory.
    @param sample_dir Directory to scan.
    @return List of image paths.
    """
    images = []
    for root, _, files in os.walk(sample_dir):
        for name in files:
            if os.path.splitext(name)[1].lower() in VALID_EXTENSIONS:
                images.append(os.path.join(root, name))
    return images


def preprocess_image(image, input_width, input_height, swap_rb, mean, std):
    """@brief Resize and normalize an image for inference.
    @param image BGR image.
    @param input_width Input width.
    @param input_height Input height.
    @param swap_rb Whether to swap RGB/BGR.
    @param mean Mean values.
    @param std Std values.
    @return NCHW float32 blob.
    """
    resized = cv2.resize(image, (input_width, input_height), interpolation=cv2.INTER_AREA)
    if swap_rb:
        resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

    mean_arr = np.array(mean, dtype=np.float32).reshape((1, 1, 3))
    std_arr = np.array(std, dtype=np.float32).reshape((1, 1, 3))
    std_arr[std_arr < 1e-6] = 1e-6

    image = resized.astype(np.float32) / 255.0
    image = (image - mean_arr) / std_arr
    chw = np.transpose(image, (2, 0, 1))
    return np.expand_dims(chw, axis=0)


def extract_probability(output, collision_class_index):
    """@brief Convert model output into a collision probability.
    @param output Raw model output.
    @param collision_class_index Class index for collision.
    @return Probability in [0, 1].
    """
    vector = np.array(output, dtype=np.float32).reshape(-1)
    if vector.size == 0:
        return 0.0

    if vector.size == 1:
        value = float(vector[0])
        if 0.0 <= value <= 1.0:
            return value
        if value >= 50.0:
            return 1.0
        if value <= -50.0:
            return 0.0
        return 1.0 / (1.0 + np.exp(-value))

    index = max(0, min(collision_class_index, vector.size - 1))
    max_logit = float(np.max(vector))
    exps = np.exp(vector - max_logit)
    probs = exps / np.sum(exps)
    return float(probs[index])


def validate_model(model_path, sample_dir, max_samples, input_width, input_height, swap_rb, mean, std, collision_class_index):
    """@brief Run inference on sample images to validate the model.
    @param model_path ONNX model path.
    @param sample_dir Directory with sample images.
    @param max_samples Maximum number of samples to test.
    @param input_width Input width.
    @param input_height Input height.
    @param swap_rb Whether to swap RGB/BGR.
    @param mean Mean values.
    @param std Std values.
    @param collision_class_index Collision class index.
    @return Tuple of (sample_count, avg_latency_ms).
    """
    images = list_images(sample_dir)
    if not images:
        return 0, 0.0

    net = cv2.dnn.readNetFromONNX(model_path)
    total_latency = 0.0
    sample_count = 0

    for path in images[:max_samples]:
        image = cv2.imread(path)
        if image is None:
            continue
        blob = preprocess_image(image, input_width, input_height, swap_rb, mean, std)
        start = time.perf_counter()
        net.setInput(blob)
        output = net.forward()
        _ = extract_probability(output, collision_class_index)
        total_latency += (time.perf_counter() - start) * 1000.0
        sample_count += 1

    if sample_count == 0:
        return 0, 0.0
    return sample_count, total_latency / sample_count


def main():
    """@brief CLI entrypoint for dev-to-deploy validation."""
    parser = argparse.ArgumentParser(description="Validate and deploy CNN model")
    parser.add_argument("--model", required=True, help="ONNX model path")
    parser.add_argument("--sample-dir", default="", help="Directory of sample images")
    parser.add_argument("--max-samples", type=int, default=25)
    parser.add_argument("--input-width", type=int, default=224)
    parser.add_argument("--input-height", type=int, default=224)
    parser.add_argument("--mean", default="0.485,0.456,0.406")
    parser.add_argument("--std", default="0.229,0.224,0.225")
    parser.add_argument("--swap-rb", action="store_true")
    parser.add_argument("--collision-class-index", type=int, default=1)
    parser.add_argument("--deploy", action="store_true", help="Copy model into models/")
    parser.add_argument("--dest", default=cnn_deploy_model.DEFAULT_DEST)
    parser.add_argument("--manifest", default=cnn_deploy_model.DEFAULT_MANIFEST)
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        raise SystemExit("Model file not found: {0}".format(args.model))

    mean = [float(val.strip()) for val in args.mean.split(",")]
    std = [float(val.strip()) for val in args.std.split(",")]

    if args.sample_dir:
        samples, avg_ms = validate_model(
            args.model,
            args.sample_dir,
            args.max_samples,
            args.input_width,
            args.input_height,
            args.swap_rb,
            mean,
            std,
            args.collision_class_index,
        )
        if samples:
            print("Validated {0} samples, avg latency {1:.2f} ms".format(samples, avg_ms))
        else:
            print("No valid sample images found for validation.")

    if args.deploy:
        cnn_deploy_model.deploy_model(
            args.model,
            args.dest,
            args.manifest,
            args.input_width,
            args.input_height,
            mean,
            std,
            args.swap_rb,
        )
        print("Model deployed to {0}".format(args.dest))


if __name__ == "__main__":
    main()
