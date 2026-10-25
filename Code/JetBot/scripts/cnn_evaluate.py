#!/usr/bin/env python3
"""
@file cnn_evaluate.py
@brief Evaluate a CNN collision model against labeled data.
@author Srikanth Nanda
@date 2026-05-16
"""

import argparse
import csv
import json
import os
import time

import cv2
import numpy as np


VALID_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp")


def read_label_rows(labels_path):
    """@brief Read label CSV rows with image path and label.
    @param labels_path Path to the labels CSV file.
    @return List of dicts with image and label keys.
    """
    entries = []
    with open(labels_path, "r", newline="") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row or all(not cell.strip() for cell in row):
                continue
            if _is_header_row(row):
                continue
            if len(row) < 2:
                continue
            entries.append({"image": row[0].strip(), "label": _parse_label(row[1])})
    return entries


def _is_header_row(row):
    """@brief Detect if a CSV row is a header.
    @param row CSV row.
    @return True if row looks like a header.
    """
    if len(row) < 2:
        return False
    header_a = row[0].strip().lower()
    header_b = row[1].strip().lower()
    return header_a in ("image", "path", "filename") or header_b in ("label", "class", "target")


def _parse_label(value):
    """@brief Parse a label value into an integer.
    @param value Label string.
    @return Integer label.
    """
    try:
        return int(value)
    except ValueError:
        return int(float(value))


def resolve_source_path(input_dir, image_path):
    """@brief Resolve a source image path.
    @param input_dir Base input directory.
    @param image_path Path from labels file.
    @return Absolute source path.
    """
    if os.path.isabs(image_path):
        return image_path
    return os.path.join(input_dir, image_path)


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


def compute_metrics(tp, fp, tn, fn):
    """@brief Compute accuracy, precision, recall, and F1.
    @param tp True positives.
    @param fp False positives.
    @param tn True negatives.
    @param fn False negatives.
    @return Dict of metrics.
    """
    total = tp + fp + tn + fn
    accuracy = (tp + tn) / total if total else 0.0
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    if precision + recall == 0.0:
        f1 = 0.0
    else:
        f1 = 2.0 * precision * recall / (precision + recall)
    return {
        "accuracy": accuracy,
        "precision": precision,
        "recall": recall,
        "f1": f1,
    }


def main():
    """@brief CLI entrypoint for CNN evaluation."""
    parser = argparse.ArgumentParser(description="Evaluate CNN collision model")
    parser.add_argument("--model", required=True, help="ONNX model path")
    parser.add_argument("--input-dir", required=True, help="Directory with images")
    parser.add_argument("--labels", required=True, help="CSV file with image,label")
    parser.add_argument("--input-width", type=int, default=224)
    parser.add_argument("--input-height", type=int, default=224)
    parser.add_argument("--mean", default="0.485,0.456,0.406")
    parser.add_argument("--std", default="0.229,0.224,0.225")
    parser.add_argument("--swap-rb", action="store_true")
    parser.add_argument("--collision-class-index", type=int, default=1)
    parser.add_argument("--threshold", type=float, default=0.60)
    parser.add_argument("--output-json", default="", help="Optional JSON output path")
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        raise SystemExit("Model file not found: {0}".format(args.model))

    mean = [float(val.strip()) for val in args.mean.split(",")]
    std = [float(val.strip()) for val in args.std.split(",")]

    entries = read_label_rows(args.labels)
    if not entries:
        raise SystemExit("No labeled entries found.")

    net = cv2.dnn.readNetFromONNX(args.model)
    tp = fp = tn = fn = 0
    latencies = []

    for entry in entries:
        source_path = resolve_source_path(args.input_dir, entry["image"])
        if not os.path.isfile(source_path):
            continue
        image = cv2.imread(source_path)
        if image is None:
            continue

        blob = preprocess_image(image, args.input_width, args.input_height, args.swap_rb, mean, std)
        start = time.perf_counter()
        net.setInput(blob)
        output = net.forward()
        latencies.append((time.perf_counter() - start) * 1000.0)
        prob = extract_probability(output, args.collision_class_index)

        predicted = 1 if prob >= args.threshold else 0
        actual = 1 if entry["label"] else 0

        if predicted == 1 and actual == 1:
            tp += 1
        elif predicted == 1 and actual == 0:
            fp += 1
        elif predicted == 0 and actual == 0:
            tn += 1
        else:
            fn += 1

    metrics = compute_metrics(tp, fp, tn, fn)
    avg_latency = sum(latencies) / len(latencies) if latencies else 0.0

    results = {
        "tp": tp,
        "fp": fp,
        "tn": tn,
        "fn": fn,
        "metrics": metrics,
        "avg_latency_ms": avg_latency,
    }

    print("Accuracy: {0:.3f} Precision: {1:.3f} Recall: {2:.3f} F1: {3:.3f}".format(
        metrics["accuracy"], metrics["precision"], metrics["recall"], metrics["f1"]
    ))
    print("Avg latency: {0:.2f} ms".format(avg_latency))

    if args.output_json:
        with open(args.output_json, "w") as handle:
            json.dump(results, handle, indent=2)
        print("Saved results to {0}".format(args.output_json))


if __name__ == "__main__":
    main()
