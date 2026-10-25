#!/usr/bin/env python3
"""
@file cnn_kdd_transform.py
@brief Data transformation and mining utilities for CNN datasets.
@author Srikanth Nanda
@date 2026-05-16
"""

import argparse
import csv
import hashlib
import json
import os

import cv2


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


def compute_blur_score(image):
    """@brief Compute a blur score using variance of Laplacian.
    @param image BGR image.
    @return Blur score.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def compute_file_hash(path):
    """@brief Compute an MD5 hash for a file.
    @param path File path.
    @return Hex digest string.
    """
    hasher = hashlib.md5()
    with open(path, "rb") as handle:
        while True:
            chunk = handle.read(8192)
            if not chunk:
                break
            hasher.update(chunk)
    return hasher.hexdigest()


def resolve_source_path(input_dir, image_path):
    """@brief Resolve a source image path.
    @param input_dir Base input directory.
    @param image_path Path from labels file.
    @return Absolute source path and relative path.
    """
    if os.path.isabs(image_path):
        source_path = image_path
        rel_path = os.path.relpath(image_path, input_dir) if os.path.commonpath([input_dir, image_path]) == os.path.abspath(input_dir) else os.path.basename(image_path)
    else:
        source_path = os.path.join(input_dir, image_path)
        rel_path = image_path
    return source_path, rel_path


def ensure_extension(path):
    """@brief Ensure file extension is supported for output.
    @param path Input path.
    @return Path with a supported extension.
    """
    _, ext = os.path.splitext(path)
    if ext.lower() in VALID_EXTENSIONS:
        return path
    return path + ".png"


def augment_image(image):
    """@brief Generate augmented variants of an image.
    @param image BGR image.
    @return List of (suffix, image) tuples.
    """
    flip = cv2.flip(image, 1)
    bright = cv2.convertScaleAbs(image, alpha=1.0, beta=30)
    return [("flip", flip), ("bright", bright)]


def write_csv(path, rows, header):
    """@brief Write a CSV file.
    @param path Output CSV path.
    @param rows Data rows.
    @param header Header row.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


def main():
    """@brief CLI entrypoint for KDD transformation and mining."""
    parser = argparse.ArgumentParser(description="CNN KDD transform and mining")
    parser.add_argument("--input-dir", required=True, help="Directory with images")
    parser.add_argument("--labels", required=True, help="CSV file with image,label")
    parser.add_argument("--output-dir", default="", help="Directory for reports and outputs")
    parser.add_argument("--blur-threshold", type=float, default=0.0, help="Flag images below this blur score")
    parser.add_argument("--dedupe", action="store_true", help="Remove duplicates in output labels")
    parser.add_argument("--augment", action="store_true", help="Write augmented images to output dir")
    args = parser.parse_args()

    input_dir = os.path.abspath(args.input_dir)
    output_dir = os.path.abspath(args.output_dir) if args.output_dir else input_dir

    entries = read_label_rows(args.labels)
    class_counts = {}
    duplicates = []
    blurred = []
    hashes = {}
    filtered_entries = []

    for entry in entries:
        source_path, rel_path = resolve_source_path(input_dir, entry["image"])
        if not os.path.isfile(source_path):
            continue

        class_counts[entry["label"]] = class_counts.get(entry["label"], 0) + 1

        file_hash = compute_file_hash(source_path)
        if file_hash in hashes:
            duplicates.append((rel_path, hashes[file_hash]))
            if args.dedupe:
                continue
        else:
            hashes[file_hash] = rel_path

        image = cv2.imread(source_path)
        if image is None:
            continue

        if args.blur_threshold > 0.0:
            blur_score = compute_blur_score(image)
            if blur_score < args.blur_threshold:
                blurred.append((rel_path, blur_score))

        filtered_entries.append(entry)

    report = {
        "total": len(entries),
        "kept": len(filtered_entries),
        "class_counts": class_counts,
        "duplicates": len(duplicates),
        "blurred": len(blurred),
    }

    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, "kdd_report.json"), "w") as handle:
        json.dump(report, handle, indent=2)

    if duplicates:
        write_csv(os.path.join(output_dir, "kdd_duplicates.csv"), duplicates, ["duplicate", "original"])

    if blurred:
        write_csv(os.path.join(output_dir, "kdd_blur.csv"), blurred, ["image", "blur_score"])

    if args.dedupe:
        rows = [[entry["image"], entry["label"]] for entry in filtered_entries]
        write_csv(os.path.join(output_dir, "kdd_filtered_labels.csv"), rows, ["image", "label"])

    if args.augment:
        aug_rows = []
        aug_root = os.path.join(output_dir, "augmented", "images")
        for entry in filtered_entries:
            source_path, rel_path = resolve_source_path(input_dir, entry["image"])
            image = cv2.imread(source_path)
            if image is None:
                continue
            for suffix, aug_image in augment_image(image):
                rel_path = ensure_extension(rel_path)
                base, ext = os.path.splitext(rel_path)
                aug_rel = "{0}_{1}{2}".format(base, suffix, ext)
                dest_path = os.path.join(aug_root, aug_rel)
                os.makedirs(os.path.dirname(dest_path), exist_ok=True)
                if cv2.imwrite(dest_path, aug_image):
                    aug_rows.append([os.path.join("images", aug_rel), entry["label"]])
        if aug_rows:
            write_csv(os.path.join(output_dir, "augmented", "labels.csv"), aug_rows, ["image", "label"])

    print("KDD report written to {0}".format(os.path.join(output_dir, "kdd_report.json")))


if __name__ == "__main__":
    main()
