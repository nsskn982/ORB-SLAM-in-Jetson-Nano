#!/usr/bin/env python3
"""
@file cnn_database_process.py
@brief Data selection and preprocessing for CNN collision datasets.
@author Srikanth Nanda
@date 2026-05-16
"""

import argparse
import csv
import os
import random

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
            image_path = row[0].strip()
            label = _parse_label(row[1])
            entries.append({"image": image_path, "label": label})
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


def parse_split(split_str):
    """@brief Parse a train/val/test split string.
    @param split_str Comma-separated ratios.
    @return Tuple of normalized ratios.
    """
    parts = [float(part.strip()) for part in split_str.split(",")]
    if len(parts) != 3:
        raise ValueError("Split must have three values: train,val,test")
    total = sum(parts)
    if total <= 0.0:
        raise ValueError("Split ratios must sum to > 0")
    return (parts[0] / total, parts[1] / total, parts[2] / total)


def filter_entries(entries, labels_keep):
    """@brief Filter entries by label.
    @param entries Dataset entries.
    @param labels_keep Optional set of labels to keep.
    @return Filtered entries.
    """
    if not labels_keep:
        return entries
    return [entry for entry in entries if entry["label"] in labels_keep]


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


def process_subset(entries, subset_name, input_dir, output_dir, width, height):
    """@brief Process and copy images into a subset directory.
    @param entries Entries to process.
    @param subset_name Subset name (train/val/test).
    @param input_dir Base input directory.
    @param output_dir Base output directory.
    @param width Output width (0 keeps original).
    @param height Output height (0 keeps original).
    @return List of [relative_image_path, label] rows.
    """
    rows = []
    for entry in entries:
        source_path, rel_path = resolve_source_path(input_dir, entry["image"])
        if not os.path.isfile(source_path):
            continue
        image = cv2.imread(source_path)
        if image is None:
            continue

        if width > 0 and height > 0:
            image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

        rel_path = ensure_extension(rel_path)
        dest_path = os.path.join(output_dir, subset_name, "images", rel_path)
        os.makedirs(os.path.dirname(dest_path), exist_ok=True)
        if not cv2.imwrite(dest_path, image):
            continue

        rows.append([os.path.join("images", rel_path), entry["label"]])
    return rows


def write_labels_csv(path, rows):
    """@brief Write labels CSV to disk.
    @param path Output CSV path.
    @param rows Rows to write.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["image", "label"])
        writer.writerows(rows)


def main():
    """@brief CLI entrypoint for dataset selection and preprocessing."""
    parser = argparse.ArgumentParser(description="CNN dataset selection and preprocessing")
    parser.add_argument("--input-dir", required=True, help="Directory with raw images")
    parser.add_argument("--labels", required=True, help="CSV file with image,label")
    parser.add_argument("--output-dir", default="data/processed", help="Output directory")
    parser.add_argument("--width", type=int, default=224, help="Resize width (0 keeps original)")
    parser.add_argument("--height", type=int, default=224, help="Resize height (0 keeps original)")
    parser.add_argument("--labels-keep", default="", help="Comma-separated labels to keep")
    parser.add_argument("--split", default="0.8,0.1,0.1", help="Train/val/test ratios")
    parser.add_argument("--seed", type=int, default=42, help="Shuffle seed")
    args = parser.parse_args()

    input_dir = os.path.abspath(args.input_dir)
    output_dir = os.path.abspath(args.output_dir)

    labels_keep = set()
    if args.labels_keep:
        labels_keep = {int(val.strip()) for val in args.labels_keep.split(",") if val.strip()}

    entries = read_label_rows(args.labels)
    entries = filter_entries(entries, labels_keep)

    random.seed(args.seed)
    random.shuffle(entries)

    split = parse_split(args.split)
    total = len(entries)
    train_count = int(total * split[0])
    val_count = int(total * split[1])
    test_count = total - train_count - val_count

    train_entries = entries[:train_count]
    val_entries = entries[train_count:train_count + val_count]
    test_entries = entries[train_count + val_count:]

    train_rows = process_subset(train_entries, "train", input_dir, output_dir, args.width, args.height)
    val_rows = process_subset(val_entries, "val", input_dir, output_dir, args.width, args.height)
    test_rows = process_subset(test_entries, "test", input_dir, output_dir, args.width, args.height)

    write_labels_csv(os.path.join(output_dir, "train", "labels.csv"), train_rows)
    write_labels_csv(os.path.join(output_dir, "val", "labels.csv"), val_rows)
    write_labels_csv(os.path.join(output_dir, "test", "labels.csv"), test_rows)

    print("Processed {0} entries -> train {1}, val {2}, test {3}".format(
        total, len(train_rows), len(val_rows), len(test_rows)
    ))


if __name__ == "__main__":
    main()
