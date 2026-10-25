#!/usr/bin/env python3
"""
@file cnn_train_tf.py
@brief Train and evaluate a CNN collision model using TensorFlow.
@author Srikanth Nanda
@date 2026-05-16
"""

import argparse
import csv
import json
import os
import time

import tensorflow as tf


VALID_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp")
AUTOTUNE = tf.data.AUTOTUNE if hasattr(tf.data, "AUTOTUNE") else 1


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


def resolve_dataset_paths(data_dir, subset):
    """@brief Resolve labels and image directory for a subset.
    @param data_dir Base data directory.
    @param subset Subset name (train/val/test).
    @return Tuple of (labels_path, images_root).
    """
    labels_path = os.path.join(data_dir, subset, "labels.csv")
    images_root = os.path.join(data_dir, subset)
    return labels_path, images_root


def build_dataset(entries, images_root, input_width, input_height, mean, std, swap_rb, batch_size, shuffle):
    """@brief Build a tf.data dataset from entries.
    @param entries List of data entries.
    @param images_root Root directory for images.
    @param input_width Input width.
    @param input_height Input height.
    @param mean Mean normalization values.
    @param std Std normalization values.
    @param swap_rb Whether to swap RGB/BGR.
    @param batch_size Batch size.
    @param shuffle Whether to shuffle.
    @return tf.data.Dataset instance.
    """
    image_paths = [os.path.join(images_root, entry["image"]) for entry in entries]
    labels = [int(entry["label"]) for entry in entries]

    dataset = tf.data.Dataset.from_tensor_slices((image_paths, labels))
    if shuffle:
        dataset = dataset.shuffle(buffer_size=min(len(image_paths), 1000), reshuffle_each_iteration=True)

    mean_tensor = tf.constant(mean, dtype=tf.float32)
    std_tensor = tf.constant(std, dtype=tf.float32)

    def _load_image(path, label):
        data = tf.io.read_file(path)
        image = tf.io.decode_image(data, channels=3)
        image.set_shape([None, None, 3])
        image = tf.image.resize(image, [input_height, input_width])
        if swap_rb:
            image = tf.reverse(image, axis=[-1])
        image = tf.cast(image, tf.float32) / 255.0
        image = (image - mean_tensor) / std_tensor
        return image, tf.cast(label, tf.float32)

    dataset = dataset.map(_load_image, num_parallel_calls=AUTOTUNE)
    dataset = dataset.batch(batch_size).prefetch(AUTOTUNE)
    return dataset


def build_model(input_shape):
    """@brief Build a lightweight CNN model.
    @param input_shape Input tensor shape.
    @return Compiled tf.keras.Model.
    """
    inputs = tf.keras.Input(shape=input_shape)
    x = tf.keras.layers.Conv2D(16, 3, padding="same", activation="relu")(inputs)
    x = tf.keras.layers.MaxPooling2D()(x)
    x = tf.keras.layers.Conv2D(32, 3, padding="same", activation="relu")(x)
    x = tf.keras.layers.MaxPooling2D()(x)
    x = tf.keras.layers.Conv2D(64, 3, padding="same", activation="relu")(x)
    x = tf.keras.layers.MaxPooling2D()(x)
    x = tf.keras.layers.Flatten()(x)
    x = tf.keras.layers.Dropout(0.3)(x)
    outputs = tf.keras.layers.Dense(1, activation="sigmoid")(x)
    return tf.keras.Model(inputs=inputs, outputs=outputs)


def save_json(path, payload):
    """@brief Write JSON data to disk.
    @param path Output path.
    @param payload JSON-serializable object.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=2)


def evaluate_model(model, dataset):
    """@brief Evaluate a model and return metrics.
    @param model Keras model.
    @param dataset Evaluation dataset.
    @return Dict of metrics.
    """
    results = model.evaluate(dataset, verbose=0)
    metrics = dict(zip(model.metrics_names, results))
    return metrics


def try_export_onnx(model, output_path):
    """@brief Export a Keras model to ONNX if tf2onnx is installed.
    @param model Keras model.
    @param output_path Output ONNX path.
    @return True if export succeeded, otherwise False.
    """
    try:
        import tf2onnx
    except ImportError:
        print("tf2onnx not installed; skipping ONNX export.")
        return False

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    spec = (tf.TensorSpec(model.inputs[0].shape, tf.float32, name="input"),)
    tf2onnx.convert.from_keras(model, input_signature=spec, output_path=output_path, opset=13)
    print("Exported ONNX to {0}".format(output_path))
    return True


def main():
    """@brief CLI entrypoint for TensorFlow training."""
    parser = argparse.ArgumentParser(description="Train CNN collision model with TensorFlow")
    parser.add_argument("--data-dir", default="data/processed", help="Processed dataset directory")
    parser.add_argument("--input-width", type=int, default=224)
    parser.add_argument("--input-height", type=int, default=224)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--mean", default="0.485,0.456,0.406")
    parser.add_argument("--std", default="0.229,0.224,0.225")
    parser.add_argument("--swap-rb", action="store_true")
    parser.add_argument("--output-dir", default="models/tf_training")
    parser.add_argument("--model-out", default="models/tf_training/collision_tf.h5")
    parser.add_argument("--log-dir", default="models/tf_training/logs")
    parser.add_argument("--export-onnx", action="store_true")
    parser.add_argument("--onnx-out", default="models/collision.onnx")
    args = parser.parse_args()

    mean = [float(val.strip()) for val in args.mean.split(",")]
    std = [float(val.strip()) for val in args.std.split(",")]

    train_labels, train_root = resolve_dataset_paths(args.data_dir, "train")
    val_labels, val_root = resolve_dataset_paths(args.data_dir, "val")
    test_labels, test_root = resolve_dataset_paths(args.data_dir, "test")

    if not os.path.isfile(train_labels):
        raise SystemExit("Train labels not found: {0}".format(train_labels))

    train_entries = read_label_rows(train_labels)
    val_entries = read_label_rows(val_labels) if os.path.isfile(val_labels) else []
    test_entries = read_label_rows(test_labels) if os.path.isfile(test_labels) else []

    train_ds = build_dataset(
        train_entries,
        train_root,
        args.input_width,
        args.input_height,
        mean,
        std,
        args.swap_rb,
        args.batch_size,
        shuffle=True,
    )

    val_ds = None
    if val_entries:
        val_ds = build_dataset(
            val_entries,
            val_root,
            args.input_width,
            args.input_height,
            mean,
            std,
            args.swap_rb,
            args.batch_size,
            shuffle=False,
        )

    model = build_model((args.input_height, args.input_width, 3))
    optimizer = tf.keras.optimizers.Adam(learning_rate=args.learning_rate)
    model.compile(
        optimizer=optimizer,
        loss=tf.keras.losses.BinaryCrossentropy(),
        metrics=[
            tf.keras.metrics.BinaryAccuracy(name="accuracy"),
            tf.keras.metrics.Precision(name="precision"),
            tf.keras.metrics.Recall(name="recall"),
        ],
    )

    os.makedirs(args.output_dir, exist_ok=True)

    callbacks = [
        tf.keras.callbacks.TensorBoard(log_dir=args.log_dir, update_freq="epoch"),
        tf.keras.callbacks.ModelCheckpoint(
            filepath=os.path.join(args.output_dir, "best_model.h5"),
            monitor="val_loss" if val_ds else "loss",
            save_best_only=True,
        ),
        tf.keras.callbacks.EarlyStopping(
            monitor="val_loss" if val_ds else "loss",
            patience=5,
            restore_best_weights=True,
        ),
        tf.keras.callbacks.CSVLogger(os.path.join(args.output_dir, "training_log.csv")),
    ]

    history = model.fit(
        train_ds,
        validation_data=val_ds,
        epochs=args.epochs,
        callbacks=callbacks,
        verbose=1,
    )

    model.save(args.model_out)
    save_json(os.path.join(args.output_dir, "training_history.json"), history.history)

    metrics = {}
    if test_entries:
        test_ds = build_dataset(
            test_entries,
            test_root,
            args.input_width,
            args.input_height,
            mean,
            std,
            args.swap_rb,
            args.batch_size,
            shuffle=False,
        )
        metrics = evaluate_model(model, test_ds)
        save_json(os.path.join(args.output_dir, "test_metrics.json"), metrics)

    if args.export_onnx:
        try_export_onnx(model, args.onnx_out)

    summary = {
        "model_path": os.path.abspath(args.model_out),
        "log_dir": os.path.abspath(args.log_dir),
        "test_metrics": metrics,
    }
    save_json(os.path.join(args.output_dir, "training_summary.json"), summary)

    print("Training complete. Model: {0}".format(args.model_out))
    if metrics:
        print("Test metrics saved to {0}".format(os.path.join(args.output_dir, "test_metrics.json")))


if __name__ == "__main__":
    main()
