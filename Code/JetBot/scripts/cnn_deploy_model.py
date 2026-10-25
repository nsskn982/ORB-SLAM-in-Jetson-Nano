#!/usr/bin/env python3
"""
@file cnn_deploy_model.py
@brief Deploy a CNN ONNX model and write a manifest.
@author Srikanth Nanda
@date 2026-05-16
"""

import argparse
import hashlib
import json
import os
import shutil
import time


DEFAULT_DEST = os.path.join("models", "collision.onnx")
DEFAULT_MANIFEST = os.path.join("models", "collision_manifest.json")


def compute_sha256(path):
    """@brief Compute SHA256 hash for a file.
    @param path File path.
    @return Hex digest string.
    """
    hasher = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            chunk = handle.read(8192)
            if not chunk:
                break
            hasher.update(chunk)
    return hasher.hexdigest()


def deploy_model(model_path, dest_path, manifest_path, input_width, input_height, mean, std, swap_rb):
    """@brief Copy model to destination and write a manifest.
    @param model_path Source ONNX model path.
    @param dest_path Destination model path.
    @param manifest_path Manifest JSON path.
    @param input_width Model input width.
    @param input_height Model input height.
    @param mean Mean normalization values.
    @param std Std normalization values.
    @param swap_rb Whether to swap RGB/BGR.
    """
    os.makedirs(os.path.dirname(dest_path), exist_ok=True)
    shutil.copy2(model_path, dest_path)

    manifest = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source_model": os.path.abspath(model_path),
        "deployed_model": os.path.abspath(dest_path),
        "sha256": compute_sha256(dest_path),
        "input_width": input_width,
        "input_height": input_height,
        "mean": mean,
        "std": std,
        "swap_rb": swap_rb,
    }

    os.makedirs(os.path.dirname(manifest_path), exist_ok=True)
    with open(manifest_path, "w") as handle:
        json.dump(manifest, handle, indent=2)

    return manifest


def main():
    """@brief CLI entrypoint for model deployment."""
    parser = argparse.ArgumentParser(description="Deploy CNN model")
    parser.add_argument("--model", required=True, help="Source ONNX model path")
    parser.add_argument("--dest", default=DEFAULT_DEST, help="Destination model path")
    parser.add_argument("--manifest", default=DEFAULT_MANIFEST, help="Manifest JSON path")
    parser.add_argument("--input-width", type=int, default=224)
    parser.add_argument("--input-height", type=int, default=224)
    parser.add_argument("--mean", default="0.485,0.456,0.406")
    parser.add_argument("--std", default="0.229,0.224,0.225")
    parser.add_argument("--swap-rb", action="store_true")
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        raise SystemExit("Model file not found: {0}".format(args.model))

    mean = [float(val.strip()) for val in args.mean.split(",")]
    std = [float(val.strip()) for val in args.std.split(",")]

    manifest = deploy_model(
        args.model,
        args.dest,
        args.manifest,
        args.input_width,
        args.input_height,
        mean,
        std,
        args.swap_rb,
    )

    print("Deployed model to {0}".format(args.dest))
    print("Manifest: {0}".format(args.manifest))
    print("SHA256: {0}".format(manifest["sha256"]))


if __name__ == "__main__":
    main()
