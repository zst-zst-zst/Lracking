#!/usr/bin/env python3
"""Train Layer 2: Laser RX detection ensemble

Train multiple models with different random seeds and create an ensemble.
Ensemble averaging improves accuracy by reducing variance.
"""

import argparse
import os
import sys
import subprocess
from pathlib import Path

DETECT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def train_single_model(seed, data, model, epochs, imgsz, batch, device, project):
    """Train a single model with a specific seed."""
    name = f"ensemble_seed{seed}"
    cmd = [
        "python3", "train_layer2_improved.py",
        "--data", data,
        "--model", model,
        "--epochs", str(epochs),
        "--imgsz", str(imgsz),
        "--batch", str(batch),
        "--device", device,
        "--project", project,
        "--name", name
    ]
    print(f"\n=== Training model with seed={seed} ===")
    print(f"Command: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=DETECT_DIR)
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description="Train Layer 2 ensemble")
    parser.add_argument("--data", default=os.path.join(DETECT_DIR, "laser_rx/dataset.yaml"),
                        help="Dataset YAML path")
    parser.add_argument("--model", default="yolov8s.pt",
                        help="Base model (yolov8n/s/m/l/x)")
    parser.add_argument("--epochs", type=int, default=300,
                        help="Training epochs per model")
    parser.add_argument("--imgsz", type=int, default=256,
                        help="Training image size")
    parser.add_argument("--batch", type=int, default=32,
                        help="Batch size")
    parser.add_argument("--device", default="0")
    parser.add_argument("--project", default=os.path.join(DETECT_DIR, "model/layer2"),
                        help="Output directory")
    parser.add_argument("--n-models", type=int, default=3,
                        help="Number of models in ensemble")
    parser.add_argument("--seeds", nargs='+', type=int, default=[42, 123, 456],
                        help="Random seeds for each model")
    args = parser.parse_args()

    if len(args.seeds) != args.n_models:
        print(f"Warning: --seeds has {len(args.seeds)} values but --n-models={args.n_models}")
        print(f"Using first {args.n_models} seeds")
        args.seeds = args.seeds[:args.n_models]

    print("=== Ensemble Training ===")
    print(f"Number of models: {args.n_models}")
    print(f"Seeds: {args.seeds}")
    print(f"Epochs per model: {args.epochs}")

    success_count = 0
    for i, seed in enumerate(args.seeds):
        if train_single_model(seed, args.data, args.model, args.epochs,
                               args.imgsz, args.batch, args.device, args.project):
            success_count += 1
            print(f"✓ Model {i+1}/{args.n_models} trained successfully")
        else:
            print(f"✗ Model {i+1}/{args.n_models} training failed")

    print(f"\n=== Ensemble Training Complete ===")
    print(f"Successfully trained: {success_count}/{args.n_models} models")
    print("\n=== Ensemble Inference ===")
    print("To use ensemble for inference, modify cascade_detector.cpp to:")
    print("1. Load all ensemble models")
    print("2. Run inference with each model")
    print("3. Average the predictions (NMS across all results)")
    print("\nModel paths:")
    for seed in args.seeds:
        model_path = os.path.join(args.project, f"ensemble_seed{seed}", "weights", "best.pt")
        print(f"  {model_path}")


if __name__ == "__main__":
    main()
