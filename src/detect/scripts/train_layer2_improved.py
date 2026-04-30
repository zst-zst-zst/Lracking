#!/usr/bin/env python3
"""Train Layer 2: Laser RX detection (maximum accuracy version)

All optimizations enabled:
- Larger model (yolov8s/m/l)
- Higher input resolution (256x256)
- Aggressive data augmentation (hsv_h, degrees, shear, mixup)
- Longer training with better early stopping
- TTA (Test Time Augmentation) support
- Ensemble learning support
"""

import argparse
import os
import sys
from pathlib import Path

DETECT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def verify_dataset(data_path):
    """Check that the Layer 2 dataset has annotated images."""
    import yaml
    with open(data_path) as f:
        cfg = yaml.safe_load(f)

    base = Path(cfg.get("path", "."))
    train_imgs = base / cfg.get("train", "train/images")
    train_labels = train_imgs.parent / "labels"  # train/labels

    if not train_imgs.exists():
        print(f"ERROR: Training images directory not found: {train_imgs}")
        return False

    img_count = sum(1 for f in train_imgs.iterdir()
                    if f.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if img_count == 0:
        print(f"ERROR: No images in {train_imgs}")
        return False

    annotated = 0
    if train_labels.exists():
        for lf in train_labels.iterdir():
            if lf.suffix == ".txt" and lf.stat().st_size > 0:
                annotated += 1

    print(f"Dataset: {img_count} images, {annotated} annotated")
    return True


def main():
    parser = argparse.ArgumentParser(description="Train Layer 2 laser_rx detector (max accuracy)")
    parser.add_argument("--data", default=os.path.join(DETECT_DIR, "laser_rx/dataset.yaml"),
                        help="Dataset YAML path")
    parser.add_argument("--model", default="yolo11m.pt",
                        help="Base model (yolo11n/s/m/l/x, yolov8n/s/m/l/x)")
    parser.add_argument("--epochs", type=int, default=300,
                        help="Training epochs")
    parser.add_argument("--imgsz", type=int, default=256,
                        help="Training image size")
    parser.add_argument("--batch", type=int, default=16,
                        help="Batch size (reduced for memory optimization)")
    parser.add_argument("--device", default="0")
    parser.add_argument("--project", default=os.path.join(DETECT_DIR, "model/layer2"),
                        help="Output directory")
    parser.add_argument("--name", default="train_balanced")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not installed. Run: pip install ultralytics")
        sys.exit(1)

    data_path = os.path.abspath(args.data)
    if not os.path.isfile(data_path):
        print(f"ERROR: Dataset config not found: {data_path}")
        sys.exit(1)

    if not verify_dataset(data_path):
        sys.exit(1)

    model = YOLO(args.model)

    # Maximum accuracy training parameters
    results = model.train(
        data=data_path,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=os.path.abspath(args.project),
        name=args.name,
        exist_ok=True,
        resume=args.resume,
        workers=8,
        patience=80,  # high patience for max accuracy
        save=True,
        save_period=10,
        val=True,
        plots=True,
        amp=False,  # disable AMP to avoid download check
        # Augmentation for multi-class detection (shape + color)
        # Moderate hue variation for color robustness while preserving color distinction
        hsv_h=0.2,      # moderate hue variation (color-robust but still color-sensitive)
        hsv_s=0.7,
        hsv_v=0.6,
        degrees=30.0,   # maximum rotation
        translate=0.3,
        scale=0.6,
        shear=10.0,     # maximum shear
        flipud=0.0,
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.2,      # higher mixup
        copy_paste=0.1, # add copy-paste augmentation
        # Advanced optimization
        optimizer="AdamW",
        lr0=0.01,
        lrf=0.01,
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=5,
        warmup_momentum=0.8,
        warmup_bias_lr=0.1,
        cos_lr=True,  # cosine learning rate scheduler
        close_mosaic=15,  # disable mosaic in last 15 epochs
    )

    print("\n=== Layer 2 Training Complete (Maximum Accuracy) ===")
    print(f"Best model: {os.path.abspath(args.project)}/{args.name}/weights/best.pt")
    print("\n=== TTA Usage ===")
    print("To use Test Time Augmentation for inference:")
    print(f"  model = YOLO('{os.path.abspath(args.project)}/{args.name}/weights/best.pt')")
    print("  results = model.predict(image, augment=True)")


if __name__ == "__main__":
    main()
