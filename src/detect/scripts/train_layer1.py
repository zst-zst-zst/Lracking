#!/usr/bin/env python3
"""Train Layer 1: Plane (drone) detection using YOLOv8.

Uses the existing plane dataset at /home/zst/Tracking/plane/.
"""

import argparse
import os
import sys

# detect/ directory (parent of scripts/)
DETECT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

def main():
    parser = argparse.ArgumentParser(description="Train Layer 1 plane detector")
    parser.add_argument("--data", default=os.path.join(DETECT_DIR, "plane/dataset.yaml"),
                        help="Dataset YAML path")
    parser.add_argument("--model", default="yolov8n.pt",
                        help="Base model (yolov8n/s/m/l/x)")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640,
                        help="Training image size")
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0", help="CUDA device")
    parser.add_argument("--project", default=os.path.join(DETECT_DIR, "model/layer1"),
                        help="Output directory")
    parser.add_argument("--name", default="train", help="Run name")
    parser.add_argument("--resume", action="store_true",
                        help="Resume from last checkpoint")
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

    model = YOLO(args.model)

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
        # Performance tuning
        workers=8,
        patience=20,
        save=True,
        save_period=10,
        val=True,
        plots=True,
        # Augmentation suitable for aerial targets
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        degrees=10.0,
        translate=0.1,
        scale=0.5,
        flipud=0.5,      # vertical flip useful for aerial views
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.1,
    )

    print("\n=== Layer 1 Training Complete ===")
    print(f"Best model: {os.path.abspath(args.project)}/{args.name}/weights/best.pt")


if __name__ == "__main__":
    main()
