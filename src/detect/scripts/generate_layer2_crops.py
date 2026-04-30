#!/usr/bin/env python3
"""Generate Layer 2 training crops from Layer 1 plane detections.

For each image in the plane dataset:
1. Run Layer 1 model to detect drones
2. For each detection, compute the ROI above the drone (where Laser RX lives)
3. Save the cropped ROI image for Layer 2 annotation/training

The Laser RX module sits on a rigid bar ~350mm above the drone propeller center.
From the camera's perspective, it appears above (and sometimes overlapping) the
drone's bounding box. The ROI extends upward from the drone bbox top by roughly
1x the drone's bbox height, centered horizontally.
"""

import argparse
import os
import sys
import json
from pathlib import Path

import cv2
import numpy as np


def compute_roi(bbox, img_w, img_h,
                top_extend=1.0, bottom_extend=0.3,
                width_scale=1.2, min_size=64):
    """Compute the ROI above a detected drone bbox.

    Args:
        bbox: (x1, y1, x2, y2) of the drone detection in pixel coords
        img_w, img_h: full image dimensions
        top_extend: how far above bbox top to extend (fraction of bbox height)
        bottom_extend: how far below bbox top to include (fraction of bbox height)
        width_scale: horizontal scale relative to bbox width
        min_size: minimum ROI side length

    Returns:
        (rx1, ry1, rx2, ry2) clipped ROI coords, or None if too small
    """
    x1, y1, x2, y2 = bbox
    bw = x2 - x1
    bh = y2 - y1
    cx = (x1 + x2) / 2.0

    # ROI width: centered on drone, scaled
    roi_w = max(bw * width_scale, min_size)
    # ROI vertical: from (top - top_extend*bh) to (top + bottom_extend*bh)
    roi_top = y1 - top_extend * bh
    roi_bottom = y1 + bottom_extend * bh
    roi_h = roi_bottom - roi_top
    if roi_h < min_size:
        mid = (roi_top + roi_bottom) / 2.0
        roi_top = mid - min_size / 2.0
        roi_bottom = mid + min_size / 2.0

    rx1 = int(max(0, cx - roi_w / 2.0))
    ry1 = int(max(0, roi_top))
    rx2 = int(min(img_w, cx + roi_w / 2.0))
    ry2 = int(min(img_h, roi_bottom))

    if rx2 - rx1 < min_size // 2 or ry2 - ry1 < min_size // 2:
        return None
    return (rx1, ry1, rx2, ry2)


def main():
    parser = argparse.ArgumentParser(
        description="Generate Layer 2 training crops from Layer 1 detections")
    parser.add_argument("--model", required=True,
                        help="Layer 1 model path (e.g. model/layer1/train/weights/best.pt)")
    parser.add_argument("--images", required=True,
                        help="Source image directory (e.g. ../plane/images/train)")
    parser.add_argument("--output", required=True,
                        help="Output crop directory (e.g. dataset/layer2/images/train)")
    parser.add_argument("--labels-out", default=None,
                        help="Output labels directory (empty .txt per crop)")
    parser.add_argument("--conf", type=float, default=0.5,
                        help="Layer 1 confidence threshold")
    parser.add_argument("--top-extend", type=float, default=1.0,
                        help="ROI top extension (fraction of drone bbox height)")
    parser.add_argument("--bottom-extend", type=float, default=0.3,
                        help="ROI bottom extension below drone bbox top")
    parser.add_argument("--width-scale", type=float, default=1.2,
                        help="ROI width scale relative to drone bbox width")
    parser.add_argument("--min-size", type=int, default=64,
                        help="Minimum ROI side length")
    parser.add_argument("--save-meta", action="store_true",
                        help="Save metadata JSON for each crop (source image, ROI coords)")
    parser.add_argument("--max-images", type=int, default=0,
                        help="Max images to process (0=all)")
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not installed. Run: pip install ultralytics")
        sys.exit(1)

    model = YOLO(args.model)
    images_dir = Path(args.images)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    labels_out = None
    if args.labels_out:
        labels_out = Path(args.labels_out)
        labels_out.mkdir(parents=True, exist_ok=True)

    meta_dir = None
    if args.save_meta:
        meta_dir = output_dir.parent / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)

    extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
    image_files = sorted([
        f for f in images_dir.iterdir()
        if f.suffix.lower() in extensions
    ])
    if args.max_images > 0:
        image_files = image_files[:args.max_images]

    print(f"Processing {len(image_files)} images from {images_dir}")
    print(f"Output crops to {output_dir}")

    total_crops = 0
    total_detections = 0

    for idx, img_path in enumerate(image_files):
        if (idx + 1) % 100 == 0 or idx == 0:
            print(f"  [{idx+1}/{len(image_files)}] {img_path.name}")

        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  WARNING: Failed to read {img_path}")
            continue

        h, w = img.shape[:2]

        results = model.predict(
            source=img,
            conf=args.conf,
            device=args.device,
            verbose=False,
        )

        if not results or len(results[0].boxes) == 0:
            continue

        boxes = results[0].boxes
        for det_idx, box in enumerate(boxes):
            total_detections += 1
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])

            roi = compute_roi(
                (x1, y1, x2, y2), w, h,
                top_extend=args.top_extend,
                bottom_extend=args.bottom_extend,
                width_scale=args.width_scale,
                min_size=args.min_size,
            )
            if roi is None:
                continue

            rx1, ry1, rx2, ry2 = roi
            crop = img[ry1:ry2, rx1:rx2]
            if crop.size == 0:
                continue

            stem = img_path.stem
            crop_name = f"{stem}_d{det_idx}"
            crop_path = output_dir / f"{crop_name}.jpg"
            cv2.imwrite(str(crop_path), crop, [cv2.IMWRITE_JPEG_QUALITY, 95])
            total_crops += 1

            # Create empty label file (to be annotated later)
            if labels_out is not None:
                label_path = labels_out / f"{crop_name}.txt"
                label_path.touch()

            # Save metadata
            if meta_dir is not None:
                meta = {
                    "source": str(img_path),
                    "source_size": [w, h],
                    "plane_bbox": [float(x1), float(y1), float(x2), float(y2)],
                    "plane_conf": conf,
                    "roi": [rx1, ry1, rx2, ry2],
                    "crop_size": [rx2 - rx1, ry2 - ry1],
                }
                meta_path = meta_dir / f"{crop_name}.json"
                with open(meta_path, "w") as f:
                    json.dump(meta, f, indent=2)

    print(f"\nDone: {total_detections} plane detections → {total_crops} ROI crops")
    print(f"Crops saved to: {output_dir}")
    if labels_out:
        print(f"Empty labels saved to: {labels_out}")
    print("\nNext step: annotate laser_rx in the crops using:")
    print(f"  python3 scripts/annotate_laser_rx.py --images {output_dir} --labels {labels_out or output_dir.parent / 'labels' / output_dir.name}")


if __name__ == "__main__":
    main()
