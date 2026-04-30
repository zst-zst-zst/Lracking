#!/usr/bin/env python3
"""Validate the full two-layer cascade detection pipeline.

Runs Layer 1 (plane detection) then Layer 2 (laser_rx detection in ROI)
on a video or image directory, with visualization.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np


def is_light_strip(det_region, enemy_color="blue",
                   color_ratio_thresh=0.25, min_sat=80, min_val=80,
                   contour_elongation=2.5):
    """Classical CV light strip rejection with per-contour compactness analysis.

    IMPORTANT: Both the laser module and light strips glow in team color!
    - Laser module: compact blob (~72x50mm, φ42mm circular detection area)
    - Light strips: thin elongated lines (>1500mm, zigzag/V/parallelogram)

    Algorithm:
      1) HSV color mask for enemy team color
      2) Per-contour analysis: classify as "compact" or "elongated"
      3) If ANY compact contour exists → KEEP (laser module likely present)
      4) If ONLY elongated contours → REJECT as light strip

    Returns True if the region looks like a light strip (should be rejected).
    """
    if det_region is None or det_region.size == 0:
        return False

    hsv = cv2.cvtColor(det_region, cv2.COLOR_BGR2HSV)

    # Build color mask
    if enemy_color == "blue":
        mask = cv2.inRange(hsv, (90, min_sat, min_val), (130, 255, 255))
    else:
        mask1 = cv2.inRange(hsv, (0, min_sat, min_val), (15, 255, 255))
        mask2 = cv2.inRange(hsv, (165, min_sat, min_val), (180, 255, 255))
        mask = mask1 | mask2

    # Check enemy-color pixel ratio
    total_px = det_region.shape[0] * det_region.shape[1]
    color_px = int(cv2.countNonZero(mask))
    color_ratio = color_px / max(total_px, 1)

    if color_ratio < color_ratio_thresh:
        return False  # not enough team-colored pixels

    # Per-contour analysis
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    min_contour_area = total_px * 0.01  # at least 1% of bbox area
    has_compact = False
    has_elongated = False
    significant = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_contour_area:
            continue
        significant += 1
        if len(cnt) < 5:
            continue

        # Circularity: 4π * area / perimeter² (1.0 = perfect circle)
        perimeter = cv2.arcLength(cnt, True)
        circularity = (4.0 * np.pi * area / (perimeter * perimeter)) if perimeter > 0 else 0.0

        # MinAreaRect aspect ratio
        rect = cv2.minAreaRect(cnt)
        rw, rh = rect[1]
        aspect = max(rw, rh) / max(min(rw, rh), 1.0)

        # Compact: could be laser module (φ42mm, roughly circular)
        if circularity > 0.3 and aspect < 2.0:
            has_compact = True

        if aspect > contour_elongation:
            has_elongated = True

    # If compact blob exists → laser module likely present, KEEP
    if has_compact:
        return False

    # Only elongated colored contours → light strip, REJECT
    if has_elongated and significant > 0:
        return True

    return False


def compute_roi(bbox, img_w, img_h,
                top_extend=1.0, bottom_extend=0.3,
                width_scale=1.2, min_size=64):
    """Compute ROI above detected drone bbox."""
    x1, y1, x2, y2 = bbox
    bw = x2 - x1
    bh = y2 - y1
    cx = (x1 + x2) / 2.0

    roi_w = max(bw * width_scale, min_size)
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


def draw_results(img, planes, rois, laser_rxs):
    """Draw detection results on the image."""
    display = img.copy()

    for plane in planes:
        x1, y1, x2, y2 = [int(v) for v in plane["bbox"]]
        conf = plane["conf"]
        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(display, f"plane {conf:.2f}", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    for roi in rois:
        rx1, ry1, rx2, ry2 = roi
        cv2.rectangle(display, (rx1, ry1), (rx2, ry2), (255, 255, 0), 1)
        cv2.putText(display, "ROI", (rx1, ry1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    for lrx in laser_rxs:
        x1, y1, x2, y2 = [int(v) for v in lrx["bbox"]]
        conf = lrx["conf"]
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.drawMarker(display, (cx, cy), (0, 0, 255),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.putText(display, f"laser_rx {conf:.2f}", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return display


def main():
    parser = argparse.ArgumentParser(description="Validate cascade pipeline")
    parser.add_argument("--layer1", required=True, help="Layer 1 model path (.pt)")
    parser.add_argument("--layer2", required=True, help="Layer 2 model path (.pt)")
    parser.add_argument("--video", default=None, help="Video file path")
    parser.add_argument("--images", default=None, help="Image directory")
    parser.add_argument("--cam", type=int, default=None, help="Camera device ID")
    parser.add_argument("--layer1-conf", type=float, default=0.5)
    parser.add_argument("--layer2-conf", type=float, default=0.4)
    parser.add_argument("--top-extend", type=float, default=1.0)
    parser.add_argument("--bottom-extend", type=float, default=0.3)
    parser.add_argument("--enemy-side", default="right",
                        help="Enemy side: 'right' or 'left'")
    parser.add_argument("--enemy-x-thresh", type=float, default=0.5,
                        help="Normalized x threshold for enemy side")
    parser.add_argument("--max-aspect", type=float, default=2.5,
                        help="Max aspect ratio for laser_rx (reject light strips)")
    parser.add_argument("--width-scale", type=float, default=1.2)
    parser.add_argument("--enemy-color", default="blue",
                        help="Enemy team color: 'blue' or 'red'")
    parser.add_argument("--strip-reject", action="store_true", default=True,
                        help="Enable classical light strip rejection (HSV+contour)")
    parser.add_argument("--no-strip-reject", action="store_true",
                        help="Disable classical light strip rejection")
    parser.add_argument("--strip-color-thresh", type=float, default=0.25,
                        help="Min enemy-color pixel ratio to suspect light strip")
    parser.add_argument("--strip-min-sat", type=int, default=80,
                        help="HSV saturation threshold (0-255)")
    parser.add_argument("--strip-min-val", type=int, default=80,
                        help="HSV value threshold (0-255)")
    parser.add_argument("--strip-elongation", type=float, default=2.5,
                        help="Contour minAreaRect elongation threshold")
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not installed")
        sys.exit(1)

    print("Loading Layer 1 model...")
    model1 = YOLO(args.layer1)
    print("Loading Layer 2 model...")
    model2 = YOLO(args.layer2)

    # Setup input source
    frames = []
    if args.video:
        cap = cv2.VideoCapture(args.video)
    elif args.cam is not None:
        cap = cv2.VideoCapture(args.cam)
    elif args.images:
        from pathlib import Path
        exts = {".jpg", ".jpeg", ".png", ".bmp"}
        frames = sorted([
            str(f) for f in Path(args.images).iterdir()
            if f.suffix.lower() in exts
        ])
        cap = None
    else:
        print("ERROR: Provide --video, --cam, or --images")
        sys.exit(1)

    frame_idx = 0
    total_time_l1 = 0.0
    total_time_l2 = 0.0
    total_frames = 0
    total_planes = 0
    total_laser_rx = 0

    while True:
        # Get frame
        if cap:
            ret, img = cap.read()
            if not ret:
                break
        elif frame_idx < len(frames):
            img = cv2.imread(frames[frame_idx])
            frame_idx += 1
            if img is None:
                continue
        else:
            break

        h, w = img.shape[:2]
        total_frames += 1

        # Layer 1: detect planes
        t0 = time.perf_counter()
        results1 = model1.predict(
            source=img, conf=args.layer1_conf,
            device=args.device, verbose=False)
        t1 = time.perf_counter()
        total_time_l1 += (t1 - t0)

        planes = []
        if results1 and len(results1[0].boxes) > 0:
            for box in results1[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = (x1 + x2) / 2.0
                norm_cx = cx / w

                # Enemy/Friendly filtering
                if args.enemy_side == "right":
                    is_enemy = norm_cx > args.enemy_x_thresh
                else:
                    is_enemy = norm_cx < args.enemy_x_thresh
                if not is_enemy:
                    continue

                planes.append({
                    "bbox": (float(x1), float(y1), float(x2), float(y2)),
                    "conf": float(box.conf[0]),
                })
        total_planes += len(planes)

        # Layer 2: detect laser_rx in ROI above each plane
        rois = []
        laser_rxs = []
        t2 = time.perf_counter()
        for plane in planes:
            roi = compute_roi(
                plane["bbox"], w, h,
                top_extend=args.top_extend,
                bottom_extend=args.bottom_extend,
                width_scale=args.width_scale,
            )
            if roi is None:
                continue
            rois.append(roi)
            rx1, ry1, rx2, ry2 = roi
            roi_area = (rx2 - rx1) * (ry2 - ry1)
            crop = img[ry1:ry2, rx1:rx2]
            if crop.size == 0:
                continue

            results2 = model2.predict(
                source=crop, conf=args.layer2_conf,
                device=args.device, verbose=False)

            if results2 and len(results2[0].boxes) > 0:
                for box in results2[0].boxes:
                    cls_id = int(box.cls[0]) if box.cls is not None else 0
                    # Reject light_strip (class 1)
                    if cls_id != 0:
                        continue

                    lx1, ly1, lx2, ly2 = box.xyxy[0].cpu().numpy()
                    det_w = lx2 - lx1
                    det_h = ly2 - ly1

                    # Aspect ratio filter
                    aspect = max(det_w, det_h) / max(min(det_w, det_h), 1.0)
                    if aspect > args.max_aspect:
                        continue

                    # Area ratio filter
                    det_area = det_w * det_h
                    area_ratio = det_area / max(roi_area, 1.0)
                    if area_ratio < 0.002 or area_ratio > 0.5:
                        continue

                    # Classical light strip rejection (HSV + contour)
                    use_strip = args.strip_reject and not args.no_strip_reject
                    if use_strip:
                        bx1 = max(0, int(lx1))
                        by1 = max(0, int(ly1))
                        bx2 = min(crop.shape[1], int(lx2))
                        by2 = min(crop.shape[0], int(ly2))
                        if bx2 - bx1 > 2 and by2 - by1 > 2:
                            det_patch = crop[by1:by2, bx1:bx2]
                            if is_light_strip(
                                det_patch,
                                enemy_color=args.enemy_color,
                                color_ratio_thresh=args.strip_color_thresh,
                                min_sat=args.strip_min_sat,
                                min_val=args.strip_min_val,
                                contour_elongation=args.strip_elongation,
                            ):
                                continue  # rejected as light strip

                    # Map back to full-frame coords
                    laser_rxs.append({
                        "bbox": (float(lx1 + rx1), float(ly1 + ry1),
                                 float(lx2 + rx1), float(ly2 + ry1)),
                        "conf": float(box.conf[0]),
                    })
        t3 = time.perf_counter()
        total_time_l2 += (t3 - t2)
        total_laser_rx += len(laser_rxs)

        # Display
        if not args.no_show:
            display = draw_results(img, planes, rois, laser_rxs)

            fps_l1 = 1.0 / max(t1 - t0, 1e-6)
            total_ms = (t3 - t0) * 1000
            cv2.putText(display, f"L1: {(t1-t0)*1000:.1f}ms  L2: {(t3-t2)*1000:.1f}ms  "
                        f"Total: {total_ms:.1f}ms  Planes: {len(planes)}  "
                        f"LaserRX: {len(laser_rxs)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2)

            cv2.imshow("Cascade Validation", display)
            key = cv2.waitKey(1 if cap else 0) & 0xFF
            if key == ord('q') or key == 27:
                break
            if key == ord('n'):
                continue

    if cap:
        cap.release()
    cv2.destroyAllWindows()

    # Summary
    print(f"\n=== Cascade Validation Summary ===")
    print(f"Frames: {total_frames}")
    print(f"Planes detected: {total_planes}")
    print(f"Laser RX detected: {total_laser_rx}")
    if total_frames > 0:
        print(f"Avg Layer 1 time: {total_time_l1 / total_frames * 1000:.1f} ms")
        print(f"Avg Layer 2 time: {total_time_l2 / total_frames * 1000:.1f} ms")
        print(f"Avg total time: {(total_time_l1 + total_time_l2) / total_frames * 1000:.1f} ms")


if __name__ == "__main__":
    main()
