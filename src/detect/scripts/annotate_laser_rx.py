#!/usr/bin/env python3
"""Simple OpenCV GUI annotation tool for Laser RX + Light Strip in Layer 2 crops.

Controls:
  - Left click + drag: draw bounding box
  - '1': set current class to laser_rx (class 0, green)  [default]
  - '2': set current class to light_strip (class 1, yellow)
  - 'n' / Right arrow: next image
  - 'p' / Left arrow: previous image
  - 'r': reset all annotations on current image
  - 'd': delete annotation file for current image
  - 's': save and continue
  - 'q': save and quit
  - 'z': zoom toggle (2x)

Output: YOLO format labels (class_id cx cy w h) normalized to [0,1].
  Class 0 = laser_rx (target)
  Class 1 = light_strip (distractor — train model to distinguish from laser_rx)

Labeling light_strip as a separate class teaches the model NOT to confuse
the drone's LED navigation light strips with the laser detection module.
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np


CLASS_NAMES = {0: "laser_rx", 1: "light_strip"}
CLASS_COLORS = {0: (0, 255, 0), 1: (0, 255, 255)}  # green, yellow


class Annotator:
    def __init__(self, images_dir, labels_dir):
        self.images_dir = Path(images_dir)
        self.labels_dir = Path(labels_dir)
        self.labels_dir.mkdir(parents=True, exist_ok=True)

        extensions = {".jpg", ".jpeg", ".png", ".bmp"}
        self.image_files = sorted([
            f for f in self.images_dir.iterdir()
            if f.suffix.lower() in extensions
        ])
        if not self.image_files:
            print(f"No images found in {images_dir}")
            sys.exit(1)

        self.idx = 0
        self.drawing = False
        self.start_pt = None
        self.end_pt = None
        self.boxes = []        # list of (class_id, x1, y1, x2, y2)
        self.current_class = 0  # 0=laser_rx, 1=light_strip
        self.zoom = False
        self.window_name = "Annotate Laser RX"

        # Skip to first unannotated image
        for i, f in enumerate(self.image_files):
            label_path = self.labels_dir / (f.stem + ".txt")
            if not label_path.exists() or label_path.stat().st_size == 0:
                self.idx = i
                break

    def load_existing_labels(self, img_w, img_h):
        """Load existing YOLO labels if present. Returns list of (cls, x1, y1, x2, y2)."""
        label_path = self.labels_dir / (self.image_files[self.idx].stem + ".txt")
        boxes = []
        if not label_path.exists():
            return boxes
        for line in label_path.read_text().strip().splitlines():
            parts = line.strip().split()
            if len(parts) < 5:
                continue
            try:
                cls = int(parts[0])
                cx, cy, w, h = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
                x1 = int((cx - w / 2) * img_w)
                y1 = int((cy - h / 2) * img_h)
                x2 = int((cx + w / 2) * img_w)
                y2 = int((cy + h / 2) * img_h)
                boxes.append((cls, x1, y1, x2, y2))
            except ValueError:
                continue
        return boxes

    def save_labels(self, boxes, img_w, img_h):
        """Save all boxes in YOLO format."""
        label_path = self.labels_dir / (self.image_files[self.idx].stem + ".txt")
        if not boxes:
            label_path.write_text("")
            return
        lines = []
        for cls, x1, y1, x2, y2 in boxes:
            cx = ((x1 + x2) / 2.0) / img_w
            cy = ((y1 + y2) / 2.0) / img_h
            w = abs(x2 - x1) / img_w
            h = abs(y2 - y1) / img_h
            lines.append(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        label_path.write_text("\n".join(lines) + "\n")

    def count_annotated(self):
        count = 0
        for f in self.image_files:
            lp = self.labels_dir / (f.stem + ".txt")
            if lp.exists() and lp.stat().st_size > 0:
                count += 1
        return count

    def draw_display(self, img):
        display = img.copy()
        h, w = display.shape[:2]

        # Draw crosshair at center
        cv2.line(display, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2),
                 (128, 128, 128), 1)
        cv2.line(display, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20),
                 (128, 128, 128), 1)

        # Draw all saved boxes
        for i, (cls, x1, y1, x2, y2) in enumerate(self.boxes):
            color = CLASS_COLORS.get(cls, (255, 255, 255))
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            label = f"[{i}] {CLASS_NAMES.get(cls, '?')}"
            cv2.putText(display, label, (x1, max(y1 - 5, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # Draw box being drawn
        if self.drawing and self.start_pt and self.end_pt:
            color = CLASS_COLORS.get(self.current_class, (255, 255, 255))
            cv2.rectangle(display, self.start_pt, self.end_pt, color, 1)

        # Status bar
        annotated = self.count_annotated()
        cls_name = CLASS_NAMES.get(self.current_class, "?")
        cls_color = CLASS_COLORS.get(self.current_class, (255, 255, 255))
        status = (f"[{self.idx + 1}/{len(self.image_files)}] "
                  f"{self.image_files[self.idx].name}  "
                  f"Boxes: {len(self.boxes)}  "
                  f"Annotated: {annotated}/{len(self.image_files)}  "
                  f"{'ZOOM ' if self.zoom else ''}")
        cv2.putText(display, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        # Current class indicator
        cv2.putText(display, f"Class: {cls_name}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, cls_color, 2)
        cv2.putText(display,
                    "1:laser_rx 2:light_strip | n:next p:prev r:reset u:undo d:del q:quit",
                    (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (200, 200, 200), 1)

        if self.zoom:
            display = cv2.resize(display, (w * 2, h * 2),
                                 interpolation=cv2.INTER_LINEAR)
        return display

    def mouse_callback(self, event, x, y, flags, param):
        if self.zoom:
            x //= 2
            y //= 2

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_pt = (x, y)
            self.end_pt = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.end_pt = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drawing:
            self.drawing = False
            self.end_pt = (x, y)
            x1 = min(self.start_pt[0], self.end_pt[0])
            y1 = min(self.start_pt[1], self.end_pt[1])
            x2 = max(self.start_pt[0], self.end_pt[0])
            y2 = max(self.start_pt[1], self.end_pt[1])
            if x2 - x1 > 3 and y2 - y1 > 3:
                self.boxes.append((self.current_class, x1, y1, x2, y2))

    def run(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        while True:
            img_path = self.image_files[self.idx]
            img = cv2.imread(str(img_path))
            if img is None:
                print(f"Failed to read {img_path}")
                self.idx = (self.idx + 1) % len(self.image_files)
                continue

            h, w = img.shape[:2]
            self.boxes = self.load_existing_labels(w, h)

            while True:
                display = self.draw_display(img)
                cv2.imshow(self.window_name, display)
                key = cv2.waitKey(30) & 0xFF

                if key == ord('q'):
                    self.save_labels(self.boxes, w, h)
                    print(f"Saved. Annotated: {self.count_annotated()}/{len(self.image_files)}")
                    cv2.destroyAllWindows()
                    return

                elif key == ord('n') or key == 83:  # 'n' or right arrow
                    self.save_labels(self.boxes, w, h)
                    self.idx = (self.idx + 1) % len(self.image_files)
                    break

                elif key == ord('p') or key == 81:  # 'p' or left arrow
                    self.save_labels(self.boxes, w, h)
                    self.idx = (self.idx - 1) % len(self.image_files)
                    break

                elif key == ord('1'):
                    self.current_class = 0  # laser_rx

                elif key == ord('2'):
                    self.current_class = 1  # light_strip

                elif key == ord('r'):
                    self.boxes = []
                    self.drawing = False

                elif key == ord('u'):
                    if self.boxes:
                        self.boxes.pop()

                elif key == ord('d'):
                    self.boxes = []
                    self.save_labels([], w, h)

                elif key == ord('s'):
                    self.save_labels(self.boxes, w, h)
                    print(f"Saved. Annotated: {self.count_annotated()}/{len(self.image_files)}")

                elif key == ord('z'):
                    self.zoom = not self.zoom


def main():
    parser = argparse.ArgumentParser(description="Annotate Laser RX in Layer 2 crops")
    parser.add_argument("--images", required=True, help="Crop images directory")
    parser.add_argument("--labels", required=True, help="Output labels directory")
    args = parser.parse_args()

    annotator = Annotator(args.images, args.labels)
    print(f"Found {len(annotator.image_files)} images")
    print(f"Starting at image {annotator.idx + 1}")
    annotator.run()


if __name__ == "__main__":
    main()
