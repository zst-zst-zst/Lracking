#!/usr/bin/env python3
"""SR Phase 1: 从 merged 数据集抽出 laser_rx 高清 patch (HR samples).

策略:
  1. 遍历所有图片+标签
  2. 解析每个 laser_rx 多边形 → 紧致 bbox + 10% margin
  3. 过滤 bbox 短边 < HR_MIN (默认 128px)
  4. 过滤模糊 (Laplacian variance < blur_threshold)
  5. 过滤曝光异常 (mean < 20 或 mean > 235)
  6. 保存到 sr_dataset/hr/<class>/<id>.png

输出格式: 每个样本是该类别的清晰 laser_rx 裁剪图 (变长尺寸)
"""

import os, sys, cv2, argparse, random
from pathlib import Path
import numpy as np

random.seed(42)


def parse_polygon_label(line: str):
    """解析 YOLO segmentation 行: cls x1 y1 x2 y2 ... → (cls, xs, ys)."""
    parts = line.strip().split()
    if len(parts) < 5:
        return None
    cls = int(parts[0])
    coords = [float(x) for x in parts[1:]]
    xs = coords[0::2]
    ys = coords[1::2]
    return cls, xs, ys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="/home/zst/Tracking/src/detect/model/layer2/dataset/merged")
    ap.add_argument("--dst", default="/home/zst/Tracking/src/detect/model/layer2/sr_dataset")
    ap.add_argument("--hr_min", type=int, default=128, help="HR 裁剪最短边阈值")
    ap.add_argument("--margin", type=float, default=0.1, help="bbox 外扩比例")
    ap.add_argument("--blur_threshold", type=float, default=80.0,
                    help="Laplacian variance 阈值 (低于=模糊, 丢弃)")
    args = ap.parse_args()

    src = Path(args.src)
    dst = Path(args.dst) / "hr"
    dst.mkdir(parents=True, exist_ok=True)
    class_names = ["blue", "purple", "red"]
    for c in class_names:
        (dst / c).mkdir(exist_ok=True)

    stats = {
        "total_labels": 0,
        "skipped_too_small": 0,
        "skipped_blurry": 0,
        "skipped_exposure": 0,
        "saved": {c: 0 for c in class_names},
    }

    sample_id = 0
    for split in ["train", "valid", "test"]:
        img_dir = src / split / "images"
        lbl_dir = src / split / "labels"
        if not img_dir.exists():
            continue
        for img_file in sorted(img_dir.iterdir()):
            if img_file.suffix.lower() not in (".jpg", ".jpeg", ".png", ".bmp"):
                continue
            lbl_file = lbl_dir / (img_file.stem + ".txt")
            if not lbl_file.exists():
                continue

            img = cv2.imread(str(img_file))
            if img is None:
                continue
            H, W = img.shape[:2]

            for line in lbl_file.read_text().strip().split("\n"):
                parsed = parse_polygon_label(line)
                if not parsed:
                    continue
                cls, xs, ys = parsed
                stats["total_labels"] += 1

                # bbox in pixel coords
                x1 = min(xs) * W
                x2 = max(xs) * W
                y1 = min(ys) * H
                y2 = max(ys) * H
                bw, bh = x2 - x1, y2 - y1

                # 过滤太小的
                if min(bw, bh) < args.hr_min:
                    stats["skipped_too_small"] += 1
                    continue

                # 加 margin
                mx = bw * args.margin
                my = bh * args.margin
                cx1 = max(0, int(x1 - mx))
                cy1 = max(0, int(y1 - my))
                cx2 = min(W, int(x2 + mx))
                cy2 = min(H, int(y2 + my))

                crop = img[cy1:cy2, cx1:cx2]
                if crop.size == 0:
                    continue

                # 模糊检测
                gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
                if laplacian_var < args.blur_threshold:
                    stats["skipped_blurry"] += 1
                    continue

                # 曝光检测
                mean_brightness = gray.mean()
                if mean_brightness < 20 or mean_brightness > 235:
                    stats["skipped_exposure"] += 1
                    continue

                # 保存
                cls_name = class_names[cls] if cls < len(class_names) else "unknown"
                out_name = f"{sample_id:06d}_{img_file.stem[:30]}.png"
                cv2.imwrite(str(dst / cls_name / out_name), crop,
                            [cv2.IMWRITE_PNG_COMPRESSION, 3])
                stats["saved"][cls_name] += 1
                sample_id += 1

    # 统计
    print("=" * 60)
    print(f"扫描标签总数: {stats['total_labels']}")
    print(f"过滤 < {args.hr_min}px:     {stats['skipped_too_small']}")
    print(f"过滤模糊 (lap_var<{args.blur_threshold}): {stats['skipped_blurry']}")
    print(f"过滤曝光异常:        {stats['skipped_exposure']}")
    print(f"保存样本:")
    total = 0
    for c, n in stats["saved"].items():
        print(f"  {c}: {n}")
        total += n
    print(f"  总计: {total}")
    print(f"\n输出目录: {dst}")
    if total < 500:
        print("⚠️  警告: HR 样本 < 500, SR 训练可能欠拟合, 考虑降低 hr_min 或 blur_threshold")


if __name__ == "__main__":
    main()
