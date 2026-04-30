#!/usr/bin/env python3
"""
通过复制 purple 样本来平衡数据集
将 purple 样本复制 2 倍，使其数量接近 blue 和 red
"""

import os
import shutil
from pathlib import Path

DETECT_DIR = Path(__file__).parent.parent
DATASET_DIR = DETECT_DIR / "laser_rx"

def balance_purple_samples():
    """复制 purple 样本以平衡数据集"""
    splits = ["train", "valid", "test"]

    for split in splits:
        img_dir = DATASET_DIR / split / "images"
        lbl_dir = DATASET_DIR / split / "labels"

        # 找到所有 purple 样本 (类别 1)
        purple_imgs = []
        for lbl_file in lbl_dir.glob("*.txt"):
            with open(lbl_file, 'r') as f:
                first_line = f.readline().strip()
                if first_line.startswith('1 '):
                    img_file = img_dir / f"{lbl_file.stem}.jpg"
                    if img_file.exists():
                        purple_imgs.append((img_file, lbl_file))

        print(f"{split}: 找到 {len(purple_imgs)} 个 purple 样本")

        # 复制 2 倍
        for i, (img_file, lbl_file) in enumerate(purple_imgs):
            for copy_idx in range(2):  # 复制 2 次
                new_img_name = f"{img_file.stem}_copy{copy_idx+1}{img_file.suffix}"
                new_lbl_name = f"{lbl_file.stem}_copy{copy_idx+1}{lbl_file.suffix}"

                shutil.copy(img_file, img_dir / new_img_name)
                shutil.copy(lbl_file, lbl_dir / new_lbl_name)

        print(f"{split}: 复制了 {len(purple_imgs) * 2} 个 purple 样本")

    # 统计结果
    print("\n=== 平衡后统计 ===")
    for split in ["train", "valid", "test"]:
        img_dir = DATASET_DIR / split / "images"
        lbl_dir = DATASET_DIR / split / "labels"
        img_count = len(list(img_dir.glob("*.jpg")))

        # 统计各类别
        class_counts = {0: 0, 1: 0, 2: 0}
        for lbl_file in lbl_dir.glob("*.txt"):
            with open(lbl_file, 'r') as f:
                first_line = f.readline().strip()
                if first_line:
                    cls = int(first_line.split()[0])
                    class_counts[cls] += 1

        print(f"{split}: {img_count} images")
        print(f"  blue (0): {class_counts[0]}")
        print(f"  purple (1): {class_counts[1]}")
        print(f"  red (2): {class_counts[2]}")

if __name__ == "__main__":
    print("=== 平衡 purple 样本 ===")
    balance_purple_samples()
    print("\n✓ 平衡完成")
