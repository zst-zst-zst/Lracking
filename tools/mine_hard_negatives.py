#!/usr/bin/env python3
"""硬负样本挖掘 (Hard Negative Mining) for layer2 laser_rx detector.

工作流:
  1. 喂入「确认无敌方 laser_rx 的视频/图片」(空场景, 友方场景等)
  2. 用当前 yolo11n best.pt 跑推理
  3. 凡是被检测到的 → 全部都是 false positive
  4. 把这些帧及其检测框周边 crop 当作「负样本」加入训练集
     - 选项 A (推荐): 整帧加入, 标注文件留空 (yolo 格式: 空 .txt)
     - 选项 B: 只 crop FP 周围, 同样空 .txt
  5. 用扩充后的数据集 fine-tune 同一个 yolo11n

使用:
  # 干跑 (只看会挖出多少负样本, 不写入)
  python tools/mine_hard_negatives.py --videos path/to/empty_scene.mp4 --dry-run

  # 实际生成 (整帧模式, 默认)
  python tools/mine_hard_negatives.py \\
      --videos vid1.mp4 vid2.mp4 \\
      --conf 0.30 --sample-every 5 \\
      --max-per-video 200

  # 之后直接 fine-tune
  python tools/mine_hard_negatives.py --finetune --epochs 50
"""
from __future__ import annotations

import argparse
import os
import sys
import shutil
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = ROOT / "src/detect/model/layer2/weights/best.pt"
DEFAULT_DATASET = ROOT / "src/detect/model/layer2/dataset/merged"
NEG_TAG = "neg_"  # 文件名前缀, 方便区分


def collect_frames_from_video(video: Path, sample_every: int,
                              max_per_video: int) -> List[Tuple[int, np.ndarray]]:
    """每 sample_every 帧抽 1 帧, 最多 max_per_video 张."""
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        print(f"  ✗ 无法打开 {video}")
        return []
    frames = []
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % sample_every == 0:
            frames.append((idx, frame))
            if len(frames) >= max_per_video:
                break
        idx += 1
    cap.release()
    return frames


def run_inference(model: YOLO, frames: List[Tuple[int, np.ndarray]],
                  conf: float, imgsz: int) -> List[Tuple[int, np.ndarray, int]]:
    """返回 (frame_idx, frame, num_fp_detections)."""
    out = []
    for fi, frame in frames:
        results = model.predict(frame, conf=conf, imgsz=imgsz, verbose=False)
        n = len(results[0].boxes) if results and results[0].boxes is not None else 0
        out.append((fi, frame, n))
    return out


def write_negatives(detections: List[Tuple[int, np.ndarray, int]],
                    video_name: str, dataset_dir: Path,
                    only_with_fp: bool):
    """把帧写入训练集. yolo 格式空标注 .txt = 整帧无目标."""
    img_dir = dataset_dir / "train/images"
    lbl_dir = dataset_dir / "train/labels"
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    written = 0
    for fi, frame, n in detections:
        if only_with_fp and n == 0:
            continue
        stem = f"{NEG_TAG}{video_name}_f{fi:06d}"
        cv2.imwrite(str(img_dir / f"{stem}.jpg"), frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        # 空标注 = 该帧没有任何目标 (训练时强制模型把所有候选都判成 background)
        (lbl_dir / f"{stem}.txt").write_text("", encoding="utf-8")
        written += 1
    return written


def cmd_mine(args):
    model = YOLO(str(args.model))
    print(f"模型: {args.model}")
    print(f"conf 阈值: {args.conf}  imgsz: {args.imgsz}")
    print(f"采样: 每 {args.sample_every} 帧, 每视频最多 {args.max_per_video}")

    total_frames = 0
    total_fp = 0
    total_written = 0

    for v in args.videos:
        v = Path(v)
        if not v.exists():
            print(f"  ✗ 不存在: {v}")
            continue
        print(f"\n→ {v.name}")
        frames = collect_frames_from_video(v, args.sample_every, args.max_per_video)
        print(f"  抽取 {len(frames)} 帧")
        dets = run_inference(model, frames, args.conf, args.imgsz)
        n_fp_frames = sum(1 for _, _, n in dets if n > 0)
        n_fp_total = sum(n for _, _, n in dets)
        print(f"  误检帧: {n_fp_frames}/{len(frames)} ({n_fp_total} 个 FP bbox)")
        total_frames += len(frames)
        total_fp += n_fp_total

        if not args.dry_run:
            w = write_negatives(dets, v.stem, Path(args.dataset),
                                only_with_fp=args.only_with_fp)
            total_written += w
            print(f"  已写入 {w} 张到 {args.dataset}/train/")

    print("\n========================================")
    print(f"总计: {total_frames} 帧采样, {total_fp} 个 FP bbox")
    if args.dry_run:
        print("(dry-run, 未写入. 去掉 --dry-run 实际生成.)")
    else:
        print(f"已加入训练集 {total_written} 张负样本")
        print(f"\n下一步:  python tools/mine_hard_negatives.py --finetune")


def cmd_finetune(args):
    model = YOLO(str(args.model))
    data_yaml = Path(args.dataset) / "data.yaml"
    print(f"Fine-tune 起点: {args.model}")
    print(f"数据集: {data_yaml}")
    print(f"epochs={args.epochs}, batch={args.batch}, imgsz={args.imgsz}")

    project_dir = ROOT / "src/detect/model/layer2"
    name = "train_yolo11n_with_negatives"
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(project_dir),
        name=name,
        exist_ok=True,
        amp=False,  # 跳过 AMP 预检 (网络访问 github 会 hang); 训练本身仍用 fp32
        workers=4,
        # 保守 lr (fine-tune): 1e-3 → 1e-4
        lr0=1e-4,
        cos_lr=True,
        # 强增强, 推动模型理解负样本
        mosaic=1.0,
        mixup=0.15,
        copy_paste=0.3,
        hsv_h=0.02, hsv_s=0.7, hsv_v=0.5,
        # 早停
        patience=20,
        # 优化器
        optimizer="AdamW",
        weight_decay=5e-4,
    )
    print(f"\n训练完成. 最佳权重: {project_dir/name}/weights/best.pt")
    print("接下来: 重新导出 ONNX + TRT engine (按 README 流程)")


def cmd_from_layer1(args):
    """从 layer1 plane 数据集裁 ROI 作负样本.

    layer1 标注的每个 plane bbox, 按 cascade 的 ROI 规则(top_extend/bottom_extend/width_scale)
    往上裁出与部署时 layer2 完全同分布的 ROI. 因为 layer1 图都没 laser_rx,
    这些 ROI 全是真负样本, 比全图误检挖掘信号更强.
    """
    src_root = Path(args.layer1_dataset)
    img_root = src_root / "images"
    lbl_root = src_root / "labels"
    if not img_root.is_dir() or not lbl_root.is_dir():
        print(f"✗ layer1 数据集结构不对: {src_root}")
        return

    dst_img = Path(args.dataset) / "train/images"
    dst_lbl = Path(args.dataset) / "train/labels"
    dst_img.mkdir(parents=True, exist_ok=True)
    dst_lbl.mkdir(parents=True, exist_ok=True)

    top_ext = args.roi_top_extend
    bot_ext = args.roi_bottom_extend
    w_scale = args.roi_width_scale
    min_size = args.roi_min_size

    total_imgs = 0
    total_rois = 0
    total_written = 0
    written_per_split = {"train": 0, "val": 0, "test": 0}

    for split in ["train", "val", "test"]:
        img_dir = img_root / split
        lbl_dir = lbl_root / split
        if not img_dir.is_dir():
            continue
        files = sorted(list(img_dir.glob("*.jpg")) + list(img_dir.glob("*.png")))
        if args.layer1_max > 0:
            files = files[:args.layer1_max]
        print(f"\n→ layer1/{split}: {len(files)} 张")
        for f in files:
            total_imgs += 1
            lbl_f = lbl_dir / (f.stem + ".txt")
            if not lbl_f.exists():
                continue
            img = cv2.imread(str(f))
            if img is None:
                continue
            H, W = img.shape[:2]

            for line_idx, line in enumerate(lbl_f.read_text().strip().splitlines()):
                parts = line.strip().split()
                if len(parts) < 5:
                    continue
                try:
                    cx, cy, bw, bh = [float(x) for x in parts[1:5]]
                except ValueError:
                    continue
                # yolo 归一化 → 像素
                bx1 = (cx - bw / 2) * W
                by1 = (cy - bh / 2) * H
                bx2 = (cx + bw / 2) * W
                by2 = (cy + bh / 2) * H
                bbw = bx2 - bx1
                bbh = by2 - by1
                # 模拟 cascade ROI: 向上扩, 向下少量扩, 水平缩放
                rx1 = int(bx1 - bbw * (w_scale - 1.0) / 2)
                rx2 = int(bx2 + bbw * (w_scale - 1.0) / 2)
                ry1 = int(by1 - bbh * top_ext)
                ry2 = int(by1 + bbh * bot_ext)
                rx1 = max(0, rx1); ry1 = max(0, ry1)
                rx2 = min(W, rx2); ry2 = min(H, ry2)
                if rx2 - rx1 < min_size or ry2 - ry1 < min_size:
                    continue
                total_rois += 1
                if args.dry_run:
                    continue
                roi = img[ry1:ry2, rx1:rx2]
                stem = f"{NEG_TAG}l1_{split}_{f.stem}_b{line_idx}"
                cv2.imwrite(str(dst_img / f"{stem}.jpg"), roi,
                            [cv2.IMWRITE_JPEG_QUALITY, 90])
                (dst_lbl / f"{stem}.txt").write_text("", encoding="utf-8")
                total_written += 1
                written_per_split[split] += 1

    print("\n========================================")
    print(f"处理 {total_imgs} 张 layer1 图, 生成 {total_rois} 个候选 ROI")
    if args.dry_run:
        print("(dry-run, 未写入)")
    else:
        print(f"已写入 {total_written} 张 ROI 到 {args.dataset}/train/")
        for k, v in written_per_split.items():
            print(f"  来源 layer1/{k}: {v}")


def cmd_clean(args):
    """删掉之前挖掘加入的负样本 (NEG_TAG 前缀)."""
    train_img = Path(args.dataset) / "train/images"
    train_lbl = Path(args.dataset) / "train/labels"
    n = 0
    for f in list(train_img.glob(f"{NEG_TAG}*.jpg")) + \
             list(train_lbl.glob(f"{NEG_TAG}*.txt")):
        f.unlink()
        n += 1
    print(f"已删除 {n} 个负样本文件")


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                  description=__doc__)
    ap.add_argument("--model", default=str(DEFAULT_MODEL))
    ap.add_argument("--dataset", default=str(DEFAULT_DATASET))
    ap.add_argument("--imgsz", type=int, default=640)

    # mine 子命令 (默认)
    ap.add_argument("--videos", nargs="+", default=[],
                    help="待挖掘的视频路径 (可多个)")
    ap.add_argument("--conf", type=float, default=0.30,
                    help="检测置信度阈值. 越低挖到的 FP 越多")
    ap.add_argument("--sample-every", type=int, default=5,
                    help="每 N 帧采样 1 张")
    ap.add_argument("--max-per-video", type=int, default=300)
    ap.add_argument("--only-with-fp", action="store_true",
                    help="只保留有误检的帧 (更精炼)")
    ap.add_argument("--dry-run", action="store_true")

    # finetune
    ap.add_argument("--finetune", action="store_true")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--device", default="0")

    # clean
    ap.add_argument("--clean", action="store_true",
                    help="删除之前挖掘的所有负样本")

    # from-layer1: 从 layer1 plane 数据集裁 ROI 当负样本
    ap.add_argument("--from-layer1", action="store_true",
                    help="从 layer1 plane 数据集按 bbox 裁 ROI, 加入训练集作负样本")
    ap.add_argument("--layer1-dataset",
                    default=str(ROOT / "src/detect/model/layer1/dataset/plane"),
                    help="layer1 数据集根目录 (含 images/{train,val,test} 和 labels/...)")
    ap.add_argument("--layer1-max", type=int, default=0,
                    help="每个 split 最多采样多少张 (0=全部)")
    ap.add_argument("--roi-top-extend", type=float, default=1.0,
                    help="与 cascade 一致: bbox 顶部向上扩的倍数")
    ap.add_argument("--roi-bottom-extend", type=float, default=0.3,
                    help="与 cascade 一致: bbox 顶部向下扩的倍数")
    ap.add_argument("--roi-width-scale", type=float, default=1.2,
                    help="与 cascade 一致: ROI 水平缩放比")
    ap.add_argument("--roi-min-size", type=int, default=64,
                    help="ROI 边长小于此值则跳过")

    args = ap.parse_args()

    if args.clean:
        cmd_clean(args)
    elif args.finetune:
        cmd_finetune(args)
    elif args.from_layer1:
        cmd_from_layer1(args)
    elif args.videos:
        cmd_mine(args)
    else:
        ap.print_help()
        print("\n请选一个: --videos / --from-layer1 / --finetune / --clean")
        sys.exit(1)


if __name__ == "__main__":
    main()
