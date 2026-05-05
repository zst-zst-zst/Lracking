"""SR 可视化对比: bicubic vs ESPCN.

策略:
  1. 从 sr_dataset/hr 取若干 HR 样本
  2. 下采样 ×2 → LR
  3. 三种方式上采样回 HR 尺寸:
       - bicubic (基线)
       - ESPCN
       - HR (ground truth)
  4. 拼图保存到 sr_compare.png
  5. 关键测试: 对真实"小目标 ROI" (从原始数据集中找 <128px 的样本) 也做 SR
"""
import sys, os, random
from pathlib import Path
import cv2
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(__file__))
from sr_model import ESPCN

random.seed(0)

CKPT = "/home/zst/Tracking/src/detect/model/sr/espcn_x2_best.pt"
HR_DIR = "/home/zst/Tracking/src/detect/model/layer2/sr_dataset/hr"
MERGED = "/home/zst/Tracking/src/detect/model/layer2/dataset/merged"
OUT = "/home/zst/Tracking/src/detect/model/sr/visualize"


def load_model():
    state = torch.load(CKPT, map_location="cpu", weights_only=False)
    m = ESPCN(scale=2)
    m.load_state_dict(state["model"])
    m.eval()
    return m.cuda()


def sr_infer(model, lr_bgr: np.ndarray) -> np.ndarray:
    """LR uint8 BGR → SR uint8 BGR."""
    lr = cv2.cvtColor(lr_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    t = torch.from_numpy(lr.transpose(2, 0, 1))[None].cuda()
    with torch.no_grad():
        sr = model(t)[0].cpu().numpy().transpose(1, 2, 0)
    sr = (sr * 255).clip(0, 255).astype(np.uint8)
    return cv2.cvtColor(sr, cv2.COLOR_RGB2BGR)


def make_compare_grid(model, samples, title_size=24):
    """每行: HR | LR(放大bicubic) | SR(ESPCN) | bicubic参考."""
    rows = []
    target_h = 128
    for idx, hr in enumerate(samples):
        H, W = hr.shape[:2]
        # 中心裁 256×256
        ps = min(256, min(H, W))
        cy, cx = H // 2, W // 2
        hr_crop = hr[cy - ps // 2:cy + ps // 2, cx - ps // 2:cx + ps // 2]
        ps = hr_crop.shape[0]
        # 下采样到 LR (ps/2)
        lr = cv2.resize(hr_crop, (ps // 2, ps // 2), interpolation=cv2.INTER_CUBIC)
        # 三种上采样回 ps
        bicubic = cv2.resize(lr, (ps, ps), interpolation=cv2.INTER_CUBIC)
        sr = sr_infer(model, lr)
        if sr.shape[:2] != (ps, ps):
            sr = cv2.resize(sr, (ps, ps))

        # 拼成一行: HR | bicubic | SR (各 target_h 高)
        def lab(im, txt):
            im = cv2.resize(im, (target_h, target_h))
            cv2.putText(im, txt, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255), 1, cv2.LINE_AA)
            return im
        row = np.hstack([
            lab(hr_crop, "HR"),
            lab(bicubic, "bicubic"),
            lab(sr, "ESPCN"),
        ])
        rows.append(row)
    return np.vstack(rows)


def main():
    Path(OUT).mkdir(parents=True, exist_ok=True)
    model = load_model()

    # ── 测试 1: 合成 (HR → LR → 上采样) ──
    hr_files = []
    for cls in ["blue", "purple", "red"]:
        d = Path(HR_DIR) / cls
        if d.exists():
            for f in d.iterdir():
                hr_files.append(str(f))
    random.shuffle(hr_files)
    samples = []
    for f in hr_files[:6]:
        im = cv2.imread(f)
        if im is None:
            continue
        if min(im.shape[:2]) >= 256:
            samples.append(im)
        if len(samples) >= 4:
            break

    grid = make_compare_grid(model, samples)
    out_path = os.path.join(OUT, "synthetic_compare.png")
    cv2.imwrite(out_path, grid)
    print(f"合成对比图: {out_path}")

    # ── 测试 2: 真实小目标 (laser_rx <96px) ──
    # 从 merged 数据集找那些被过滤掉的 "太小" 的样本
    real_small = []
    for split in ["train", "valid", "test"]:
        img_dir = Path(MERGED) / split / "images"
        lbl_dir = Path(MERGED) / split / "labels"
        if not img_dir.exists():
            continue
        for img_file in sorted(img_dir.iterdir())[:300]:
            if img_file.suffix.lower() not in (".jpg", ".png"):
                continue
            lbl_file = lbl_dir / (img_file.stem + ".txt")
            if not lbl_file.exists():
                continue
            img = cv2.imread(str(img_file))
            if img is None:
                continue
            H, W = img.shape[:2]
            for line in lbl_file.read_text().strip().split("\n"):
                parts = line.split()
                if len(parts) < 5:
                    continue
                coords = [float(x) for x in parts[1:]]
                xs = coords[0::2]
                ys = coords[1::2]
                bw = (max(xs) - min(xs)) * W
                bh = (max(ys) - min(ys)) * H
                if 32 <= min(bw, bh) <= 96:  # 真实小目标
                    cx_pix = int((min(xs) + max(xs)) / 2 * W)
                    cy_pix = int((min(ys) + max(ys)) / 2 * H)
                    half = max(int(max(bw, bh) * 0.7), 32)
                    x1 = max(0, cx_pix - half)
                    y1 = max(0, cy_pix - half)
                    x2 = min(W, cx_pix + half)
                    y2 = min(H, cy_pix + half)
                    crop = img[y1:y2, x1:x2]
                    if crop.size > 0 and min(crop.shape[:2]) >= 32:
                        real_small.append(crop)
            if len(real_small) >= 4:
                break
        if len(real_small) >= 4:
            break

    if real_small:
        rows = []
        target_h = 200
        for crop in real_small[:4]:
            sr = sr_infer(model, crop)
            cubic = cv2.resize(crop, (sr.shape[1], sr.shape[0]),
                               interpolation=cv2.INTER_CUBIC)

            def lab(im, txt):
                im2 = cv2.resize(im, (target_h, target_h))
                cv2.putText(im2, txt, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 255), 1, cv2.LINE_AA)
                return im2
            row = np.hstack([
                lab(crop, f"orig {crop.shape[1]}x{crop.shape[0]}"),
                lab(cubic, "bicubic x2"),
                lab(sr, "ESPCN x2"),
            ])
            rows.append(row)
        grid2 = np.vstack(rows)
        out2 = os.path.join(OUT, "real_small_compare.png")
        cv2.imwrite(out2, grid2)
        print(f"真实小目标对比: {out2}")
    else:
        print("未找到真实小目标样本 (32-96px)")


if __name__ == "__main__":
    main()
