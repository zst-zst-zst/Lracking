"""SR 自监督数据集.

策略:
  1. 从 HR 大图随机裁出 patch_size×patch_size 的 patch 作为 HR target
  2. bicubic 下采样到 patch_size/scale 作为 LR 输入
  3. 加退化使 LR 更像真实小目标:
       - 随机高斯模糊 (sigma 0~1.5)
       - 随机高斯噪声 (sigma 0~5/255)
       - JPEG 压缩 (quality 70~95) 模拟相机/编码损失
  4. 随机翻转/90度旋转/颜色微抖动
"""
import os, random
from pathlib import Path
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


def _bgr_to_tensor(im: np.ndarray) -> torch.Tensor:
    """HWC BGR uint8 → CHW RGB float [0,1]."""
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    im = im.astype(np.float32) / 255.0
    return torch.from_numpy(im.transpose(2, 0, 1))


def _random_degrade(lr: np.ndarray, rng: random.Random) -> np.ndarray:
    """对 LR 加退化, 模拟真实小目标的成像质量."""
    out = lr.copy()

    # 高斯模糊
    if rng.random() < 0.7:
        sigma = rng.uniform(0.2, 1.5)
        ksize = max(3, int(2 * round(2 * sigma) + 1))
        out = cv2.GaussianBlur(out, (ksize, ksize), sigma)

    # 高斯噪声
    if rng.random() < 0.5:
        nsigma = rng.uniform(0, 5.0)
        noise = np.random.randn(*out.shape) * nsigma
        out = np.clip(out.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    # JPEG 压缩
    if rng.random() < 0.4:
        q = rng.randint(70, 95)
        ok, enc = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, q])
        if ok:
            out = cv2.imdecode(enc, cv2.IMREAD_COLOR)

    return out


class SRDataset(Dataset):
    def __init__(self, root: str, patch_size: int = 128, scale: int = 2,
                 split: str = "train", val_ratio: float = 0.05):
        self.patch_size = patch_size
        self.scale = scale
        self.lr_size = patch_size // scale

        # 收集所有 HR 图片
        all_files = []
        for cls_dir in sorted(Path(root).iterdir()):
            if cls_dir.is_dir():
                for f in cls_dir.iterdir():
                    if f.suffix.lower() in (".png", ".jpg", ".jpeg"):
                        all_files.append(str(f))

        # 固定种子划分 train/val
        rng_split = random.Random(42)
        rng_split.shuffle(all_files)
        n_val = max(1, int(len(all_files) * val_ratio))
        if split == "train":
            self.files = all_files[n_val:]
        else:  # val
            self.files = all_files[:n_val]

        # 训练用退化 (val 只用 bicubic 下采样, 不加退化, 便于稳定评估 PSNR)
        self.split = split
        print(f"SRDataset({split}): {len(self.files)} files")

    def __len__(self):
        return len(self.files)

    def _crop_random(self, im: np.ndarray) -> np.ndarray:
        h, w = im.shape[:2]
        ps = self.patch_size
        if h < ps or w < ps:
            # 太小 → resize 到至少 ps
            scale = ps / min(h, w) * 1.05
            im = cv2.resize(im, (int(w * scale), int(h * scale)),
                            interpolation=cv2.INTER_CUBIC)
            h, w = im.shape[:2]
        x = random.randint(0, w - ps)
        y = random.randint(0, h - ps)
        return im[y:y + ps, x:x + ps]

    def _augment(self, hr: np.ndarray) -> np.ndarray:
        # 翻转
        if random.random() < 0.5:
            hr = cv2.flip(hr, 1)
        if random.random() < 0.5:
            hr = cv2.flip(hr, 0)
        # 90 度旋转
        k = random.randint(0, 3)
        if k > 0:
            hr = np.rot90(hr, k).copy()
        # 颜色微抖动 (HSV)
        if random.random() < 0.3:
            hsv = cv2.cvtColor(hr, cv2.COLOR_BGR2HSV).astype(np.int32)
            hsv[..., 0] = (hsv[..., 0] + random.randint(-5, 5)) % 180
            hsv[..., 1] = np.clip(hsv[..., 1] + random.randint(-15, 15), 0, 255)
            hsv[..., 2] = np.clip(hsv[..., 2] + random.randint(-15, 15), 0, 255)
            hr = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
        return hr

    def __getitem__(self, idx):
        path = self.files[idx]
        hr = cv2.imread(path)
        if hr is None:
            # 损坏图片 → 重采样
            return self.__getitem__((idx + 1) % len(self.files))

        hr = self._crop_random(hr)
        if self.split == "train":
            hr = self._augment(hr)

        # 下采样到 LR
        lr = cv2.resize(hr, (self.lr_size, self.lr_size),
                        interpolation=cv2.INTER_CUBIC)
        if self.split == "train":
            lr = _random_degrade(lr, random)

        return _bgr_to_tensor(lr), _bgr_to_tensor(hr)


if __name__ == "__main__":
    ds = SRDataset("/home/zst/Tracking/src/detect/model/layer2/sr_dataset/hr",
                   patch_size=128, scale=2, split="train")
    lr, hr = ds[0]
    print(f"LR shape: {tuple(lr.shape)}, HR shape: {tuple(hr.shape)}")
    print(f"LR range: [{lr.min():.3f}, {lr.max():.3f}], HR range: [{hr.min():.3f}, {hr.max():.3f}]")
