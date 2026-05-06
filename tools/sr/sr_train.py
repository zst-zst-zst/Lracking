"""SR Phase 2: ESPCN ×2 训练.

Loss = L1(SR, HR) + λ * L1(Sobel(SR), Sobel(HR))   (强化边缘)

输出:
  src/detect/model/sr/espcn_x2.pt          (PyTorch checkpoint)
  src/detect/model/sr/training_log.csv     (epoch loss + PSNR)
"""
import argparse, os, sys, csv, math, time
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader

sys.path.insert(0, os.path.dirname(__file__))
from sr_model import ESPCN
from sr_dataset import SRDataset


# ── Sobel 边缘 loss ──────────────────────────────────────────
def sobel_grad(x: torch.Tensor) -> torch.Tensor:
    """计算 Sobel x/y 梯度 (concat). x: B×C×H×W."""
    kx = torch.tensor([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]],
                      dtype=x.dtype, device=x.device).view(1, 1, 3, 3)
    ky = kx.transpose(2, 3)
    C = x.size(1)
    kx = kx.expand(C, 1, 3, 3)
    ky = ky.expand(C, 1, 3, 3)
    gx = F.conv2d(x, kx, padding=1, groups=C)
    gy = F.conv2d(x, ky, padding=1, groups=C)
    return torch.cat([gx, gy], dim=1)


def edge_loss(sr: torch.Tensor, hr: torch.Tensor) -> torch.Tensor:
    return F.l1_loss(sobel_grad(sr), sobel_grad(hr))


def psnr(sr: torch.Tensor, hr: torch.Tensor) -> float:
    mse = F.mse_loss(sr, hr).item()
    if mse < 1e-10:
        return 100.0
    return 10.0 * math.log10(1.0 / mse)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="/home/zst/Tracking/src/detect/model/layer2/sr_dataset/hr")
    ap.add_argument("--out", default="/home/zst/Tracking/src/detect/model/sr")
    ap.add_argument("--scale", type=int, default=2)
    ap.add_argument("--patch", type=int, default=128)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--edge_lambda", type=float, default=0.5)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--device", default="cuda")
    args = ap.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    train_ds = SRDataset(args.data, patch_size=args.patch, scale=args.scale, split="train")
    val_ds = SRDataset(args.data, patch_size=args.patch, scale=args.scale, split="val")
    train_loader = DataLoader(train_ds, batch_size=args.batch, shuffle=True,
                              num_workers=args.workers, pin_memory=True, drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch, shuffle=False,
                            num_workers=args.workers, pin_memory=True)

    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    model = ESPCN(scale=args.scale).to(device)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"Device: {device}, ESPCN x{args.scale} params: {n_params:,}")

    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-5)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs, eta_min=args.lr * 0.01)

    log_path = out_dir / "training_log.csv"
    with open(log_path, "w", newline="") as f:
        csv.writer(f).writerow(["epoch", "train_loss", "val_l1", "val_psnr", "lr"])

    best_psnr = 0.0
    for epoch in range(1, args.epochs + 1):
        model.train()
        t0 = time.time()
        train_loss_sum = 0.0
        train_steps = 0
        for lr, hr in train_loader:
            lr = lr.to(device, non_blocking=True)
            hr = hr.to(device, non_blocking=True)

            sr = model(lr)
            loss_l1 = F.l1_loss(sr, hr)
            loss_edge = edge_loss(sr, hr)
            loss = loss_l1 + args.edge_lambda * loss_edge

            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            opt.step()

            train_loss_sum += loss.item()
            train_steps += 1
        sched.step()
        train_loss = train_loss_sum / max(1, train_steps)

        # 评估
        model.eval()
        val_l1_sum = 0.0
        val_psnr_sum = 0.0
        val_steps = 0
        with torch.no_grad():
            for lr, hr in val_loader:
                lr = lr.to(device, non_blocking=True)
                hr = hr.to(device, non_blocking=True)
                sr = model(lr)
                val_l1_sum += F.l1_loss(sr, hr).item()
                val_psnr_sum += psnr(sr, hr)
                val_steps += 1
        val_l1 = val_l1_sum / max(1, val_steps)
        val_psnr = val_psnr_sum / max(1, val_steps)

        cur_lr = opt.param_groups[0]["lr"]
        elapsed = time.time() - t0
        print(f"Epoch {epoch:3d}/{args.epochs} | train_loss={train_loss:.4f} | "
              f"val_l1={val_l1:.4f} | val_psnr={val_psnr:.2f} | lr={cur_lr:.2e} | {elapsed:.1f}s")

        with open(log_path, "a", newline="") as f:
            csv.writer(f).writerow([epoch, train_loss, val_l1, val_psnr, cur_lr])

        # 保存最佳模型
        if val_psnr > best_psnr:
            best_psnr = val_psnr
            torch.save({
                "model": model.state_dict(),
                "scale": args.scale,
                "epoch": epoch,
                "val_psnr": val_psnr,
            }, out_dir / "espcn_x2_best.pt")

        # 定期保存
        if epoch % 20 == 0 or epoch == args.epochs:
            torch.save({
                "model": model.state_dict(),
                "scale": args.scale,
                "epoch": epoch,
            }, out_dir / "espcn_x2_last.pt")

    print(f"\n训练完成. 最佳 PSNR: {best_psnr:.2f} dB")
    print(f"权重: {out_dir / 'espcn_x2_best.pt'}")


if __name__ == "__main__":
    main()
