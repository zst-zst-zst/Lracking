"""SR Phase 3a: 导出 ESPCN PyTorch → ONNX → TensorRT engine.

ONNX 选用动态尺寸 (HW 变长), 因为 ROI 尺寸不固定.
TRT engine 用 optimization profile 覆盖典型 ROI 尺寸 (64-256).
"""
import argparse, os, subprocess, sys
from pathlib import Path

import torch
sys.path.insert(0, os.path.dirname(__file__))
from sr_model import ESPCN


def export_onnx(ckpt_path: Path, out_path: Path, scale: int = 2):
    state = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    model = ESPCN(scale=scale)
    model.load_state_dict(state["model"])
    model.eval()

    # 用动态尺寸: batch=1, C=3, H/W 任意 (TRT 用 profile 限制范围)
    dummy = torch.randn(1, 3, 96, 96)
    torch.onnx.export(
        model,
        dummy,
        str(out_path),
        input_names=["lr"],
        output_names=["sr"],
        opset_version=17,
        do_constant_folding=True,
        dynamic_axes={
            "lr": {0: "N", 2: "H", 3: "W"},
            "sr": {0: "N", 2: "H2", 3: "W2"},
        },
    )
    print(f"ONNX 导出: {out_path}")


def build_trt(onnx_path: Path, engine_path: Path,
              min_hw=(48, 48), opt_hw=(128, 128), max_hw=(320, 320)):
    """用 trtexec 构建 TRT engine, 含 optimization profile."""
    trtexec = "/usr/src/tensorrt/bin/trtexec"
    if not os.path.exists(trtexec):
        print(f"trtexec 未找到: {trtexec}")
        return False

    min_shape = f"lr:1x3x{min_hw[0]}x{min_hw[1]}"
    opt_shape = f"lr:1x3x{opt_hw[0]}x{opt_hw[1]}"
    max_shape = f"lr:1x3x{max_hw[0]}x{max_hw[1]}"

    cmd = [
        trtexec,
        f"--onnx={onnx_path}",
        f"--saveEngine={engine_path}",
        "--fp16",
        f"--minShapes={min_shape}",
        f"--optShapes={opt_shape}",
        f"--maxShapes={max_shape}",
        "--memPoolSize=workspace:2048",
    ]
    print("运行:", " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("trtexec 失败:", result.stderr[-3000:])
        return False
    print(f"TRT engine: {engine_path}")
    # 提取 latency
    for line in result.stdout.split("\n"):
        if "GPU Compute Time" in line and "mean" in line:
            print("性能:", line.strip())
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="/home/zst/Tracking/src/detect/model/sr/espcn_x2_best.pt")
    ap.add_argument("--out_dir", default="/home/zst/Tracking/src/detect/model/export")
    ap.add_argument("--scale", type=int, default=2)
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    onnx_path = out_dir / "espcn_x2.onnx"
    engine_path = out_dir / "espcn_x2_fp16.engine"

    export_onnx(Path(args.ckpt), onnx_path, args.scale)
    build_trt(onnx_path, engine_path)


if __name__ == "__main__":
    main()
