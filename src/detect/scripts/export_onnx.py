#!/usr/bin/env python3
"""Export trained Layer 1 and Layer 2 models to ONNX format.

Optionally convert to TensorRT engine using trtexec.
"""

import argparse
import os
import subprocess
import sys

# detect/ directory (parent of scripts/)
DETECT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def export_model(model_path, imgsz, output_dir, name, half=True):
    """Export a YOLO model to ONNX."""
    from ultralytics import YOLO

    model = YOLO(model_path)
    onnx_path = model.export(
        format="onnx",
        imgsz=imgsz,
        half=half,
        simplify=True,
        opset=17,
        dynamic=False,
    )
    print(f"Exported ONNX: {onnx_path}")

    # Copy to output dir with desired name
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        dst = os.path.join(output_dir, f"{name}.onnx")
        if os.path.abspath(onnx_path) != os.path.abspath(dst):
            import shutil
            shutil.copy2(onnx_path, dst)
            print(f"Copied to: {dst}")
        return dst
    return onnx_path


def build_trt_engine(onnx_path, engine_path, fp16=True):
    """Build TensorRT engine from ONNX using trtexec."""
    import shutil
    trtexec = shutil.which("trtexec")
    if not trtexec:
        for p in ["/usr/src/tensorrt/bin/trtexec", "/usr/local/bin/trtexec"]:
            if os.path.isfile(p) and os.access(p, os.X_OK):
                trtexec = p
                break
    if not trtexec:
        print("WARNING: trtexec not found in PATH. Skipping TRT engine build.")
        print("  Install TensorRT or add trtexec to PATH, then re-run with --trt")
        return False

    # Set CUDA device
    env = os.environ.copy()
    env["CUDA_VISIBLE_DEVICES"] = "0"

    cmd = [trtexec, f"--onnx={onnx_path}", f"--saveEngine={engine_path}"]
    if fp16:
        cmd.append("--fp16")
    # Use memPoolSize instead of workspace for newer TensorRT versions
    cmd.extend(["--memPoolSize=workspace:4096"])

    print(f"Building TRT engine: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if result.returncode != 0:
        print(f"trtexec failed:\n{result.stderr[-2000:]}")
        return False
    print(f"TRT engine saved: {engine_path}")
    return True


def main():
    parser = argparse.ArgumentParser(description="Export models to ONNX/TRT")
    parser.add_argument("--layer1", default=os.path.join(DETECT_DIR, "model/layer1/train/weights/best.pt"),
                        help="Layer 1 model path")
    parser.add_argument("--layer2", default=os.path.join(DETECT_DIR, "model/layer2/train/weights/best.pt"),
                        help="Layer 2 model path")
    parser.add_argument("--layer1-imgsz", type=int, default=640)
    parser.add_argument("--layer2-imgsz", type=int, default=192)
    parser.add_argument("--output", default=os.path.join(DETECT_DIR, "model/export"),
                        help="Output directory for exported models")
    parser.add_argument("--trt", action="store_true",
                        help="Also build TensorRT engines")
    parser.add_argument("--fp16", action="store_true", default=True,
                        help="Use FP16 for TRT engines")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not installed")
        sys.exit(1)

    output = os.path.abspath(args.output)
    trt_jobs = []  # (onnx_path, engine_path) pairs for TRT build

    # --- Phase 1: ONNX exports (uses PyTorch / ultralytics) ---
    onnx1 = None
    if os.path.isfile(args.layer1):
        print("\n=== Exporting Layer 1 (plane) ===")
        onnx1 = export_model(args.layer1, args.layer1_imgsz, output, "layer1_plane")
    else:
        print(f"Layer 1 model not found: {args.layer1}")

    onnx2 = None
    if os.path.isfile(args.layer2):
        print("\n=== Exporting Layer 2 (laser_rx) ===")
        onnx2 = export_model(args.layer2, args.layer2_imgsz, output, "layer2_laser_rx")
    else:
        print(f"Layer 2 model not found: {args.layer2}")

    # --- Phase 2: TRT engine builds (separate from PyTorch to avoid CUDA context conflict) ---
    if args.trt:
        if onnx1:
            trt_jobs.append((onnx1, os.path.join(output, "layer1_plane_fp16.engine")))
        if onnx2:
            trt_jobs.append((onnx2, os.path.join(output, "layer2_laser_rx_fp16.engine")))

        if trt_jobs:
            import gc, torch
            torch.cuda.empty_cache()
            del torch
            gc.collect()

            print("\n=== Building TensorRT engines ===")
            for onnx_path, engine_path in trt_jobs:
                build_trt_engine(onnx_path, engine_path, fp16=args.fp16)

    print("\nDone. Exported models in:", output)


if __name__ == "__main__":
    main()
