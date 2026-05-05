#!/usr/bin/env python3
"""SR Phase 1.5: 从屏幕录屏自动抽取 laser_rx HR patches.

策略:
  1. 视频采样: 每 N 帧抽一帧
  2. 找相机窗口: 简单大暗区检测 (录屏中相机画面是大块连续暗区)
  3. 在相机区域跑 layer2 detector (yolo11n) 找 laser_rx bbox
  4. 内缩 4-6 像素裁剪, 避开 UI 绘制的边框线
  5. 质量过滤: 大小 >= 64px, 模糊 (Laplacian var > threshold), 曝光合理
  6. 保存到 sr_dataset/hr_video/<class>/<id>.png

输出会与 sr_dataset/hr/ 合并 (train 时两个目录都用).
"""
import argparse, os, sys, cv2, hashlib
from pathlib import Path
import numpy as np

import onnxruntime as ort  # 用 ONNX 不锁 GPU, 避免和训练冲突


CLS_NAMES = ["blue", "purple", "red"]


def find_camera_region(frame: np.ndarray) -> tuple:
    """在屏幕录屏帧中找相机窗口区域.

    相机画面是大块连续暗区, 周围是浅色桌面/UI. 用阈值二值化 + 最大连通域.
    返回 (x, y, w, h) 或 None.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    H, W = gray.shape
    # 暗区掩码 (像素 < 80 视为暗)
    _, mask = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
    # 形态学闭合连接
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    # 最大轮廓
    c = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    # 面积过小 (< 1/4 屏幕) 视为无效
    if w * h < W * H * 0.15:
        return None
    # 长宽比合理 (摄像头画面 4:3 或 16:9 居多)
    ar = w / h
    if ar < 0.7 or ar > 2.5:
        return None
    return (x, y, w, h)


# ── YOLOv11 ONNX 推理简版 ──
class YoloONNX:
    def __init__(self, onnx_path: str, input_size: int = 640,
                 conf_thres: float = 0.4, iou_thres: float = 0.45):
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self.session = ort.InferenceSession(onnx_path, providers=providers)
        self.input_size = input_size
        self.conf = conf_thres
        self.iou = iou_thres
        self.input_name = self.session.get_inputs()[0].name

    def _letterbox(self, img):
        h, w = img.shape[:2]
        s = min(self.input_size / h, self.input_size / w)
        nh, nw = int(round(h * s)), int(round(w * s))
        rs = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        out = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        px = (self.input_size - nw) // 2
        py = (self.input_size - nh) // 2
        out[py:py + nh, px:px + nw] = rs
        return out, s, px, py

    def __call__(self, bgr: np.ndarray):
        lb, s, px, py = self._letterbox(bgr)
        x = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        x = x.transpose(2, 0, 1)[None]
        out = self.session.run(None, {self.input_name: x})[0]
        # YOLOv11 输出 (1, 4+num_classes, N) 或 (1, N, 4+num_classes)
        out = out[0]
        if out.shape[0] < out.shape[1]:
            out = out.T  # → (N, 4+nc)
        boxes_xywh = out[:, :4]
        cls_scores = out[:, 4:]
        conf = cls_scores.max(axis=1)
        cls_id = cls_scores.argmax(axis=1)
        keep = conf > self.conf
        if not keep.any():
            return []
        boxes_xywh = boxes_xywh[keep]
        conf = conf[keep]
        cls_id = cls_id[keep]
        # xywh → xyxy in letterbox space → 原图
        x1 = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2
        y1 = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2
        x2 = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2
        y2 = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2
        x1 = (x1 - px) / s
        y1 = (y1 - py) / s
        x2 = (x2 - px) / s
        y2 = (y2 - py) / s
        boxes = np.stack([x1, y1, x2, y2], axis=1)
        # NMS
        keep_idx = cv2.dnn.NMSBoxes(
            [[float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])] for b in boxes],
            conf.tolist(), self.conf, self.iou)
        if len(keep_idx) == 0:
            return []
        keep_idx = np.array(keep_idx).flatten()
        results = []
        for i in keep_idx:
            results.append({
                "x1": float(boxes[i, 0]), "y1": float(boxes[i, 1]),
                "x2": float(boxes[i, 2]), "y2": float(boxes[i, 3]),
                "conf": float(conf[i]), "cls": int(cls_id[i]),
            })
        return results


def quality_check(crop: np.ndarray, blur_thr: float, min_size: int) -> bool:
    h, w = crop.shape[:2]
    if min(h, w) < min_size:
        return False
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    if gray.mean() < 15 or gray.mean() > 240:
        return False
    if cv2.Laplacian(gray, cv2.CV_64F).var() < blur_thr:
        return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", nargs="+", required=True, help="视频路径列表")
    ap.add_argument("--onnx", default="/home/zst/Tracking/src/detect/model/export/layer2_laser_rx.onnx")
    ap.add_argument("--out", default="/home/zst/Tracking/src/detect/model/layer2/sr_dataset/hr_video")
    ap.add_argument("--sample_every", type=int, default=30, help="每 N 帧抽一帧")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--min_size", type=int, default=64,
                    help="laser_rx bbox 最短边阈值 (放宽因屏幕录屏分辨率有限)")
    ap.add_argument("--blur_thr", type=float, default=40.0,
                    help="模糊阈值 (放宽因屏幕录屏本身有压缩)")
    ap.add_argument("--shrink", type=int, default=4, help="bbox 内缩像素数, 避开 UI 边框线")
    ap.add_argument("--max_per_video", type=int, default=2000)
    args = ap.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    for c in CLS_NAMES:
        (out_dir / c).mkdir(exist_ok=True)

    yolo = YoloONNX(args.onnx, conf_thres=args.conf)

    total_saved = 0
    for video_path in args.videos:
        if not os.path.exists(video_path):
            print(f"跳过不存在: {video_path}")
            continue
        print(f"\n=== {Path(video_path).name} ===")
        cap = cv2.VideoCapture(video_path)
        n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"帧数: {n_frames}, fps: {fps:.1f}")

        saved_this = 0
        frame_idx = 0
        sample_count = 0
        # 复用 region (假设单视频内窗口位置稳定)
        cached_region = None

        while saved_this < args.max_per_video:
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            ok, frame = cap.read()
            if not ok:
                break
            sample_count += 1

            # 每 30 个采样点重新找一次窗口位置 (有时用户拖动窗口)
            if cached_region is None or sample_count % 30 == 0:
                region = find_camera_region(frame)
                if region:
                    cached_region = region

            if cached_region is None:
                frame_idx += args.sample_every
                continue

            x, y, w, h = cached_region
            cam_frame = frame[y:y + h, x:x + w]

            # 检测
            dets = yolo(cam_frame)
            for det in dets:
                bx1 = max(0, int(det["x1"]) + args.shrink)
                by1 = max(0, int(det["y1"]) + args.shrink)
                bx2 = min(cam_frame.shape[1], int(det["x2"]) - args.shrink)
                by2 = min(cam_frame.shape[0], int(det["y2"]) - args.shrink)
                if bx2 - bx1 < args.min_size or by2 - by1 < args.min_size:
                    continue
                crop = cam_frame[by1:by2, bx1:bx2]
                if not quality_check(crop, args.blur_thr, args.min_size):
                    continue

                cls_name = CLS_NAMES[det["cls"]] if det["cls"] < len(CLS_NAMES) else "unknown"
                # 用 frame 内容 hash 防重复
                h_hash = hashlib.md5(crop.tobytes()).hexdigest()[:10]
                stem = Path(video_path).stem.replace(" ", "_")[:20]
                out_name = f"{stem}_{frame_idx:06d}_{h_hash}.png"
                out_path = out_dir / cls_name / out_name
                if not out_path.exists():
                    cv2.imwrite(str(out_path), crop, [cv2.IMWRITE_PNG_COMPRESSION, 3])
                    saved_this += 1
                    total_saved += 1

            frame_idx += args.sample_every

        cap.release()
        print(f"本视频保存: {saved_this}")

    # 统计
    print(f"\n========== 总计保存: {total_saved} ==========")
    for c in CLS_NAMES:
        n = len(list((out_dir / c).iterdir()))
        print(f"  {c}: {n}")


if __name__ == "__main__":
    main()
