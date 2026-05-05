#!/usr/bin/env bash
# 一键导出 + 替换 layer2 引擎 (带备份)
# 用法: bash tools/deploy_retrained_layer2.sh
set -euo pipefail

ROOT="/home/zst/Tracking"
NEW_PT="$ROOT/src/detect/model/layer2/train_yolo11n_with_negatives/weights/best.pt"
EXPORT_DIR="$ROOT/src/detect/model/export"
OLD_ONNX="$EXPORT_DIR/layer2_laser_rx.onnx"
OLD_ENGINE="$EXPORT_DIR/layer2_laser_rx_fp16.engine"
PROD_WEIGHTS="$ROOT/src/detect/model/layer2/weights/best.pt"

TS=$(date +%Y%m%d_%H%M%S)

if [[ ! -f "$NEW_PT" ]]; then
    echo "ERROR: 新权重不存在: $NEW_PT"
    echo "训练还在跑? tail -f \$(ls -t $ROOT/log/finetune_*.log | head -1)"
    exit 1
fi

echo "=== 备份旧模型 ==="
for f in "$OLD_ONNX" "$OLD_ENGINE" "$PROD_WEIGHTS"; do
    if [[ -f "$f" ]]; then
        cp -v "$f" "${f}.bak_${TS}"
    fi
done

echo ""
echo "=== 导出新 ONNX + TRT 引擎 (FP16, imgsz=640) ==="
"$ROOT/.venv/bin/python3" "$ROOT/src/detect/scripts/export_onnx.py" \
    --layer2 "$NEW_PT" \
    --layer2-imgsz 640 \
    --output "$EXPORT_DIR" \
    --trt --fp16

echo ""
echo "=== 同步 production 权重 ==="
cp -v "$NEW_PT" "$PROD_WEIGHTS"

echo ""
echo "=== 完成. 验证: ==="
ls -la "$EXPORT_DIR/layer2_laser_rx.onnx" "$EXPORT_DIR/layer2_laser_rx_fp16.engine"
echo ""
echo "运行 tracking 会自动加载新引擎 (按 config/cascade.yaml 的路径)"
echo "回滚命令:"
echo "  cp \"${OLD_ENGINE}.bak_${TS}\" \"$OLD_ENGINE\""
