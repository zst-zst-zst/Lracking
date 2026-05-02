# thirdparty

这个目录目前主要用于放可选的 ONNX Runtime 预构建包。

## 当前内容

```text
src/thirdparty/
├── onnxruntime -> onnxruntime-linux-x64-gpu-1.23.2
├── onnxruntime-linux-x64-gpu-1.23.2/
└── onnxruntime-linux-x64-gpu-1.23.2.tgz
```

## 现在的作用

- 默认检测链路走 TensorRT / `TRTInferX`
- `src/detect/CMakeLists.txt` 里的 `DETECT_WITH_ONNX` 默认是 `OFF`
- 只有在你显式开启 ONNX 后端或做开发机调试时，这个目录才有意义

## 默认情况下不需要动它

如果你只是按仓库当前主链路构建：

```bash
cmake -S src -B src/build
cmake --build src/build -j"$(nproc)"
```

那么主要依赖是 CUDA + TensorRT，而不是这里的 ONNX Runtime。

## 需要 ONNX 时

可以在自定义构建里开启：

```bash
cmake -S src/detect -B build_detect_onnx -DDETECT_WITH_ONNX=ON
```

实际能否直接链接成功，还取决于系统里 `onnxruntime` 的查找方式和链接路径设置。

## 维护建议

- `onnxruntime` 软链接保持指向当前启用版本
- 大体积压缩包和展开目录已经在 `.gitignore` 里处理
- 如果默认 TensorRT 链路能满足需求，不建议再额外扩散 ONNX 路径
