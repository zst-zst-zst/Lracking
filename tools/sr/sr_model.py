"""ESPCN ×2 super-resolution model (lightweight, ~25K params).

Reference: Shi et al. "Real-Time Single Image and Video Super-Resolution Using
an Efficient Sub-Pixel Convolutional Neural Network" (CVPR 2016).

Modifications for laser_rx detection:
  - 3-channel input (RGB instead of Y from YCbCr)
  - Slightly wider channels for color fidelity
  - PReLU activation (better edges than ReLU)
"""
import torch
import torch.nn as nn


class ESPCN(nn.Module):
    """ESPCN with sub-pixel convolution upsampling."""

    def __init__(self, scale: int = 2, num_channels: int = 3, n_feats: int = 64):
        super().__init__()
        self.scale = scale

        # 特征提取
        self.feat = nn.Sequential(
            nn.Conv2d(num_channels, n_feats, kernel_size=5, padding=2),
            nn.PReLU(n_feats),
            nn.Conv2d(n_feats, n_feats, kernel_size=3, padding=1),
            nn.PReLU(n_feats),
            nn.Conv2d(n_feats, n_feats // 2, kernel_size=3, padding=1),
            nn.PReLU(n_feats // 2),
        )
        # 子像素卷积: 输出 (scale^2 * C) 通道, 然后 PixelShuffle
        self.upscale = nn.Sequential(
            nn.Conv2d(n_feats // 2, num_channels * scale * scale,
                      kernel_size=3, padding=1),
            nn.PixelShuffle(scale),
        )

        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        feat = self.feat(x)
        out = self.upscale(feat)
        return torch.clamp(out, 0.0, 1.0)


if __name__ == "__main__":
    # 参数量统计
    m = ESPCN(scale=2)
    n = sum(p.numel() for p in m.parameters())
    print(f"ESPCN x2 参数量: {n:,} ({n/1024:.1f}K)")
    x = torch.randn(1, 3, 64, 64)
    y = m(x)
    print(f"输入 {tuple(x.shape)} → 输出 {tuple(y.shape)}")
