#pragma once

// 简单噪声模块，用于生成体积云的基础密度字段
// 为了在 ESP32‑S3 上保持轻量，这里使用纯数学函数生成伪噪声，避免外部库依赖。
// 可通过调整系数获得不同尺度的云结构。

float getDensity(int x, int y, int z, float time);
