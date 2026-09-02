#pragma once

// ═══════════════════════════════════════════════════════════════
//  Atmosphere Phase 1 — 构成云朵的软体质点 Blob
// ═══════════════════════════════════════════════════════════════

struct CloudBlob {
    float baseX;              // 相对云中心的基准 X 偏移
    float baseY;              // 相对云中心的基准 Y 偏移
    
    float currentX;           // 当前世界坐标 X
    float currentY;           // 当前世界坐标 Y
    
    float targetX;            // 目标世界坐标 X (含整体重力偏移与局部形变)
    float targetY;            // 目标世界坐标 Y
    
    float vx;                 // 速度 X
    float vy;                 // 速度 Y
    
    float baseRadius;         // 基准半径 (像素)
    float currentRadius;      // 动态半径 (经独立相位呼吸计算后)
    float phase;              // 独立呼吸相位 (弧度)
    
    float gravityFactorX;     // 重力局部响应形变因子 X (各 blob 差异化响应)
    float gravityFactorY;     // 重力局部响应形变因子 Y
};
