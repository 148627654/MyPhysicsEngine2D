// SIMDIntegrator.cpp — AVX2 8 路并发前向欧拉积分内核（V5 Day 3）
//
// 核心思路：SoA 拆分单精度流让一条 256-bit YMM 寄存器严丝合缝装下
// 8 个刚体的同一属性。FMA 融合乘加一条指令完成 乘+加，速度翻倍且
// 中间无截断舍入。静态/睡眠刚体用掩码 blend 免分支处理（分支预测
// 惩罚为零）。尾部不足 8 个的刚体走标量余数循环，任意数量 100% 正确。
//
// 标量回退路径（IntegrateScalar）与旧版积分逐位一致：CPU 不支持
// AVX2 或用户关闭 SIMD 时自动降级，也可供验收测试做对照基准。
#include "SoABody.h"
#include "SIMDMath.h"
#include "Body.h"

void BodySoA::IntegrateScalar(float dt, const Vector2& gravity)
{
    const int count = GetCount();
    const float gx = gravity.getX(), gy = gravity.getY();

    float* posX = positionsX.data();
    float* posY = positionsY.data();
    float* velX = velocitiesX.data();
    float* velY = velocitiesY.data();
    float* rot = rotations.data();
    float* angVel = angularVels.data();
    const float* invM = invMasses.data();
    const float* invI = invInertias.data();
    float* frcX = forcesX.data();
    float* frcY = forcesY.data();
    float* trq = torques.data();
    const float* gs = gravityScales.data();
    const uint32_t* flg = flags.data();
    Body** owners = bodyOwners.data();

    for (int i = 0; i < count; ++i) {
        if (invM[i] == 0.0f || !(flg[i] & FLAG_AWAKE)) continue; // 静态/睡眠跳过
        velX[i] += (frcX[i] * invM[i] + gx * gs[i]) * dt;
        velY[i] += (frcY[i] * invM[i] + gy * gs[i]) * dt;
        angVel[i] += trq[i] * invI[i] * dt;

        Body* b = owners[i];
        b->m_prevPosition.x = posX[i]; // 记录当前位置为 P0
        b->m_prevPosition.y = posY[i];
        b->m_prevRotation = rot[i];

        posX[i] += velX[i] * dt; // 预测 P1
        posY[i] += velY[i] * dt;
        rot[i] += angVel[i] * dt;

        frcX[i] = 0.0f;
        frcY[i] = 0.0f;
        trq[i] = 0.0f;
    }
}

void BodySoA::IntegrateSIMD(float dt, const Vector2& gravity)
{
#if SIMD_AVX2_ENABLED
    // 运行时 CPU 能力检测 + 用户开关：不支持则无缝降级标量路径
    if (!m_useSIMD || !Physics2D::SIMD::IsAVX2Supported()) {
        IntegrateScalar(dt, gravity);
        return;
    }

    const int count = GetCount();
    const int simdCount = count & ~7; // 8 路批处理对齐数量

    const __m256 dtV  = _mm256_set1_ps(dt);
    const __m256 gxV  = _mm256_set1_ps(gravity.getX());
    const __m256 gyV  = _mm256_set1_ps(gravity.getY());
    const __m256 zeroV = _mm256_setzero_ps();
    const __m256i awakeBitV = _mm256_set1_epi32((int)FLAG_AWAKE);
    const __m256i zeroIV = _mm256_setzero_si256();

    float* posX = positionsX.data();
    float* posY = positionsY.data();
    float* velX = velocitiesX.data();
    float* velY = velocitiesY.data();
    float* rot = rotations.data();
    float* angVel = angularVels.data();
    const float* invM = invMasses.data();
    const float* invI = invInertias.data();
    float* frcX = forcesX.data();
    float* frcY = forcesY.data();
    float* trq = torques.data();
    const float* gs = gravityScales.data();
    const uint32_t* flg = flags.data();
    Body** owners = bodyOwners.data();

    // 1. AVX2 8 路并发主循环（免分支：静态/睡眠刚体用掩码 blend 保持原值）
    for (int i = 0; i < simdCount; i += 8)
    {
        // 动态刚体掩码：invMass > 0 且 AWAKE 的通道才推进
        const __m256 invMV = _mm256_loadu_ps(&invM[i]);
        const __m256 invMask = _mm256_cmp_ps(invMV, zeroV, _CMP_GT_OQ);
        const __m256i awakeBitsI = _mm256_and_si256(_mm256_loadu_si256((const __m256i*)&flg[i]), awakeBitV);
        const __m256i awakeMaskI = _mm256_cmpgt_epi32(awakeBitsI, zeroIV);
        const __m256 mask = _mm256_and_ps(invMask, _mm256_castsi256_ps(awakeMaskI));

        // ---- X 方向：a_x = FMA(Fx, m^-1, gx*gs)，v_x = FMA(a_x, dt, v_x) ----
        const __m256 fxV = _mm256_loadu_ps(&frcX[i]);
        const __m256 velXV = _mm256_loadu_ps(&velX[i]);
        const __m256 posXV = _mm256_loadu_ps(&posX[i]);
        const __m256 gsV = _mm256_loadu_ps(&gs[i]);
        const __m256 axV = _mm256_fmadd_ps(fxV, invMV, _mm256_mul_ps(gxV, gsV));
        __m256 newVelX = _mm256_blendv_ps(velXV, _mm256_fmadd_ps(axV, dtV, velXV), mask);
        __m256 newPosX = _mm256_blendv_ps(posXV, _mm256_fmadd_ps(newVelX, dtV, posXV), mask);

        // ---- Y 方向（同构） ----
        const __m256 fyV = _mm256_loadu_ps(&frcY[i]);
        const __m256 velYV = _mm256_loadu_ps(&velY[i]);
        const __m256 posYV = _mm256_loadu_ps(&posY[i]);
        const __m256 ayV = _mm256_fmadd_ps(fyV, invMV, _mm256_mul_ps(gyV, gsV));
        __m256 newVelY = _mm256_blendv_ps(velYV, _mm256_fmadd_ps(ayV, dtV, velYV), mask);
        __m256 newPosY = _mm256_blendv_ps(posYV, _mm256_fmadd_ps(newVelY, dtV, posYV), mask);

        // ---- 角速度与角度：w = FMA(t, I^-1·dt, w)，theta = FMA(w, dt, theta) ----
        const __m256 trqV = _mm256_loadu_ps(&trq[i]);
        const __m256 angVelV = _mm256_loadu_ps(&angVel[i]);
        const __m256 rotV = _mm256_loadu_ps(&rot[i]);
        const __m256 invIV = _mm256_loadu_ps(&invI[i]);
        __m256 newAngVel = _mm256_blendv_ps(angVelV, _mm256_fmadd_ps(trqV, _mm256_mul_ps(invIV, dtV), angVelV), mask);
        __m256 newRot = _mm256_blendv_ps(rotV, _mm256_fmadd_ps(newAngVel, dtV, rotV), mask);

        // ---- P0 快照（CCD 需要）：8 通道全写——被掩码的静态/睡眠刚体
        //      写"当前位姿"与 World 的静态/睡眠 prev 同步循环幂等一致。
        //      直接 x/y 成员写，杜绝 Debug 下 Vector2 构造的函数调用开销 ----
        Body** own = &owners[i];
        for (int k = 0; k < 8; ++k) {
            own[k]->m_prevPosition.x = posX[i + k];
            own[k]->m_prevPosition.y = posY[i + k];
            own[k]->m_prevRotation = rot[i + k];
        }

        // ---- 力/扭矩积分后清零（仅推进的通道清零，掩码通道保持旧值） ----
        _mm256_storeu_ps(&frcX[i], _mm256_blendv_ps(fxV, zeroV, mask));
        _mm256_storeu_ps(&frcY[i], _mm256_blendv_ps(fyV, zeroV, mask));
        _mm256_storeu_ps(&trq[i], _mm256_blendv_ps(trqV, zeroV, mask));

        // ---- 高速存回内存 ----
        _mm256_storeu_ps(&velX[i], newVelX);
        _mm256_storeu_ps(&posX[i], newPosX);
        _mm256_storeu_ps(&velY[i], newVelY);
        _mm256_storeu_ps(&posY[i], newPosY);
        _mm256_storeu_ps(&angVel[i], newAngVel);
        _mm256_storeu_ps(&rot[i], newRot);
    }

    // 2. 标量尾部循环：处理剩余 count % 8 个刚体（任意数量 100% 正确）
    const float gx = gravity.getX(), gy = gravity.getY();
    for (int i = simdCount; i < count; ++i) {
        if (invM[i] == 0.0f || !(flg[i] & FLAG_AWAKE)) continue;
        velX[i] += (frcX[i] * invM[i] + gx * gs[i]) * dt;
        velY[i] += (frcY[i] * invM[i] + gy * gs[i]) * dt;
        angVel[i] += trq[i] * invI[i] * dt;

        Body* b = owners[i];
        b->m_prevPosition.x = posX[i];
        b->m_prevPosition.y = posY[i];
        b->m_prevRotation = rot[i];

        posX[i] += velX[i] * dt;
        posY[i] += velY[i] * dt;
        rot[i] += angVel[i] * dt;

        frcX[i] = 0.0f;
        frcY[i] = 0.0f;
        trq[i] = 0.0f;
    }
#else
    // 编译期未开启 /arch:AVX2：直接标量回退
    (void)dt; (void)gravity;
    IntegrateScalar(dt, gravity);
#endif
}
