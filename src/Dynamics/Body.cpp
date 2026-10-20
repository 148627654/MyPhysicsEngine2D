#include "Body.h"

Vector2 Body::addForce(Vector2 f)
{
    addSlotForceV(f);
    setAwake(true); // 被推了一把，必须醒来
    return getForce();
}

void Body::setPosition(float x, float y)
{
    setSlotPosV(Vector2(x, y)); updateAABB();
    setAwake(true); // 被推了一把，必须醒来
}

void Body::setPosition(const Vector2& v)
{
    setSlotPosV(v);
    setAwake(true); // 被推了一把，必须醒来
    updateAABB();

}

void Body::setPositionQuiet(const Vector2& v)
{
    setSlotPosV(v);
    updateAABB();
}

void Body::setRotationQuiet(float r)
{
    setSlotRotV(r);
    updateAABB();
}

void Body::setRotation(float r)
{
    setSlotRotV(r);
    setAwake(true); // 被推了一把，必须醒来
    updateAABB();
}
void Body::setShape(Shape* s, float density)
{
    this->shape = s;
    s->material.density = density; // density 写回材质，保持唯一数据源
    MassData data = s->computeMass(density);
    this->mass = data.mass;
    if (this->mass > 0)
        this->invMass = (1.0f / this->mass);
    else
        this->invMass = 0.0f;
    this->inertia = data.inertia;
    if (this->inertia > 0)
        this->invInertia = 1.0f / (this->inertia);
    else
        this->invInertia = 0.0f;
    syncMassToSoA();
}

void Body::applyForceAtPoint(Vector2 force, Vector2 worldPoint)
{
    addForce(force);
    //施加力之后肯定会跑偏
    Vector2 r = worldPoint - getPosition();

    // 在 2D 中，叉乘公式：x1*y2 - y1*x2
    float t = r.getX() * force.getY() - r.getY() * force.getX();
    setAwake(true); // 被推了一把，必须醒来
    addTorque(t);
}

void Body::updateAABB()
{
    if (shape)
        worldAABB = shape->computeAABB(getPosition(), getRotation());
}

void Body::applyImpulse(Vector2 impulse)
{
    if (getInvMass() == 0.0f) return;
    addSlotVelV(impulse * getInvMass());
}

void Body::applyImpulse(const Vector2& impulse, const Vector2& contactVector) {
    if (getInvMass() == 0.0f) return;
    addSlotVelV(impulse * getInvMass());
    addSlotAngVelV(getInvInertia() * Vector2::cross(contactVector, impulse));
}

void Body::setAwake(bool w) {
    if (getInvMass() == 0.0f) return;
    if (w) {
        m_isAwake = true;
        m_sleepTimer = 0.0f; // 只要醒了，重新开始计算“疲劳值”
    }
    else {
        m_isAwake = false;
        m_sleepTimer = 0.0f;
        // 入睡时，理论上速度应该已经被归零（在 Island::solve 里处理了）
    }
    syncAwakeFlag();
}

void Body::setType(BodyType type, float density) {
    if (type == BodyType::Static) {
        // --- 切换到静态 ---
        this->mass = 0.0f;
        this->invMass = 0.0f;
        this->inertia = 0.0f;
        this->invInertia = 0.0f;

        // 静态物体必须强制进入非清醒状态，且不参与睡眠计时
        this->m_isAwake = false;
        this->m_sleepTimer = 0.0f;
        setSlotVelV(Vector2(0.0f, 0.0f));
        setSlotAngVelV(0.0f);
    }
    else {
        // --- 切换到动态 ---
        // 1. 重新计算物理属性
        shape->material.density = density; // density 写回材质，保持唯一数据源
        MassData data = shape->computeMass(density);
        this->mass = data.mass;
        this->invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
        this->inertia = data.inertia;
        this->invInertia = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;

        // 2. 【核心新增】：自动唤醒
        setAwake(true);
    }

    syncMassToSoA();
    syncAwakeFlag();
    updateAABB();
}

void Body::forceSleep() {
    // 1. 静态物体不需要强制睡眠，因为它本身就不醒
    if (getInvMass() == 0.0f) return;

    // 2. 状态强转
    m_isAwake = false;

    // 3. 必须清空所有中间状态
    m_sleepTimer = 0.0f;
    clearSlotForceV();
    setSlotTorqueV(0.0f);

    // 4. 必须物理性停稳
    setSlotVelV(Vector2(0.0f, 0.0f));
    setSlotAngVelV(0.0f);
    syncAwakeFlag();
}

AABB Body::getSweptAABB(float dt) const
{
	AABB aabb1 = this->getAABB();

    //预测下一帧的位置与角度（平移 + 旋转联合扫掠，包住整条时空轨迹）
	Vector2 predictedPosition = this->getPosition() + this->getVelocity() * dt;
	float predictedRotation = this->getRotation() + this->getAngularVelocity() * dt;
	AABB aabb2 = shape->computeAABB(predictedPosition, predictedRotation);

	return AABB::combine(aabb1, aabb2);
}

// CCD 轨迹 AABB：联合本帧起始 (P0) 与当前 (P1) 位姿的包围盒。
// 与 getSweptAABB（前向预测）不同：TOI 求解器在 P0->P1 区间插值，
// 接触对分离判定必须覆盖这条已经走过的轨迹，否则高速子弹离散位置越过
// 薄墙后（前向预测落在墙另一侧）接触对会被误销毁，CCD 无对可查 -> 穿墙
AABB Body::getTrajectoryAABB() const
{
	AABB aabb0 = shape->computeAABB(m_prevPosition, m_prevRotation);
	return AABB::combine(aabb0, getAABB());
}

Transform Body::getTransform(float alpha, float dt)
{
    Transform tf;
    // alpha = 0 对应 m_prevPosition (本帧开始)
    // alpha = 1 对应 m_position (本帧结束)
    tf.p = m_prevPosition + (getPosition() - m_prevPosition) * alpha;
    tf.q = m_prevRotation + (getRotation() - m_prevRotation) * alpha;
    return tf;
}

void Body::savePrevState()
{
    m_prevPosition = getPosition();
    m_prevRotation = getRotation();
}

void Body::removeJoint(Joint* joint)
{
    auto it = std::find(m_joints.begin(), m_joints.end(), joint);
    if (it != m_joints.end()) {
        m_joints.erase(it);
    }
}

void Body::setTransform(const Vector2& position, float angle) {
    // 1. 更新底层位姿数据
    setSlotPosV(position);
    setSlotRotV(angle);

    // 2. 【核心】立即刷新该物体的 AABB
    // 物理引擎的碰撞检测依赖 AABB，如果坐标变了 AABB 不变，
    // 那么接下来的窄相检测（Narrowphase）依然会基于错误的位置。
    this->updateAABB();
}

void Body::setTransform(const Transform& tf) {
    // 直接复用上面的逻辑
    setTransform(tf.p, tf.q);
}

void Body::updateMassData()
{
    // 运行时修改 shape->material.density 后调用，重新计算质量属性
    if (shape == nullptr) {
        mass = 0.0f;
        invMass = 0.0f;
        inertia = 0.0f;
        invInertia = 0.0f;
        syncMassToSoA();
        return;
    }

    MassData data = shape->computeMass(shape->material.density);
    mass = data.mass;
    invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    inertia = data.inertia;
    invInertia = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;
    syncMassToSoA();
}