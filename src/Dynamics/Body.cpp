#include "Body.h"

Vector2 Body::addForce(Vector2 f)
{
    this->force += f;
    setAwake(true); // 被推了一把，必须醒来
    return this->force;
}

void Body::setPosition(float x, float y)
{
    position = Vector2(x, y);updateAABB();
    setAwake(true); // 被推了一把，必须醒来
}

void Body::setPosition(const Vector2& v)
{
    position = v;
    setAwake(true); // 被推了一把，必须醒来
    updateAABB();

}

void Body::setPositionQuiet(const Vector2& v)
{
    position = v;
    updateAABB();
}

void Body::setRotationQuiet(float r)
{
    rotation = r;
    updateAABB();
}

void Body::setRotation(float r)
{
    rotation = r;
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
}

void Body::applyForceAtPoint(Vector2 force, Vector2 worldPoint)
{
    addForce(force);
    //施加力之后肯定会跑偏
    Vector2 r = worldPoint - this->position;

    // 在 2D 中，叉乘公式：x1*y2 - y1*x2
    float t = r.getX() * force.getY() - r.getY() * force.getX();
    setAwake(true); // 被推了一把，必须醒来
    addTorque(t);
}

void Body::updateAABB()
{
    if (shape)
        worldAABB = shape->computeAABB(position, rotation);
}

void Body::applyImpulse(Vector2 impulse)
{
    if (invMass == 0.0f) return;
    velocity += impulse * invMass;
}

void Body::applyImpulse(const Vector2& impulse, const Vector2& contactVector) {
    if (invMass == 0.0f) return;
    velocity += impulse * invMass;
    angularVelocity += invInertia * Vector2::cross(contactVector, impulse);
}

void Body::setAwake(bool w) {
    if (invMass == 0.0f) return;
    if (w) {
        m_isAwake = true;
        m_sleepTimer = 0.0f; // 只要醒了，重新开始计算“疲劳值”
    }
    else {
        m_isAwake = false;
        m_sleepTimer = 0.0f;
        // 入睡时，理论上速度应该已经被归零（在 Island::solve 里处理了）
    }
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
        this->velocity.clear();
        this->angularVelocity = 0.0f;
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

    updateAABB();
}

void Body::forceSleep() {
    // 1. 静态物体不需要强制睡眠，因为它本身就不醒
    if (invMass == 0.0f) return;

    // 2. 状态强转
    m_isAwake = false;

    // 3. 必须清空所有中间状态
    m_sleepTimer = 0.0f;
    force.clear();
    torque = 0.0f;

    // 4. 必须物理性停稳
    velocity.clear();
    angularVelocity = 0.0f;
}

AABB Body::getSweptAABB(float dt) const
{
	AABB aabb1 = this->getAABB();

    //预测下一帧的位置
	Vector2 predictedPosition = this->getPosition() + this->getVelocity() * dt;
	AABB aabb2 = shape->computeAABB(predictedPosition, this->getRotation());

	return AABB::combine(aabb1, aabb2);
}

Transform Body::getTransform(float alpha, float dt)
{
    Transform tf;
    // alpha = 0 对应 m_prevPosition (本帧开始)
    // alpha = 1 对应 m_position (本帧结束)
    tf.p = m_prevPosition + (position - m_prevPosition) * alpha;
    tf.q = m_prevRotation + (rotation - m_prevRotation) * alpha;
    return tf;
}

void Body::savePrevState()
{
    m_prevPosition = position;
    m_prevRotation = rotation;
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
    this->position = position;
    this->rotation = angle;

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
        return;
    }

    MassData data = shape->computeMass(shape->material.density);
    mass = data.mass;
    invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    inertia = data.inertia;
    invInertia = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;
}