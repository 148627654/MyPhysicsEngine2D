#include "Body.h"

Vector2 Body::AddForce(Vector2 f)
{
    this->force += f;
    setAwake(true); // 被推了一把，必须醒来
    return this->force;
}

void Body::SetPosition(float x, float y)
{
    position = Vector2(x, y);updateAABB();
    setAwake(true); // 被推了一把，必须醒来
}

void Body::SetPosition(const Vector2& v)
{
    position = v; 
    setAwake(true); // 被推了一把，必须醒来
    updateAABB();
    
}

void Body::SetRotation(float r)
{
    rotation = r;
    setAwake(true); // 被推了一把，必须醒来
    updateAABB();
}
void Body::SetShape(Shape* s, float density)
{
    this->shape = s;
    MassData data = s->ComputeMass(density);
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

void Body::ApplyForceAtPoint(Vector2 force, Vector2 worldPoint)
{
    AddForce(force);
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
        worldAABB = shape->ComputeAABB(position, rotation);
}

void Body::ApplyImpulse(Vector2 impulse)
{
    velocity += impulse * invMass;
}

void Body::ApplyImpulse(const Vector2& impulse, const Vector2& contactVector) {
    velocity += impulse * invMass;
    angularVelocity += invInertia * Vector2::Cross(contactVector, impulse);
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
        // 入睡时，理论上速度应该已经被归零（在 Island::Solve 里处理了）
    }
}

void Body::SetType(BodyType type, float density) {
    if (type == BodyType::Static) {
        // --- 切换到静态 ---
        this->mass = 0.0f;
        this->invMass = 0.0f;
        this->inertia = 0.0f;
        this->invInertia = 0.0f;

        // 静态物体必须强制进入非清醒状态，且不参与睡眠计时
        this->m_isAwake = false;
        this->m_sleepTimer = 0.0f;
        this->velocity.Clear();
        this->angularVelocity = 0.0f;
    }
    else {
        // --- 切换到动态 ---
        // 1. 重新计算物理属性
        MassData data = shape->ComputeMass(density);
        this->mass = data.mass;
        this->invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
        this->inertia = data.inertia;
        this->invInertia = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;

        // 2. 【核心新增】：自动唤醒
        setAwake(true);
    }

    updateAABB();
}

void Body::ForceSleep() {
    // 1. 静态物体不需要强制睡眠，因为它本身就不醒
    if (invMass == 0.0f) return;

    // 2. 状态强转
    m_isAwake = false;

    // 3. 必须清空所有中间状态
    m_sleepTimer = 0.0f;
    force.Clear();
    torque = 0.0f;

    // 4. 【非常重要】：必须物理性停稳
    velocity.Clear();
    angularVelocity = 0.0f;
}