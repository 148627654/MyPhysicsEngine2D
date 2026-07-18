#include "Body.h"

Vector2 Body::AddForce(Vector2 f)
{
    this->force += f;
    return this->force;
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
