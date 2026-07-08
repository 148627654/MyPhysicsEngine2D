#include "Body.h"

Vector2 Body::AddForce(Vector2 f)
{
    this->force += f;
    return this->force;
}
