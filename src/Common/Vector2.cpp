#include "Vector2.h"


Vector2 Vector2::operator+(const Vector2& val)
{
	Vector2 temp;
	temp.x = x + val.x;
	temp.y = y + val.y;
	return temp;
}

Vector2 Vector2::operator-(const Vector2& val)
{
	Vector2 temp;
	temp.x = x - val.x;
	temp.y = y - val.y;
	return temp;
}

Vector2& Vector2::operator+=(const Vector2& val)
{
	this->x += val.x;
	this->y += val.y;
	return *this;
}

Vector2& Vector2::operator-=(const Vector2& val)
{
	this->x -= val.x;
	this->y -= val.y;
	return *this;
}

Vector2 Vector2::operator*(float num) const
{
	Vector2 temp;
	temp.x = x * num;
	temp.y = y * num;
	return temp;
}

Vector2 Vector2::operator/(float& num)
{
	Vector2 temp;
	temp.x = x / num;
	temp.y = y / num;
	return temp;
}

Vector2& Vector2::operator*=(float& num)
{
	this->x *= num;
	this->y *= num;
	return *this;
}

Vector2& Vector2::operator/=(float& num)
{
	this->x /= num;
	this->y /= num;
	return *this;
}

Vector2 Vector2::operator-() const
{
	return Vector2(-x, -y);
}

Vector2 Vector2::Normalize() const
{
	float len = Length();
	if (len > Vector2::EPSILON) {
		float invLen = 1.0f / len;
		return Vector2(x * invLen, y * invLen);
	}
	return Vector2(0, 0); // 如果太小，返回零向量
}

float Vector2::Dot(const Vector2& v) const
{
	float x_val = this->x * v.x;
	float y_val = this->y * v.y;
	return x_val + y_val;
}

float Vector2::Distance(const Vector2& v) const
{
	return std::sqrt(DistanceSquared(v));
}

float Vector2::DistanceSquared(const Vector2& v) const
{
	float x_val = this->x - v.x;
	float y_val = this->y - v.y;
	return x_val * x_val + y_val * y_val;
}

Vector2 Vector2::Rotate(float angle) const
{
	float s = std::sin(angle);
	float c = std::cos(angle);

	float newX = x * c - y * s;
	float newY = x * s + y * c;

	return Vector2(newX, newY);
}


Vector2 operator*(float num, const Vector2& vec)
{
	return Vector2(vec.x * num, vec.y * num);
}

bool operator==(const Vector2& a, const Vector2& b)
{
	if ((std::abs(a.x - b.x) < Vector2::EPSILON) && (std::abs(a.y - b.y) < Vector2::EPSILON))
		return true;
	else
		return false;
}

bool operator!=(const Vector2& a, const Vector2& b)
{
	return !(a == b);
}
