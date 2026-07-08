#pragma once
#include <iostream>
class Vector2
{
public:
	~Vector2() = default;
	constexpr Vector2(float x = 0.0, float y = 0.0) :x(x), y(y) {}
	//向量与向量
	Vector2 operator+(const Vector2& val);
	Vector2 operator-(const Vector2& val);
	Vector2& operator+=(const Vector2& val);
	Vector2& operator-=(const Vector2& val);
	//向量与float
	Vector2 operator*(float num) const;
	friend Vector2 operator*(float num, const Vector2& vec); 
	Vector2 operator/(float num)const;
	Vector2& operator*=(float num);
	Vector2& operator/=(float num);
	//单目运算符
	Vector2 operator-() const;
	//相等判断
	friend bool operator==(const Vector2& a, const Vector2& b);
	friend bool operator!=(const Vector2& a, const Vector2& b);

	//调试
	inline float getX() const { return x; }
	inline float getY() const { return y; }

	//长度计算
	inline float Length() const{ return std::sqrt(x * x + y * y); }
	inline float LengthSquared()const { return x * x + y * y; }
	//归一化
	Vector2 Normalize()const;
	//点积
	float Dot(const Vector2& v) const;
	//距离
	float Distance(const Vector2& v) const;
	float DistanceSquared(const Vector2& v) const;
	//清空
	inline void Clear() { x = 0;y = 0; }
	//垂直向量
	Vector2 GetLeftNormal() const { return Vector2(-y, x); }
	Vector2 GetRightNormal() const { return Vector2(y, -x); }
	// 旋转
	Vector2 Rotate(float angle) const;
private:
	float x;
	float y;
	static constexpr  float EPSILON = 0.0001f;
};