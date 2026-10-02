#pragma once
#include <iostream>
#include <Vector2.h>
/// @brief 二维向量：引擎的基础数学类型
///
/// 提供加减乘除、点积（dot）、叉积（cross）、长度（length/lengthSquared）、
/// 归一化（normalize）、旋转（rotate）、左右法线（getLeftNormal/getRightNormal）。
class Vector2
{
public:
	~Vector2() = default;
	constexpr Vector2(float x = 0.0, float y = 0.0) :x(x), y(y) {}
	//向量与向量
	Vector2 operator+(const Vector2& val)const;
	Vector2 operator-(const Vector2& val)const;
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
	inline void setX(float x_) { x = x_; }
	inline float getY() const { return y; }
	inline void setY(float y_) { y = y_; }

	//长度计算
	inline float length() const{ return std::sqrt(x * x + y * y); }
	inline float lengthSquared()const { return x * x + y * y; }
	//归一化
	Vector2 normalize()const;
	//点积
	float dot(const Vector2& v) const;
	//距离
	float distance(const Vector2& v) const;
	float DistanceSquared(const Vector2& v) const;
	//清空
	inline void clear() { x = 0;y = 0; }
	//垂直向量
	Vector2 getLeftNormal() const { return Vector2(-y, x); }
	Vector2 getRightNormal() const { return Vector2(y, -x); }
	// 旋转
	Vector2 rotate(float angle) const;

	//叉积简化
	static float cross(const Vector2& a , const Vector2& b);
	static Vector2 cross(const Vector2& a , const float b);
	static Vector2 cross(const float a,const Vector2& b);

	// 在 Vector2 类定义内部添加
	inline Vector2 add(const Vector2& v) const { return Vector2(x + v.getX( ) , y + v.getY( )); }
	inline Vector2 sub(const Vector2& v) const { return Vector2(x - v.getX( ) , y - v.getY( )); }
	inline Vector2 Mul(float s) const { return Vector2(x * s , y * s); }
public:
	float x;
	float y;
};