#include "Collision.h"

#include "../Dynamics/Body.h"
#include "../Collision/Circle.h"
#include "../Collision/Box.h"
#include <vector>
bool Collision::CircleVsCircle(Manifold* m, Body* a, Body* b)
{
	//获取两个实体
	Circle* shapeA = static_cast<Circle*>(a->GetShape());
	Circle* shapeB = static_cast<Circle*>(b->GetShape());

	// relativePos=|PosB-PosA|
	Vector2 relativePos = b->GetPosition() - a->GetPosition();
	float disSq = relativePos.LengthSquared();
	float relativeradius = shapeA->getR() + shapeB->getR();
	//没碰住
	if (disSq >= relativeradius*relativeradius)
		return false;
	//碰到了
	float distance = std::sqrt(disSq);
	if (distance != 0)//圆心没重合
	{
		m->penetration = relativeradius - distance;//穿透深度
		//碰撞法线,归一化
		m->normal = relativePos / distance;
		m->contacts.push_back(a->GetPosition() + m->normal * shapeA->getR());
	}
	else//如果为0，就代表圆心重合
	{
		m->penetration = shapeA->getR();
		m->normal= Vector2(1, 0);       // 强制给个法线防止报错
		m->contacts.push_back(a->GetPosition());
	}
	return true;
}

bool Collision::BoxVsBox(Manifold* m, Body* a, Body* b)
{
	float minOverlap = FLT_MAX; // 初始化为一个很大的数
	Vector2 bestAxis;           // 记录产生最小重叠的轴
	//A,B的四个点，AB分别两个轴
	auto aWorldVertices = GetBoxWorldVertices(a);
	auto bWorldVertices=GetBoxWorldVertices(b);
	Vector2 axes[4] = { GetBodyAxisX(a), GetBodyAxisY(a), GetBodyAxisX(b), GetBodyAxisY(b) };

	for (int i = 0; i < 4; i++) {
		Vector2 axis = axes[i];
		Projection projA = GetProjection(aWorldVertices, axis);
		Projection projB = GetProjection(bWorldVertices, axis);

		float overlap = GetOverlap(projA.min, projA.max, projB.min, projB.max);

		if (overlap < 0) {
			// 只要发现一条轴没重叠，立刻判定没撞
			return false;
		}

		// 3. 寻找最小穿透量 (MTV)
		if (overlap < minOverlap) {
			minOverlap = overlap;
			bestAxis = axis;
		}
	}

	//撞了
	m->bodyA = a;
	m->bodyB = b;
	m->penetration = minOverlap;
	m->normal = bestAxis;

	// 5. 修正法线方向：确保法线是从 A 指向 B
	Vector2 dir = b->GetPosition() - a->GetPosition();
	if (dir.Dot(m->normal) < 0) {
		m->normal = m->normal * -1.0f;
	}

	return true;
}

std::vector<Vector2> Collision::GetBoxWorldVertices(const Body* body)
{
	Box* box = static_cast<Box*>(body->GetShape());//获取box
	//获取四个坐标
	//获取原点+/width2+height/2,旋转角
	Vector2 pos = body->GetPosition();
	float halfwidth= box->getHalfWidth();
	float halfheight = box->getHalfHeigh();
	float rotation =body->GetRotation();
	//左上
	Vector2 leftUp = Vector2(- halfwidth, halfheight);
	//右上
	Vector2 rightUp = Vector2( halfwidth, halfheight);
	//左下
	Vector2 leftDown = Vector2(-halfwidth, - halfheight);
	//右下
	Vector2 rightDown = Vector2( halfwidth,  - halfheight);

	Vector2 Vertices[4];
	Vertices[0] = leftUp;
	Vertices[1] = rightUp;
	Vertices[2] = leftDown;
	Vertices[3] = rightDown;

	std::vector<Vector2> worldVertices;
	worldVertices.push_back(pos + leftUp.Rotate(rotation));
	worldVertices.push_back(pos + rightUp.Rotate(rotation));
	worldVertices.push_back(pos + leftDown.Rotate(rotation));
	worldVertices.push_back(pos + rightDown.Rotate(rotation));
	return worldVertices;
}

Projection Collision::GetProjection(const std::vector<Vector2>& vertices, const Vector2& axis)
{
	float dot =vertices[0].Dot(axis);
	float min = dot;
	float max = dot;
	for (int i = 1;i < vertices.size();++i)
	{
		dot=vertices[i].Dot(axis);
		min = (min < dot) ? min : dot;
		max = (max > dot) ? max : dot;
	}
	return { min,max };
}

float Collision::GetOverlap(float minA, float maxA, float minB, float maxB)
{
	// 逻辑：是否有缝隙？
	if (maxA < minB || maxB < minA) {
		return -1.0f; // 返回负值表示没有重叠（即存在分离轴）
	}
	// 计算重叠部分的长度
	// 公式：两个 max 中的较小值 - 两个 min 中的较大值
	return std::min(maxA, maxB) - std::max(minA, minB);
}
