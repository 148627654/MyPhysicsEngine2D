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
	auto bWorldVertices = GetBoxWorldVertices(b);
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

	// 填充流形基本信息
	m->bodyA = a;
	m->bodyB = b;
	m->penetration = minOverlap;
	m->normal = bestAxis;

	// 5. 修正法线方向：确保法线是从 A 指向 B
	Vector2 dir = b->GetPosition() - a->GetPosition();
	if (dir.Dot(m->normal) < 0) {
		m->normal = m->normal * -1.0f;
	}

	// --- [新增：解决崩溃与旋转的核心代码] ---
	m->contacts.clear();
	// 寻找 A 的 4 个顶点中，在法线方向上最深入 B 的点
	// 逻辑：法线是从 A 指向 B 的，所以 A 里面 Dot 值最大的点就是最靠近 B 的点
	float maxDot = -FLT_MAX;
	Vector2 bestContact = aWorldVertices[0];

	for (const auto& v : aWorldVertices) {
		float dot = v.Dot(m->normal);
		if (dot > maxDot) {
			maxDot = dot;
			bestContact = v;
		}
	}
	m->contacts.push_back(bestContact);
	// ---------------------------------------

	return true;
}

bool Collision::CircleVsBox(Manifold* m, Body* circlebody, Body* boxbody)
{
	Circle* circle = static_cast<Circle*>(circlebody->GetShape());
	Box* box = static_cast<Box*>(boxbody->GetShape());

	// 1. 世界坐标转 Box 局部坐标
	Vector2 relationPos = circlebody->GetPosition() - boxbody->GetPosition();
	Vector2 localCirclePos = relationPos.Rotate(-boxbody->GetRotation());

	float hw = box->getHalfWidth();
	float hh = box->getHalfHeigh(); // 修正拼写

	// 2. 确定矩形上距离圆心最近的点
	Vector2 closestPoint(
		std::max(-hw, std::min(localCirclePos.getX(), hw)),
		std::max(-hh, std::min(localCirclePos.getY(), hh))
	);

	// 3. 计算距离
	Vector2 distVec = localCirclePos - closestPoint;
	float distSq = distVec.LengthSquared();
	float r = circle->getR(); // 假设你的函数名是这个

	// 判定是否碰撞
	bool isInside = (std::abs(localCirclePos.getX()) <= hw && std::abs(localCirclePos.getY()) <= hh);

	if (distSq > r * r && !isInside)
		return false;

	m->bodyA = circlebody;
	m->bodyB = boxbody;

	Vector2 localNormal; // 提前声明局部法线

	if (distSq > 0.0001f) {
		// 情况 A: 圆心在矩形外
		float dist = std::sqrt(distSq);
		m->penetration = r - dist;
		// distVec 是从最近点指向圆心的，我们需要从圆指向矩形，所以取反
		localNormal = (distVec * -1.0f) / dist;
	}
	else {
		// 情况 B: 圆心在矩形内（穿透最深的情况）
		// 寻找距离圆心最近的边
		float distToRight = hw - localCirclePos.getX();
		float distToLeft = hw + localCirclePos.getX();
		float distToTop = hh - localCirclePos.getY();
		float distToBottom = hh + localCirclePos.getY();

		if (distToRight < distToLeft && distToRight < distToTop && distToRight < distToBottom) {
			localNormal = Vector2(1, 0); // 指向右
			m->penetration = r + distToRight;
		}
		else if (distToLeft < distToTop && distToLeft < distToBottom) {
			localNormal = Vector2(-1, 0); // 指向左
			m->penetration = r + distToLeft;
		}
		else if (distToTop < distToBottom) {
			localNormal = Vector2(0, 1); // 指向上
			m->penetration = r + distToTop;
		}
		else {
			localNormal = Vector2(0, -1); // 指向下
			m->penetration = r + distToBottom;
		}
	}

	// 4. 将局部法线旋转回世界坐标
	m->normal = localNormal.Rotate(boxbody->GetRotation());

	// 5. 统一计算接触点 (就是矩形上的那个点，转回世界空间)
	m->contacts.clear();
	m->contacts.push_back(closestPoint.Rotate(boxbody->GetRotation()) + boxbody->GetPosition());

	return true;
}

bool Collision::AABBvsAABB(const AABB& a, const AABB& b) {
	// 逻辑：如果 A 在 B 的右边、左边、上边或下边，则一定没撞
	if (a.max.getX() < b.min.getX() || a.min.getX() > b.max.getX()) return false;
	if (a.max.getY() < b.min.getY() || a.min.getY() > b.max.getY()) return false;

	// 否则，说明重叠了
	return true;
}

bool Collision::Dispatch(Manifold* m, Body* a, Body* b) {
	Shape::Type typeA = a->GetShape()->type;
	Shape::Type typeB = b->GetShape()->type;

	// 1. 同类型碰撞
	if (typeA == Shape::Type::type_Circle && typeB == Shape::Type::type_Circle) return Collision::CircleVsCircle(m, a, b);
	if (typeA == Shape::Type::type_Box && typeB == Shape::Type::type_Box) return Collision::BoxVsBox(m, a, b);

	// 2. 混合类型碰撞 (Circle vs Box)
	if (typeA == Shape::Type::type_Circle && typeB == Shape::Type::type_Box) {
		return Collision::CircleVsBox(m, a, b);
	}

	// 3. 混合类型碰撞 (Box vs Circle) -> 巧妙交换参数
	if (typeA == Shape::Type::type_Box && typeB == Shape::Type::type_Circle) {
		// 交换 a, b 顺序调用，并把法线反向
		bool hit = Collision::CircleVsBox(m, b, a);
		if (hit) m->normal = m->normal * -1.0f;
		return hit;
	}

	return false;
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
