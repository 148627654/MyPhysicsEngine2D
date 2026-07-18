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
	m->contactCount = 1;
	if (distance != 0)//圆心没重合
	{
		m->penetration = relativeradius - distance;//穿透深度
		//碰撞法线,归一化
		m->normal = relativePos / distance;
		m->contacts[0] = a->GetPosition() + m->normal * shapeA->getR();
		//m->contacts.push_back(a->GetPosition() + m->normal * shapeA->getR());
	}
	else//如果为0，就代表圆心重合
	{
		m->penetration = shapeA->getR();
		m->normal= Vector2(1, 0);       // 强制给个法线防止报错
		m->contacts[0] = a->GetPosition();
		//m->contacts.push_back(a->GetPosition());
	}
	// 初始化冲量缓存
	m->impulseN[0] = 0.0f;
	m->impulseT[0] = 0.0f;
	return true;
}

bool Collision::BoxVsBox(Manifold* m, Body* a, Body* b)
{
	float minOverlap = FLT_MAX;
	int bestAxisIndex = -1;

	auto aWorldVertices = GetBoxWorldVertices(a);
	auto bWorldVertices = GetBoxWorldVertices(b);
	Vector2 axes[4] = { GetBodyAxisX(a), GetBodyAxisY(a), GetBodyAxisX(b), GetBodyAxisY(b) };

	// 1. SAT 寻找最小穿透轴
	for (int i = 0; i < 4; i++) {
		Vector2 axis = axes[i];
		Projection projA = GetProjection(aWorldVertices, axis);
		Projection projB = GetProjection(bWorldVertices, axis);

		float overlap = GetOverlap(projA.min, projA.max, projB.min, projB.max);
		if (overlap <= 0) return false;

		if (overlap < minOverlap) {
			minOverlap = overlap;
			bestAxisIndex = i;
		}
	}

	m->penetration = minOverlap;
	Vector2 normal = axes[bestAxisIndex];

	// 确保法线从 A 指向 B
	Vector2 dir = b->GetPosition() - a->GetPosition();
	if (dir.Dot(normal) < 0) normal = normal * -1.0f;
	m->normal = normal;

	// 2. 确定哪个是参考物 (Reference)，哪个是入射物 (Incident)
	// 参考物：贡献了碰撞法线的物体
	// 入射物：提供碰撞顶点的物体
	Body* refBody;
	Body* incBody;
	if (bestAxisIndex < 2) { // 轴属于 A
		refBody = a;
		incBody = b;
	}
	else { // 轴属于 B
		refBody = b;
		incBody = a;
	}

	// 3. 寻找入射边 (Incident Edge)
	// 入射边是入射物上最对准法线反方向的那条边
	auto incVertices = (incBody == a) ? aWorldVertices : bWorldVertices;
	Vector2 incidentEdge[2];
	FindIncidentEdge(incidentEdge, incVertices, m->normal);

	// 4. 裁剪并填充接触点 (Clipping)
	// 这里使用简化逻辑：在入射边的两个顶点中，
	// 找出所有在“穿透范围内”的点
	m->contactCount = 0;

	// 检查入射边的第一个点是否在参考物内部
	if (IsPointInBody(incidentEdge[0], refBody)) {
		m->contacts[m->contactCount++] = incidentEdge[0];
	}
	// 检查入射边的第二个点是否在参考物内部
	if (IsPointInBody(incidentEdge[1], refBody)) {
		m->contacts[m->contactCount++] = incidentEdge[1];
	}

	// 如果由于数值误差一个点都没找到，退化到最深点逻辑
	if (m->contactCount == 0) {
		m->contacts[0] = incidentEdge[0];
		m->contactCount = 1;
	}

	// 初始化冲量（Day 12 核心）
	m->impulseN[0] = m->impulseN[1] = 0.0f;
	m->impulseT[0] = m->impulseT[1] = 0.0f;

	return true;
}

void Collision::FindIncidentEdge(Vector2 out[2], const std::vector<Vector2>& vertices, Vector2 normal) {
	// 寻找入射物上与法线最方向相反的顶点
	float minDot = FLT_MAX;
	int index = 0;
	for (int i = 0; i < 4; ++i) {
		float dot = vertices[i].Dot(normal);
		if (dot < minDot) {
			minDot = dot;
			index = i;
		}
	}

	// 入射边由该顶点及其相邻的两个顶点中更垂直于法线的那个组成
	Vector2 v1 = vertices[index];
	Vector2 v0 = vertices[(index + 3) % 4]; // 前一个点
	Vector2 v2 = vertices[(index + 1) % 4]; // 后一个点

	// 比较哪条边更垂直于法线
	if (std::abs(v1.Sub(v0).Normalize().Dot(normal)) <= std::abs(v1.Sub(v2).Normalize().Dot(normal))) {
		out[0] = v0; out[1] = v1;
	}
	else {
		out[0] = v1; out[1] = v2;
	}
}

bool Collision::IsPointInBody(Vector2 p, Body* b) {
	// 简单版：将点转换到物体的本地空间，判断是否在 AABB 范围内
	// 这对于旋转矩形也有效
	Vector2 localP = p - b->GetPosition();
	float angle = -b->GetRotation(); // 逆旋转
	float s = std::sin(angle);
	float c = std::cos(angle);

	float rotatedX = localP.getX() * c - localP.getY()* s;
	float rotatedY = localP.getX() * s + localP.getY() * c;

	Box* shape = static_cast<Box*>(b->GetShape());
	float hx = shape->getWidth() / 2.0f;
	float hy = shape->getHeigh() / 2.0f;

	return (std::abs(rotatedX) <= hx + 0.01f && std::abs(rotatedY) <= hy + 0.01f);
}

bool Collision::CircleVsBox(Manifold* m, Body* circlebody, Body* boxbody)
{
	Circle* circle = static_cast<Circle*>(circlebody->GetShape());
	Box* box = static_cast<Box*>(boxbody->GetShape());

	// 1. 世界坐标转 Box 局部坐标
	Vector2 relationPos = circlebody->GetPosition() - boxbody->GetPosition();
	Vector2 localCirclePos = relationPos.Rotate(-boxbody->GetRotation());

	float hw = box->getHalfWidth();
	float hh = box->getHalfHeigh();

	// 2. 确定矩形上距离圆心最近的点 (Closest Point)
	// C++11 替代 std::clamp 的写法：
	float closestX = std::max(-hw, std::min(localCirclePos.getX(), hw));
	float closestY = std::max(-hh, std::min(localCirclePos.getY(), hh));
	Vector2 closestPoint(closestX, closestY);

	// 3. 计算距离
	Vector2 distVec = localCirclePos - closestPoint; // 从最近点指向圆心
	float distSq = distVec.LengthSquared();
	float r = circle->getR();

	// 判定是否碰撞
	bool isInside = (std::abs(localCirclePos.getX()) <= hw && std::abs(localCirclePos.getY()) <= hh);
	if (distSq > r * r && !isInside)
		return false;

	// 填充流形基本信息
	m->bodyA = circlebody;
	m->bodyB = boxbody;
	m->contactCount = 1; // 圆与矩形碰撞始终只有 1 个点

	Vector2 localNormal;

	if (distSq > 0.0001f && !isInside) {
		// 情况 A: 圆心在矩形外
		float dist = std::sqrt(distSq);
		m->penetration = r - dist;
		// distVec 方向：Box -> Circle，我们需要 Circle -> Box，所以取负
		localNormal = distVec * (-1.0f / dist);
	}
	else {
		// 情况 B: 圆心在矩形内
		float distToRight = hw - localCirclePos.getX();
		float distToLeft = hw + localCirclePos.getX();
		float distToTop = hh - localCirclePos.getY();
		float distToBottom = hh + localCirclePos.getY();

		if (distToRight < distToLeft && distToRight < distToTop && distToRight < distToBottom) {
			localNormal = Vector2(1, 0);
			m->penetration = r + distToRight;
		}
		else if (distToLeft < distToTop && distToLeft < distToBottom) {
			localNormal = Vector2(-1, 0);
			m->penetration = r + distToLeft;
		}
		else if (distToTop < distToBottom) {
			localNormal = Vector2(0, 1);
			m->penetration = r + distToTop;
		}
		else {
			localNormal = Vector2(0, -1);
			m->penetration = r + distToBottom;
		}
	}

	// 4. 转回世界空间
	m->normal = localNormal.Rotate(boxbody->GetRotation());
	m->contacts[0] = closestPoint.Rotate(boxbody->GetRotation()) + boxbody->GetPosition();

	// 5. 初始化冲量缓存（Day 12 核心）
	m->impulseN[0] = 0.0f;
	m->impulseT[0] = 0.0f;

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
