#include "Collision.h"

#include "../Dynamics/Body.h"
#include "../Collision/Circle.h"
#include "../Collision/Box.h"
#include <vector>
bool Collision::circleVsCircle(Manifold* m, Body* a, Body* b)
{
	//获取两个实体
	Circle* shapeA = static_cast<Circle*>(a->getShape());
	Circle* shapeB = static_cast<Circle*>(b->getShape());

	// relativePos=|PosB-PosA|
	Vector2 relativePos = b->getPosition() - a->getPosition();
	float disSq = relativePos.lengthSquared();
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
		m->contacts[0] = a->getPosition() + m->normal * shapeA->getR();
		//m->contacts.push_back(a->getPosition() + m->normal * shapeA->getR());
	}
	else//如果为0，就代表圆心重合
	{
		m->penetration = shapeA->getR();
		m->normal= Vector2(1, 0);       // 强制给个法线防止报错
		m->contacts[0] = a->getPosition();
		//m->contacts.push_back(a->getPosition());
	}
	// 初始化冲量缓存
	m->impulseN[0] = 0.0f;
	m->impulseT[0] = 0.0f;
	return true;
}

bool Collision::boxVsBox(Manifold* m, Body* a, Body* b)
{
	float minOverlap = FLT_MAX;
	int bestAxisIndex = -1;

	auto aWorldVertices = getBoxWorldVertices(a);
	auto bWorldVertices = getBoxWorldVertices(b);
	Vector2 axes[4] = { getBodyAxisX(a), getBodyAxisY(a), getBodyAxisX(b), getBodyAxisY(b) };

	// 1. SAT 寻找最小穿透轴
	for (int i = 0; i < 4; i++) {
		Vector2 axis = axes[i];
		Projection projA = getProjection(aWorldVertices, axis);
		Projection projB = getProjection(bWorldVertices, axis);

		float overlap = getOverlap(projA.min, projA.max, projB.min, projB.max);
		if (overlap <= 0) return false;

		if (overlap < minOverlap) {
			minOverlap = overlap;
			bestAxisIndex = i;
		}
	}

	m->penetration = minOverlap;
	Vector2 normal = axes[bestAxisIndex];

	// 确保法线从 A 指向 B
	Vector2 dir = b->getPosition() - a->getPosition();
	if (dir.dot(normal) < 0) normal = normal * -1.0f;
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
	findIncidentEdge(incidentEdge, incVertices, m->normal);

	// 4. 裁剪并填充接触点 (Clipping)
	// 这里使用简化逻辑：在入射边的两个顶点中，
	// 找出所有在“穿透范围内”的点
	m->contactCount = 0;

	// 检查入射边的第一个点是否在参考物内部
	if (isPointInBody(incidentEdge[0], refBody)) {
		m->contacts[m->contactCount++] = incidentEdge[0];
	}
	// 检查入射边的第二个点是否在参考物内部
	if (isPointInBody(incidentEdge[1], refBody)) {
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

void Collision::findIncidentEdge(Vector2 out[2], const std::vector<Vector2>& vertices, Vector2 normal) {
	// 寻找入射物上与法线最方向相反的顶点
	float minDot = FLT_MAX;
	int index = 0;
	for (int i = 0; i < 4; ++i) {
		float dot = vertices[i].dot(normal);
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
	if (std::abs(v1.sub(v0).normalize().dot(normal)) <= std::abs(v1.sub(v2).normalize().dot(normal))) {
		out[0] = v0; out[1] = v1;
	}
	else {
		out[0] = v1; out[1] = v2;
	}
}

bool Collision::isPointInBody(Vector2 p, Body* b) {
	// 简单版：将点转换到物体的本地空间，判断是否在 AABB 范围内
	// 这对于旋转矩形也有效
	Vector2 localP = p - b->getPosition();
	float angle = -b->getRotation(); // 逆旋转
	float s = std::sin(angle);
	float c = std::cos(angle);

	float rotatedX = localP.getX() * c - localP.getY()* s;
	float rotatedY = localP.getX() * s + localP.getY() * c;

	Box* shape = static_cast<Box*>(b->getShape());
	float hx = shape->getWidth() / 2.0f;
	float hy = shape->getHeight() / 2.0f;

	return (std::abs(rotatedX) <= hx + 0.01f && std::abs(rotatedY) <= hy + 0.01f);
}

// 1. 计算点 P 到线段 [A, B] 的最近点
Vector2 Collision::closestPointOnSegment(const Vector2& p, const Vector2& a, const Vector2& b) {
	Vector2 ab = b - a;
	float lenSq = ab.dot(ab);
	if (lenSq < 1e-6f) return a; // 端点重合退化为点
	float t = (p - a).dot(ab) / lenSq;
	t = std::max(0.0f, std::min(1.0f, t)); // 夹紧在 [0, 1] 之间
	return a.add(ab * t);
}

void Collision::findIncidentEdge(ClipVertex out[2], const Polygon* incPoly, const Body* incBody,
	const Vector2& refNormal)
{
	int count = incPoly->getVertexCount();
	float rot = incBody->getRotation();
	Vector2 pos = incBody->getPosition();

	int bestEdge = 0;
	float minDot = 1e30f;

	// 寻找单位外法线与 refNormal 点积最小（最相反）的边
	for (int i = 0; i < count; ++i) {
		Vector2 localN = incPoly->getNormal(i);
		Vector2 worldN = localN.rotate(rot);
		float dot = worldN.dot(refNormal);
		if (dot < minDot) {
			minDot = dot;
			bestEdge = i;
		}
	}

	// 提取附着边在世界坐标系下的两端点
	int i1 = bestEdge;
	int i2 = (bestEdge + 1) % count;

	out[0].v = pos + incPoly->getVertex(i1).rotate(rot);
	out[1].v = pos + incPoly->getVertex(i2).rotate(rot);
}

bool Collision::polygonVsPolygon(Manifold* m, Body* a, Body* b)
{
	Polygon* polyA = static_cast<Polygon*>(a->getShape());
	Polygon* polyB = static_cast<Polygon*>(b->getShape());

	// -------------------------------------------------------------
	// 第一步：双向 SAT 测试 (A 测 B 与 B 测 A)
	// -------------------------------------------------------------
	int edgeA = 0;
	float sepA = findMaxSeparation(edgeA, polyA, a, polyB, b);
	if (sepA > 0.0f) {
		return false; // A 的轴上有缝隙，绝对没撞
	}

	int edgeB = 0;
	float sepB = findMaxSeparation(edgeB, polyB, b, polyA, a);
	if (sepB > 0.0f) {
		return false; // B 的轴上有缝隙，绝对没撞
	}

	// -------------------------------------------------------------
	// 第二步：仲裁参考多边形 (Reference) 与 附着多边形 (Incident)
	// -------------------------------------------------------------
	const Polygon* refPoly;
	const Polygon* incPoly;
	const Body* refBody;
	const Body* incBody;
	int refIndex;
	bool flip; // 记录是否对调了身份，用于最后校正法线方向

	// 工业级容差偏置 (0.98 + 0.001)，防止在数值相近时频繁闪烁切换参考面
	if (sepB > 0.98f * sepA + 0.001f) {
		refPoly = polyB; refBody = b;
		incPoly = polyA; incBody = a;
		refIndex = edgeB;
		flip = true;
	}
	else {
		refPoly = polyA; refBody = a;
		incPoly = polyB; incBody = b;
		refIndex = edgeA;
		flip = false;
	}

	// -------------------------------------------------------------
	// 第三步：提取参考边 (Reference Edge) 及其世界裁剪几何平面
	// -------------------------------------------------------------
	int refCount = refPoly->getVertexCount();
	int r1 = refIndex;
	int r2 = (refIndex + 1) % refCount;

	float refRot = refBody->getRotation();
	Vector2 refPos = refBody->getPosition();

	// 参考边的两个世界端点
	Vector2 v1 = refPos + refPoly->getVertex(r1).rotate(refRot);
	Vector2 v2 = refPos + refPoly->getVertex(r2).rotate(refRot);

	Vector2 refTangent = (v2 - v1).normalize();
	Vector2 refNormal = refPoly->getNormal(refIndex).rotate(refRot);

	// -------------------------------------------------------------
	// 第四步：提取附着边并执行三道半平面裁剪 (Sutherland-Hodgman)
	// -------------------------------------------------------------
	ClipVertex incidentEdge[2];
	findIncidentEdge(incidentEdge, incPoly, incBody, refNormal);

	// 裁剪刀 1：参考边左侧面 (-refTangent 平面，过 v1)
	ClipVertex clip1[2];
	float offset1 = -refTangent.dot(v1);
	int np = clipSegmentToLine(clip1, incidentEdge, -refTangent, offset1);
	if (np < 2) return false;

	// 裁剪刀 2：参考边右侧面 (refTangent 平面，过 v2)
	ClipVertex clip2[2];
	float offset2 = refTangent.dot(v2);
	np = clipSegmentToLine(clip2, clip1, refTangent, offset2);
	if (np < 2) return false;

	// 裁剪刀 3：参考边正面投影测量 (过滤掉深度 <= 0 的非接触点)
	float frontOffset = refNormal.dot(v1);
	int contactCount = 0;
	float maxPenetration = 0.0f;
	Vector2 contacts[2];

	for (int i = 0; i < 2; ++i) {
		// 计算点到参考面正面的有向距离
		float dist = refNormal.dot(clip2[i].v) - frontOffset;
		if (dist <= 0.0f) {
			// dist <= 0 说明扎进了参考面内部
			float pen = -dist;
			maxPenetration = std::max(maxPenetration, pen);
			contacts[contactCount++] = clip2[i].v;
		}
	}

	if (contactCount == 0) {
		return false;
	}

	// -------------------------------------------------------------
	// 第五步：装配流形 Manifold (保证法线由 BodyA 指向 BodyB)
	// -------------------------------------------------------------
	m->bodyA = a;
	m->bodyB = b;
	// 如果是 B 作为参考面，refNormal 是由 B 指向 A 的，必须取反！
	m->normal = flip ? -refNormal : refNormal;
	m->penetration = maxPenetration;
	m->contactCount = contactCount;
	for (int i = 0; i < contactCount; ++i) {
		m->contacts[i] = contacts[i];
	}

	return true;
}

bool Collision::polygonVsCircle(Manifold* m, Body* polyBody, Body* circleBody)
{
	Circle* circle = static_cast<Circle*>(circleBody->getShape());
	Polygon* poly = static_cast<Polygon*>(polyBody->getShape());
	int count = poly->getVertexCount();

	// 1. 将圆心转换到多边形的局部坐标系 (平移 + 逆旋转)
	Vector2 localCirclePos = circleBody->getPosition() - polyBody->getPosition();
	Vector2 localCirclePosRotated = localCirclePos.rotate(-polyBody->getRotation());

	Vector2 closestV1, closestV2;
	Vector2 bestEdgeNormal;
	float maxSeparation = -FLT_MAX; // 寻找最大分离度 (最浅穿透)

	// 2. 利用 SAT 寻找离圆心最近的边
	for (int i = 0; i < count; ++i)
	{
		Vector2 v1 = poly->getVertex(i);
		Vector2 v2 = poly->getVertex((i + 1) % count);

		// 【修复】直接使用 set() 里预计算的外法线。
		// 之前用 getLeftNormal()：对逆时针多边形那是内向法线，
		// 圆心在边外侧时 dist > r 被误判为"未碰撞"，多边形接触永远返回 false
		Vector2 edge = v2 - v1;
		Vector2 edgeNormal = poly->getNormal(i);

		// 有向距离：绝不能加 abs！
		float dist = (localCirclePosRotated - v1).dot(edgeNormal);

		// 如果在某条轴上，圆心在边外侧且距离大于半径，说明绝对没撞上！
		if (dist > circle->getR()) {
			return false;
		}

		if (dist > maxSeparation)
		{
			maxSeparation = dist;
			closestV1 = v1;
			closestV2 = v2;
			bestEdgeNormal = edgeNormal;
		}
	}

	// 准备局部法线与局部接触点
	Vector2 localNormal;
	float penetration = 0.0f;
	Vector2 localContact;

	Vector2 edge = closestV2 - closestV1;

	// 3. 判断落在“面区域”还是“角区域”（沃罗诺伊区域判定）
	// --- 情况 A: 顶点 V1 角碰撞 ---
	if (edge.dot(localCirclePosRotated - closestV1) <= 0.0f)
	{
		Vector2 d = localCirclePosRotated - closestV1;
		float distSq = d.lengthSquared();
		if (distSq > circle->getR() * circle->getR()) {
			return false; // 离角点太远，未碰撞
		}

		float dist = std::sqrt(distSq);
		localNormal = (dist > 1e-6f) ? d * (1.0f / dist) : bestEdgeNormal;
		penetration = circle->getR() - dist;
		localContact = closestV1;
	}
	// --- 情况 B: 顶点 V2 角碰撞 ---
	else if (edge.dot(localCirclePosRotated - closestV2) >= 0.0f)
	{
		Vector2 d = localCirclePosRotated - closestV2;
		float distSq = d.lengthSquared();
		if (distSq > circle->getR() * circle->getR()) {
			return false; // 离角点太远，未碰撞
		}

		float dist = std::sqrt(distSq);
		localNormal = (dist > 1e-6f) ? d * (1.0f / dist) : bestEdgeNormal;
		penetration = circle->getR() - dist;
		localContact = closestV2;
	}
	// --- 情况 C: 平面接触 (纯面碰撞) ---
	else
	{
		localNormal = bestEdgeNormal;
		penetration = circle->getR() - maxSeparation;
		localContact = localCirclePosRotated - bestEdgeNormal * circle->getR();
	}

	// 4. 将法线与接触点正向旋转平移回【世界坐标系】
	float polyRot = polyBody->getRotation();
	Vector2 worldNormal = localNormal.rotate(polyRot); // 从 Polygon 指向 Circle
	Vector2 worldContact = polyBody->getPosition() + localContact.rotate(polyRot);

	// 5. 填满流形 Manifold
	m->bodyA = polyBody;
	m->bodyB = circleBody;
	m->normal = worldNormal;
	m->penetration = penetration;
	m->contacts[0] = worldContact;
	m->contactCount = 1;

	return true;
}

bool Collision::polygonVsCapsule(Manifold* m, Body* polyBody, Body* capsuleBody)
{
	Polygon* poly = static_cast<Polygon*>(polyBody->getShape());
	Capsule* cap = static_cast<Capsule*>(capsuleBody->getShape());

	float radius = cap->getRadius();
	int count = poly->getVertexCount();

	// -------------------------------------------------------------
	// 1. 获取胶囊体在世界坐标下的骨架两端点
	// -------------------------------------------------------------
	float halfLen = cap->getLength() * 0.5f;
	float capRot = capsuleBody->getRotation();
	// 局部 (0, 1) 旋转后的世界骨架方向
	Vector2 capAxis(-std::sin(capRot) * halfLen, std::cos(capRot) * halfLen);
	Vector2 worldA = capsuleBody->getPosition() - capAxis;
	Vector2 worldB = capsuleBody->getPosition() + capAxis;

	// -------------------------------------------------------------
	// 2. 将胶囊体端点变换到【多边形局部空间】(平移 + 逆旋转)
	// -------------------------------------------------------------
	float polyRot = polyBody->getRotation();
	Vector2 polyPos = polyBody->getPosition();
	float cosP = std::cos(-polyRot);
	float sinP = std::sin(-polyRot);

	auto WorldToPolyLocal = [&](const Vector2& w) -> Vector2 {
		Vector2 rel = w - polyPos;
		return Vector2(rel.x * cosP - rel.y * sinP, rel.x * sinP + rel.y * cosP);
		};

	Vector2 locA = WorldToPolyLocal(worldA);
	Vector2 locB = WorldToPolyLocal(worldB);

	// -------------------------------------------------------------
	// 3. SAT 分离轴检测：遍历多边形所有边
	// -------------------------------------------------------------
	int bestEdge = 0;
	float maxSeparation = -1e30f;

	for (int i = 0; i < count; ++i) {
		Vector2 n = poly->getNormal(i);
		Vector2 v = poly->getVertex(i);

		// 计算线段两个端点到当前边的有向距离
		float distA = (locA - v).dot(n);
		float distB = (locB - v).dot(n);

		// 胶囊体侵入该边最深的点
		float sep = std::min(distA, distB);

		// 如果在某条边上，最靠近的点距离都大于半径，说明存在分离轴，绝对没撞上！
		if (sep > radius) {
			return false;
		}

		if (sep > maxSeparation) {
			maxSeparation = sep;
			bestEdge = i;
		}
	}

	// -------------------------------------------------------------
	// 4. 提取最近边 (Reference Edge) 的几何特征
	// -------------------------------------------------------------
	Vector2 v1 = poly->getVertex(bestEdge);
	Vector2 v2 = poly->getVertex((bestEdge + 1) % count);
	Vector2 edgeDir = v2 - v1;
	float edgeLen = edgeDir.length();
	if (edgeLen < 1e-6f) return false;

	Vector2 edgeTangent = edgeDir * (1.0f / edgeLen);
	Vector2 edgeNormal = poly->getNormal(bestEdge);

	// 计算 A 和 B 沿着边切线方向相对于 v1 的投影标量
	float tA = (locA - v1).dot(edgeTangent);
	float tB = (locB - v1).dot(edgeTangent);

	// -------------------------------------------------------------
	// 5. 沃罗诺伊区域 (Voronoi) 判定：是角碰撞还是面碰撞？
	// -------------------------------------------------------------

	// --- 情况 A: 两端点都偏向 V1 侧面之外 (与角点 V1 碰撞) ---
	if (tA < 0.0f && tB < 0.0f) {
		Vector2 closestOnSeg = closestPointOnSegment(v1, locA, locB);
		Vector2 d = closestOnSeg - v1;
		float distSq = d.lengthSquared();
		if (distSq > radius * radius) return false;

		float dist = std::sqrt(distSq);
		Vector2 localNorm = (dist > 1e-6f) ? d * (1.0f / dist) : edgeNormal;

		m->bodyA = polyBody;
		m->bodyB = capsuleBody;
		m->normal = localNorm.rotate(polyRot);
		m->penetration = radius - dist;
		m->contacts[0] = polyPos + v1.rotate(polyRot);
		m->contactCount = 1;
		return true;
	}

	// --- 情况 B: 两端点都偏向 V2 侧面之外 (与角点 V2 碰撞) ---
	if (tA > edgeLen && tB > edgeLen) {
		Vector2 closestOnSeg = closestPointOnSegment(v2, locA, locB);
		Vector2 d = closestOnSeg - v2;
		float distSq = d.lengthSquared();
		if (distSq > radius * radius) return false;

		float dist = std::sqrt(distSq);
		Vector2 localNorm = (dist > 1e-6f) ? d * (1.0f / dist) : edgeNormal;

		m->bodyA = polyBody;
		m->bodyB = capsuleBody;
		m->normal = localNorm.rotate(polyRot);
		m->penetration = radius - dist;
		m->contacts[0] = polyPos + v2.rotate(polyRot);
		m->contactCount = 1;
		return true;
	}

	// --- 情况 C: 面碰撞 (线段跨越在 [0, edgeLen] 内部) ---
	// 对胶囊体骨架线段进行区间裁剪，截取落在 [0, edgeLen] 范围内的部分
	float u1 = 0.0f, u2 = 1.0f;
	float denom = tB - tA;

	if (std::abs(denom) > 1e-6f) {
		float s0 = (0.0f - tA) / denom;
		float s1 = (edgeLen - tA) / denom;
		float sMin = std::min(s0, s1);
		float sMax = std::max(s0, s1);

		u1 = std::max(u1, sMin);
		u2 = std::min(u2, sMax);
	}

	// 得到裁剪后的有效线段候选点 (最多 2 个)
	Vector2 clipPoints[2];
	clipPoints[0] = locA + (locB - locA) * u1;
	clipPoints[1] = locA + (locB - locA) * u2;

	int contactCount = 0;
	float maxPen = 0.0f;
	Vector2 worldContacts[2];

	for (int i = 0; i < 2; ++i) {
		Vector2 pt = clipPoints[i];
		// 计算候选点到边的垂直距离
		float dist = (pt - v1).dot(edgeNormal);
		if (dist <= radius) {
			float pen = radius - dist;
			maxPen = std::max(maxPen, pen);

			// 局部接触点取多边形边表面
			Vector2 localContact = pt - edgeNormal * dist;
			worldContacts[contactCount++] = polyPos + localContact.rotate(polyRot);

			// 如果两个裁剪点几乎重叠（两端极短），只保留 1 个点
			if (i == 0 && (clipPoints[1] - clipPoints[0]).lengthSquared() < 1e-6f) {
				break;
			}
		}
	}

	if (contactCount == 0) {
		return false;
	}

	// -------------------------------------------------------------
	// 6. 装配流形 Manifold (世界坐标)
	// -------------------------------------------------------------
	m->bodyA = polyBody;
	m->bodyB = capsuleBody;
	m->normal = edgeNormal.rotate(polyRot); // 从 Polygon 指向 Capsule
	m->penetration = maxPen;
	m->contactCount = contactCount;
	for (int i = 0; i < contactCount; ++i) {
		m->contacts[i] = worldContacts[i];
	}

	return true;
}

bool Collision::circleVsBox(Manifold* m, Body* circlebody, Body* boxbody)
{
	Circle* circle = static_cast<Circle*>(circlebody->getShape());
	Box* box = static_cast<Box*>(boxbody->getShape());

	// 1. 世界坐标转 Box 局部坐标
	Vector2 relationPos = circlebody->getPosition() - boxbody->getPosition();
	Vector2 localCirclePos = relationPos.rotate(-boxbody->getRotation());

	float hw = box->getHalfWidth();
	float hh = box->getHalfHeight();

	// 2. 确定矩形上距离圆心最近的点 (Closest Point)
	// C++11 替代 std::clamp 的写法：
	float closestX = std::max(-hw, std::min(localCirclePos.getX(), hw));
	float closestY = std::max(-hh, std::min(localCirclePos.getY(), hh));
	Vector2 closestPoint(closestX, closestY);

	// 3. 计算距离
	Vector2 distVec = localCirclePos - closestPoint; // 从最近点指向圆心
	float distSq = distVec.lengthSquared();
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
	m->normal = localNormal.rotate(boxbody->getRotation());
	m->contacts[0] = closestPoint.rotate(boxbody->getRotation()) + boxbody->getPosition();

	// 5. 初始化冲量缓存（Day 12 核心）
	m->impulseN[0] = 0.0f;
	m->impulseT[0] = 0.0f;

	return true;
}

bool Collision::aabbVsAabb(const AABB& a, const AABB& b) {
	// 逻辑：如果 A 在 B 的右边、左边、上边或下边，则一定没撞
	if (a.max.getX() < b.min.getX() || a.min.getX() > b.max.getX()) return false;
	if (a.max.getY() < b.min.getY() || a.min.getY() > b.max.getY()) return false;

	// 否则，说明重叠了
	return true;
}

bool Collision::dispatch(Manifold* m, Body* a, Body* b) {
	Shape::Type typeA = a->getShape()->type;
	Shape::Type typeB = b->getShape()->type;

	// 1. 同类型碰撞
	if (typeA == Shape::Type::type_Circle && typeB == Shape::Type::type_Circle) return Collision::circleVsCircle(m, a, b);
	if (typeA == Shape::Type::type_Box && typeB == Shape::Type::type_Box) return Collision::boxVsBox(m, a, b);
	if (typeA == Shape::Type::type_Capsule && typeB == Shape::Type::type_Capsule) return Collision::capsuleVsCapsule(m, a, b);
	if (typeA == Shape::Type::type_Polygon && typeB == Shape::Type::type_Polygon) return Collision::polygonVsPolygon(m, a, b);

	// 2. 混合类型碰撞 (Circle vs Box)
	if (typeA == Shape::Type::type_Circle && typeB == Shape::Type::type_Box) {
		return Collision::circleVsBox(m, a, b);
	}

	// 3. 混合类型碰撞 (Box vs Circle) -> 巧妙交换参数
	if (typeA == Shape::Type::type_Box && typeB == Shape::Type::type_Circle) {
		// circleVsBox 内部约定 A=circle, B=box, 法线方向 A->B；
		// 这里把 bodyA/bodyB 交换回与 Contact 一致的排序，同时翻转法线保持 A->B 约定
		// 【修复】之前只翻法线不换 body，导致流形法线与 bodyA->bodyB 方向相反，
		// 混合类型碰撞的法向冲量为负被 clamp 成 0，位置修正方向也反了（球会陷进地面）
		bool hit = Collision::circleVsBox(m, b, a);
		if (hit) {
			Body* tmp = m->bodyA;
			m->bodyA = m->bodyB;
			m->bodyB = tmp;
			m->normal = m->normal * -1.0f;
		}
		return hit;
	}

	// 4. 混合类型碰撞 (Capsule vs Circle) —— capsuleVsCircle 内部约定 A=capsule, B=circle, 法线 A->B
	if (typeA == Shape::Type::type_Capsule && typeB == Shape::Type::type_Circle) {
		return Collision::capsuleVsCircle(m, a, b);
	}
	if (typeA == Shape::Type::type_Circle && typeB == Shape::Type::type_Capsule) {
		// 排序是 (circle, capsule)，交换 body 并翻转法线，保持与 Contact 排序一致
		bool hit = Collision::capsuleVsCircle(m, b, a);
		if (hit) {
			Body* tmp = m->bodyA;
			m->bodyA = m->bodyB;
			m->bodyB = tmp;
			m->normal = m->normal * -1.0f;
		}
		return hit;
	}

	// 5. 混合类型碰撞 (Capsule vs Box) —— capsuleVsBox 内部约定 A=capsule, B=box, 法线 A->B
	if (typeA == Shape::Type::type_Capsule && typeB == Shape::Type::type_Box) {
		return Collision::capsuleVsBox(m, a, b);
	}
	if (typeA == Shape::Type::type_Box && typeB == Shape::Type::type_Capsule) {
		bool hit = Collision::capsuleVsBox(m, b, a);
		if (hit) {
			Body* tmp = m->bodyA;
			m->bodyA = m->bodyB;
			m->bodyB = tmp;
			m->normal = m->normal * -1.0f;
		}
		return hit;
	}

	// 6. 混合类型碰撞 (Polygon vs Circle) —— polygonVsCircle 内部约定 A=polygon, B=circle, 法线 A->B
	if (typeA == Shape::Type::type_Polygon && typeB == Shape::Type::type_Circle) {
		return Collision::polygonVsCircle(m, a, b);
	}
	if (typeA == Shape::Type::type_Circle && typeB == Shape::Type::type_Polygon) {
		bool hit = Collision::polygonVsCircle(m, b, a);
		if (hit) {
			Body* tmp = m->bodyA;
			m->bodyA = m->bodyB;
			m->bodyB = tmp;
			m->normal = m->normal * -1.0f;
		}
		return hit;
	}

	// 7. 混合类型碰撞 (Polygon vs Capsule) —— polygonVsCapsule 内部约定 A=polygon, B=capsule, 法线 A->B
	if (typeA == Shape::Type::type_Polygon && typeB == Shape::Type::type_Capsule) {
		return Collision::polygonVsCapsule(m, a, b);
	}
	if (typeA == Shape::Type::type_Capsule && typeB == Shape::Type::type_Polygon) {
		bool hit = Collision::polygonVsCapsule(m, b, a);
		if (hit) {
			Body* tmp = m->bodyA;
			m->bodyA = m->bodyB;
			m->bodyB = tmp;
			m->normal = m->normal * -1.0f;
		}
		return hit;
	}

	return false;
}
float Collision::findMaxSeparation(int& edgeIndex, const Polygon* polyA, const Body* bodyA, const Polygon* polyB, const Body* bodyB)
{
	int countA = polyA->getVertexCount();
	int countB = polyB->getVertexCount();

	// 1. 预先把多边形 B 的所有顶点转换到【世界坐标系】
	Vector2 worldVertsB[Polygon::MAX_VERTICES];
	float cosB = std::cos(bodyB->getRotation());
	float sinB = std::sin(bodyB->getRotation());
	Vector2 posB = bodyB->getPosition();

	for (int i = 0; i < countB; ++i) {
		Vector2 v = polyB->getVertex(i);
		worldVertsB[i] = Vector2(
			posB.getX() + (v.getX() * cosB - v.getY() * sinB),
			posB.getY() + (v.getX() * sinB + v.getY() * cosB)
		);
	}
	// 准备多边形 A 的旋转平移参数
	float cosA = std::cos(bodyA->getRotation());
	float sinA = std::sin(bodyA->getRotation());
	Vector2 posA = bodyA->getPosition();

	float maxSeparation = -FLT_MAX;
	int bestEdge = 0;

	// 3. 遍历 A 的每一条边的法线 (SAT 候选轴)
	for (int i = 0; i < countA; ++i)
	{
		// 【修复】SAT 轴必须是"边的外法线"，用预计算好的 getNormal(i)；
		// 之前把顶点方向 normalize 当轴用，方向全错（矩形会得到 45 度斜轴）
		Vector2 localN = polyA->getNormal(i);
		Vector2 normalA(
			localN.getX() * cosA - localN.getY() * sinA,
			localN.getX() * sinA + localN.getY() * cosA
		);
		// 获取边 i 的世界顶点 (端点作为平面基准点)
		Vector2 localV = polyA->getVertex(i);
		Vector2 vertexA(
			posA.getX() + (localV.getX() * cosA - localV.getY() * sinA),
			posA.getY() + (localV.getX() * sinA + localV.getY() * cosA)
		);

		// 4. 在该法线上，寻找多边形 B 投影距离最小的点 (即扎入 A 最深的点)
		float minSeparationForThisEdge = 1e30f;

		for (int j = 0; j < countB; ++j) {
			// 有向距离: (B_点 - A_面基准点) · 法线
			float separation = (worldVertsB[j] - vertexA).dot( normalA);
			if (separation < minSeparationForThisEdge) {
				minSeparationForThisEdge = separation;
			}
		}

		// 5. 在所有的候选轴中，保留“分离度最大”的轴 (即穿透最浅的轴)
		if (minSeparationForThisEdge > maxSeparation) {
			maxSeparation = minSeparationForThisEdge;
			bestEdge = i;
		}
	}
	edgeIndex = bestEdge;
	return maxSeparation;
}
int Collision::clipSegmentToLine(ClipVertex vOut[2], const ClipVertex vIn[2], const Vector2& normal, float offset)
{
	int numOut = 0;
	float d0 = normal.dot(vIn[0].v) - offset;
	float d1 = normal.dot(vIn[1].v) - offset;

	// 如果端点在直线的内侧 (有向距离 <= 0)
	if (d0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (d1 <= 0.0f) vOut[numOut++] = vIn[1];

	// 如果两个端点分布在直线两侧，求截断交点
	if (d0 * d1 < 0.0f) {
		float alpha = d0 / (d0 - d1);
		vOut[numOut].v = vIn[0].v + (vIn[1].v - vIn[0].v) * alpha;
		numOut++;
	}
	return numOut;
}
std::vector<Vector2> Collision::getBoxWorldVertices(const Body* body)
{
	Box* box = static_cast<Box*>(body->getShape());//获取box
	//获取四个坐标
	//获取原点+/width2+height/2,旋转角
	Vector2 pos = body->getPosition();
	float halfwidth= box->getHalfWidth();
	float halfheight = box->getHalfHeight();
	float rotation =body->getRotation();
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
	worldVertices.push_back(pos + leftUp.rotate(rotation));
	worldVertices.push_back(pos + rightUp.rotate(rotation));
	worldVertices.push_back(pos + leftDown.rotate(rotation));
	worldVertices.push_back(pos + rightDown.rotate(rotation));
	return worldVertices;
}

Projection Collision::getProjection(const std::vector<Vector2>& vertices, const Vector2& axis)
{
	float dot =vertices[0].dot(axis);
	float min = dot;
	float max = dot;
	for (int i = 1;i < vertices.size();++i)
	{
		dot=vertices[i].dot(axis);
		min = (min < dot) ? min : dot;
		max = (max > dot) ? max : dot;
	}
	return { min,max };
}

float Collision::getOverlap(float minA, float maxA, float minB, float maxB)
{
	// 逻辑：是否有缝隙？
	if (maxA < minB || maxB < minA) {
		return -1.0f; // 返回负值表示没有重叠（即存在分离轴）
	}
	// 计算重叠部分的长度
	// 公式：两个 max 中的较小值 - 两个 min 中的较大值
	return std::min(maxA, maxB) - std::max(minA, minB);
}

// 2. 计算线段 [A, B] 与线段 [C, D] 之间的最近点对 (Christer Ericson 经典算法)
void Collision::closestPointsBetweenSegments(const Vector2& a, const Vector2& b,
	const Vector2& c, const Vector2& d,
	Vector2& outP1, Vector2& outP2) {
	Vector2 d1 = b - a; // 线段 1 方向
	Vector2 d2 = d - c; // 线段 2 方向
	Vector2 r = a - c;
	float s_a = d1.dot(d1);
	float s_e = d2.dot(d2);
	float f = d2.dot(r);

	float s = 0.0f, t = 0.0f;

	if (s_a <= 1e-6f && s_e <= 1e-6f) {
		outP1 = a; outP2 = c;
		return;
	}

	if (s_a <= 1e-6f) {
		s = 0.0f;
		t = std::max(0.0f, std::min(1.0f, f / s_e));
	}
	else {
		float c_dot = d1.dot(r);
		if (s_e <= 1e-6f) {
			t = 0.0f;
			s = std::max(0.0f, std::min(1.0f, -c_dot / s_a));
		}
		else {
			float b_dot = d1.dot(d2);
			float denom = s_a * s_e - b_dot * b_dot;

			if (denom != 0.0f) {
				s = std::max(0.0f, std::min(1.0f, (b_dot * f - c_dot * s_e) / denom));
			}
			else {
				s = 0.0f; // 两线段平行
			}

			t = (b_dot * s + f) / s_e;
			if (t < 0.0f) {
				t = 0.0f;
				s = std::max(0.0f, std::min(1.0f, -c_dot / s_a));
			}
			else if (t > 1.0f) {
				t = 1.0f;
				s = std::max(0.0f, std::min(1.0f, (b_dot - c_dot) / s_a));
			}
		}
	}
	outP1 = a.add(d1 * s);
	outP2 = c.add(d2 * t);
}

// 辅助函数：获取胶囊体在世界坐标下的骨架两端点
void Collision::getCapsuleWorldSegment(Body* body, Capsule* cap, Vector2& outA, Vector2& outB) {
	float halfLen = cap->getLength() * 0.5f;
	float sinA = std::sin(body->getRotation());
	float cosA = std::cos(body->getRotation());
	Vector2 axis(-sinA * halfLen, cosA * halfLen); // 局部 (0, 1) 旋转后的世界方向
	outA = body->getPosition() - axis;
	outB = body->getPosition() + axis;
}

bool Collision::capsuleVsCircle(Manifold* m, Body* capsuleBody, Body* circleBody)
{
	Capsule* cap = static_cast<Capsule*>(capsuleBody->getShape());
	Circle* circ = static_cast<Circle*>(circleBody->getShape());

	// 1. 获取胶囊体世界骨架线段
	Vector2 segA, segB;
	getCapsuleWorldSegment(capsuleBody, cap, segA, segB);

	// 2. 求圆心到线段的最近点 P
	Vector2 closestP = closestPointOnSegment(circleBody->getPosition(), segA, segB);

	// 3. 计算两圆心连线向量与距离
	Vector2 d = circleBody->getPosition() - closestP;
	float distSq = d.dot(d);
	float radiusSum = cap->getRadius() + circ->getR();

	if (distSq > radiusSum * radiusSum) {
		return false; // 未发生碰撞
	}

	float dist = std::sqrt(distSq);

	m->bodyA = capsuleBody;
	m->bodyB = circleBody;

	if (dist > 1e-6f) {
		m->normal = d * (1.0f / dist); // 法线从 Capsule 指向 Circle
		m->penetration = radiusSum - dist;
	}
	else {
		// 圆心正好落在骨架线段上的极端重叠保护
		float sinA = std::sin(capsuleBody->getRotation());
		float cosA = std::cos(capsuleBody->getRotation());
		m->normal = Vector2(cosA, sinA); // 取骨架法向
		m->penetration = radiusSum;
	}

	// 接触点取重叠区域中间
	m->contacts[0] = closestP + m->normal * cap->getRadius();
	m->contactCount = 1;

	return true;
}

bool Collision::capsuleVsCapsule(Manifold* m, Body* bodyA, Body* bodyB)
{
	Capsule* capA = static_cast<Capsule*>(bodyA->getShape());
	Capsule* capB = static_cast<Capsule*>(bodyB->getShape());

	Vector2 a1, b1, a2, b2;
	getCapsuleWorldSegment(bodyA, capA, a1, b1);
	getCapsuleWorldSegment(bodyB, capB, a2, b2);

	// 1. 求解两线段最近点对
	Vector2 pA, pB;
	closestPointsBetweenSegments(a1, b1, a2, b2, pA, pB);

	// 2. 检测两点距离
	Vector2 d = pB - pA;
	float distSq = d.dot(d);
	float radiusSum = capA->getRadius() + capB->getRadius();

	if (distSq > radiusSum * radiusSum) {
		return false;
	}

	float dist = std::sqrt(distSq);

	m->bodyA = bodyA;
	m->bodyB = bodyB;

	if (dist > 1e-6f) {
		m->normal = d * (1.0f / dist); // 从 BodyA 指向 BodyB
		m->penetration = radiusSum - dist;
	}
	else {
		// 两线段完全相交重叠的退化防御
		Vector2 relPos = bodyB->getPosition() - bodyA->getPosition();
		m->normal = (relPos.dot(relPos) > 1e-6f) ? relPos.normalize() : Vector2(0.0f, 1.0f);
		m->penetration = radiusSum;
	}

	// 接触点取交界面
	m->contacts[0] = pA + m->normal * (capA->getRadius() - m->penetration * 0.5f);
	m->contactCount = 1;

	return true;
}

bool Collision::capsuleVsBox(Manifold* m, Body* capsuleBody, Body* boxBody)
{
	Capsule* cap = static_cast<Capsule*>(capsuleBody->getShape());
	Box* box = static_cast<Box*>(boxBody->getShape());

	float hx = box->getHalfWidth();
	float hy = box->getHalfHeight();

	// 1. 获取胶囊体世界线段
	Vector2 worldSegA, worldSegB;
	getCapsuleWorldSegment(capsuleBody, cap, worldSegA, worldSegB);

	// 2. 将胶囊体线段转换到 Box 局部空间（逆旋转平移）
	float cosB = std::cos(-boxBody->getRotation());
	float sinB = std::sin(-boxBody->getRotation());
	auto WorldToBoxLocal = [&](const Vector2& w) -> Vector2 {
		Vector2 rel = w - boxBody->getPosition();
		return Vector2(rel.getX() * cosB - rel.getY() * sinB, rel.getX() * sinB + rel.getY() * cosB);
		};

	Vector2 locA = WorldToBoxLocal(worldSegA);
	Vector2 locB = WorldToBoxLocal(worldSegB);

	// 3. 收集 6 个潜在最近点候选
	Vector2 testPoints[6];
	testPoints[0] = locA;
	testPoints[1] = locB;
	testPoints[2] = closestPointOnSegment(Vector2(-hx, -hy), locA, locB);
	testPoints[3] = closestPointOnSegment(Vector2(hx, -hy), locA, locB);
	testPoints[4] = closestPointOnSegment(Vector2(hx, hy), locA, locB);
	testPoints[5] = closestPointOnSegment(Vector2(-hx, hy), locA, locB);

	// 4. 扫描 6 个候选点，同时统计两类信息：
	//    - 盒外点：到盒面的最近距离（浅接触用）
	//    - 盒内点：轴刺入盒内的候选点集合（clamp 后距离为 0）
	// 【修复】旧实现把"轴点刺入盒内"（距离 0）误当"恰好碰面"：
	//   0/0 产生 NaN 法线，且穿透算成 0 导致胶囊直接穿盒
	float minClampedDistSq = 1e30f;
	float maxPushDepth_ = -1e30f;            // 整段深陷时的最近面深度
	Vector2 bestSegPoint, bestBoxPoint;      // 最近的盒外点对
	Vector2 pushNormal(0.0f, 1.0f);          // 整段深陷时的最近面推出法线
	Vector2 deepestSegPoint(0.0f, 0.0f);     // 盒内最深点
	Vector2 insidePoints[6];
	int insideCount = 0;

	for (int i = 0; i < 6; ++i) {
		Vector2 s = testPoints[i];
		// 将点 Clamp 到 Box 边界
		Vector2 b(std::max(-hx, std::min(hx, s.getX())),
			std::max(-hy, std::min(hy, s.getY())));

		Vector2 diff = s - b;
		float dSq = diff.dot(diff);

		if (dSq > 1e-6f) {
			// 盒外点：跟踪最小表面距离
			if (dSq < minClampedDistSq) {
				minClampedDistSq = dSq;
				bestSegPoint = s;
				bestBoxPoint = b;
			}
		}
		else {
			// 盒内点：收集，并跟踪"最近面推出"信息（整段深陷时用）
			insidePoints[insideCount++] = s;
			float dx = hx - std::abs(s.getX());
			float dy = hy - std::abs(s.getY());
			if (dx < dy) {
				if (dx > maxPushDepth_) {
					maxPushDepth_ = dx;
					pushNormal = Vector2(s.getX() >= 0.0f ? 1.0f : -1.0f, 0.0f);
					deepestSegPoint = s;
				}
			}
			else {
				if (dy > maxPushDepth_) {
					maxPushDepth_ = dy;
					pushNormal = Vector2(0.0f, s.getY() >= 0.0f ? 1.0f : -1.0f);
					deepestSegPoint = s;
				}
			}
		}
	}

	float capRadius = cap->getRadius();
	Vector2 localNormal;
	float penetration = 0.0f;
	Vector2 localContactPoint;

	if (insideCount > 0 && minClampedDistSq < 1e30f) {
		// --- 轴刺入盒内、且仍有盒外点：入口面由最近的盒外点决定 ---
		// 法线约定与浅接触分支一致：从轴点指向盒面（指向盒内），
		// 下游 dispatch 的 swap+flip 会把它变成"推出"方向
		Vector2 d = bestBoxPoint - bestSegPoint;
		float len = d.length();
		localNormal = (len > 1e-6f) ? d * (1.0f / len) : pushNormal;

		// 穿透 = 半径 + 最深轴点沿法线越过入口面的深度
		float depth = 0.0f;
		for (int i = 0; i < insideCount; ++i) {
			depth = std::max(depth, localNormal.dot(insidePoints[i] - bestBoxPoint));
		}
		penetration = capRadius + depth;
		localContactPoint = bestBoxPoint;
	}
	else if (insideCount > 0) {
		// --- 整段深陷盒内：沿用旧深穿透公式（按最近面推出）---
		float overlapX = (hx + capRadius) - std::abs(deepestSegPoint.getX());
		float overlapY = (hy + capRadius) - std::abs(deepestSegPoint.getY());
		if (overlapX < overlapY) {
			localNormal = Vector2(deepestSegPoint.getX() > 0 ? 1.0f : -1.0f, 0.0f);
			penetration = overlapX;
		}
		else {
			localNormal = Vector2(0.0f, deepestSegPoint.getY() > 0 ? 1.0f : -1.0f);
			penetration = overlapY;
		}
		localContactPoint = deepestSegPoint;
	}
	else {
		// --- 轴完全在盒外：常规距离法线 ---
		float dist = std::sqrt(minClampedDistSq);
		if (dist > capRadius) {
			return false; // 没有碰上
		}
		Vector2 d = bestBoxPoint - bestSegPoint;
		localNormal = d * (1.0f / dist);
		penetration = capRadius - dist;
		localContactPoint = bestBoxPoint;
	}

	// 5. 将法线与接触点正向旋转回世界坐标
	float worldCos = std::cos(boxBody->getRotation());
	float worldSin = std::sin(boxBody->getRotation());
	auto BoxLocalToWorldVec = [&](const Vector2& v) -> Vector2 {
		return Vector2(v.getX() * worldCos - v.getY() * worldSin, v.getX() * worldSin + v.getY() * worldCos);
		};

	m->bodyA = capsuleBody;
	m->bodyB = boxBody;
	m->normal = BoxLocalToWorldVec(localNormal); // 从 Capsule 指向 Box
	m->penetration = penetration;
	m->contacts[0] = boxBody->getPosition() + BoxLocalToWorldVec(localContactPoint);
	m->contactCount = 1;

	return true;
}