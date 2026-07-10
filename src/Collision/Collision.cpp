#include "Collision.h"

#include "../Dynamics/Body.h"
#include "../Collision/Circle.h"

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
