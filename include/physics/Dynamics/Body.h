#pragma once // 记得加上这个，防止重复包含
#include "../Common/Vector2.h"
class Body
{
public:
	Body(Vector2 pos = Vector2(0, 0), float m = 1.0f, float gScale = 1.0f)
		:position(pos),mass(m),gravityScale(gScale){
		velocity = Vector2(0, 0);
		force = Vector2(0, 0);

		// 静态物体约定：质量为 0 时，invMass = 0
		if (mass > 0.0f) {
			invMass = 1.0f / mass;
		}
		else {
			invMass = 0.0f;
		}
	}
	inline void ClearForce() { force.Clear(); }
	Vector2 AddForce(Vector2 f);
	friend class World;
	// 方便外部（如日志系统）读取数据
	Vector2 GetPosition() const { return position; }
	Vector2 GetVelocity() const { return velocity; }
	void SetVelocity(Vector2 v) { velocity = v; }
private:
	Vector2 position;		//当前位置
	Vector2 velocity;		//当前速度
	Vector2 force;			//累积力（每一帧更新前清零）

	float mass;				//质量
	float invMass;			//质量的倒数
	float gravityScale;		//重力的缩放系数

};