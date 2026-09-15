#pragma once
#include "Shape.h"
#include <vector>
#include "../Common/Vector2.h"

class Polygon :public Shape
{
public:
	static constexpr int MAX_VERTICES = 8; // 工业界通常限制凸多边形顶点数 <= 8 以换取极致缓存与计算性能
	Polygon(const Physics2D::Material& mat = Physics2D::Material())
		:Shape(Type::type_Polygon, mat),m_count(0) {}

	bool Set(Vector2* vertices, int count);
	float GetSweepRadius() const override;
	// 供测试/外部读取内部顶点（已做质心归零平移）
	inline int GetVertexCount() const { return m_count; }
	inline Vector2 GetVertex(int i) const { return m_vertices[i]; }

	MassData ComputeMass(float density);
	float getArea() const;
	AABB ComputeAABB(Vector2 pos, float angle);
	bool RayCast(RayCastOutput* output, RayCastInput& input,
		const Vector2& position, float rotation);
private:
	Vector2 m_vertices[MAX_VERTICES];
	Vector2 m_normals[MAX_VERTICES];
	int m_count;
	float m_radius = 0.0f; // 扫掠半径 (用于宽相与CCD)
	float m_area = 0.0f;   // 缓存解析面积
};