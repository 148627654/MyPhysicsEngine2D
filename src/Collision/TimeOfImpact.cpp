#include "TimeOfImpact.h"
#include "../../include/physics/Common/Setting.h"
#include <Logger.h>
TOIOutput TimeOfImpact::Solve(const TOIInput& input)
{
	//printf("--- DEBUG: TOI Solve Started ---\n");
	TOIOutput output;
	output.alpha = 1.0f;
	output.state = TOIOutput::Separated;

	float alpha = 0.0f;
	const float targetDistance = input.tolerance; // 比如 0.005
	const float tolerance = 0.001f; // 迭代容差

	for (int iter = 0; iter < input.maxIterations; ++iter)
	{
		//printf("--- iter %d---\n", iter);
		// 1. 获取当前 alpha 时的位姿并计算距离
		Vector2 normal;
		float distance = CalculateDistance(input, alpha, normal);
		// 使用 std::endl 强制刷新缓冲区，或者直接用 printf
		//printf("--- distance %f---\n",distance);
		fflush(stdout);
		// --- 2. 初始重叠检查 ---
		if (iter == 0) {
			if (distance <= Settings::LINEAR_SLOP) { // 如果初始距离小于一个极小值
				output.state = TOIOutput::Overlapped;
				output.alpha = 0.0f;
				return output;
			}
		}

		// --- 3. 判定是否达到撞击条件 ---
		// 如果距离已经在 targetDistance 的极小误差范围内，直接返回成功
		if (std::abs(distance - targetDistance) < tolerance) {
			output.state = TOIOutput::Hit;
			output.alpha = alpha;
			return output;
		}

		// --- 4. 计算接近速度 ---
		Vector2 vA = input.bodyA->GetVelocity();
		Vector2 vB = input.bodyB->GetVelocity();
		// 如果考虑旋转，使用保守估计（增加步进安全性）
		float maxAngularVel = std::abs(input.bodyA->getAngularVelocity()) * input.bodyA->GetShape()->GetSweepRadius() +
			std::abs(input.bodyB->getAngularVelocity()) * input.bodyB->GetShape()->GetSweepRadius();

		float linearComponent = (vA - vB).Dot(normal);
		float closingSpeed = linearComponent + maxAngularVel;

		// --- 5. 步进 Alpha ---
		if (closingSpeed <= 0.0f) {
			output.state = TOIOutput::Separated;
			output.alpha = 1.0f;
			return output;
		}

		// 核心公式：我们想要找到距离刚好等于 targetDistance 的时间点
		float alpha_step = (distance - targetDistance) / (closingSpeed * input.dt);

		// 关键修复：如果 alpha_step 非常小，说明我们已经非常接近撞击点了
		if (alpha_step < 0.0001f && iter > 0) {
			output.state = TOIOutput::Hit;
			output.alpha = alpha;
			return output;
		}

		alpha += alpha_step;

		// --- 6. 越界检查 ---
		if (alpha > 1.0f || alpha < 0.0f) {
			output.state = TOIOutput::Separated;
			output.alpha = 1.0f;
			return output;
		}
	}

	// 如果迭代次数用完，通常是因为 alpha_step 在 target 附近震荡
	output.state = TOIOutput::Hit;
	output.alpha = alpha;
	return output;
}

float TimeOfImpact::CalculateDistance(const TOIInput& input, float alpha, Vector2& outNormal) {
	//printf("Checking Alpha: %f\n", alpha);
	// 1. 获取 alpha 时刻的位姿
	Transform xfA = input.bodyA->GetTransform(alpha, input.dt);
	Transform xfB = input.bodyB->GetTransform(alpha, input.dt);

	Shape* shapeA = input.bodyA->GetShape();
	Shape* shapeB = input.bodyB->GetShape();
	//printf("Calculating distance between type %d and %d\n", shapeA->type, shapeB->type);
	// 2. 根据类型进行分发
	if (shapeA->type == Shape::Type::type_Circle && shapeB->type == Shape::Type::type_Circle) {
		return CircleVsCircleDistance((Circle*)shapeA, xfA, (Circle*)shapeB, xfB, outNormal);
	}

	if (shapeA->type == Shape::Type::type_Box && shapeB->type == Shape::Type::type_Circle) {
		return BoxVsCircleDistance((Box*)shapeA, xfA, (Circle*)shapeB, xfB, outNormal);
	}

	if (shapeA->type == Shape::Type::type_Circle && shapeB->type == Shape::Type::type_Box) {
		// 交换顺序，注意法线反向
		float d = BoxVsCircleDistance((Box*)shapeB, xfB, (Circle*)shapeA, xfA, outNormal);
		outNormal = -outNormal;
		return d;
	}

	if (shapeA->type == Shape::Type::type_Box && shapeB->type == Shape::Type::type_Box) {
		return BoxVsBoxDistance((Box*)shapeA, xfA, (Box*)shapeB, xfB, outNormal);
	}

	return 1e10f; // 未知类型
}

float TimeOfImpact::CircleVsCircleDistance(Circle* a, const Transform& xfA, Circle* b, const Transform& xfB, Vector2& outNormal)
{
	Vector2 d = xfB.p - xfA.p;
	float dist = d.Length();
	if (dist > Settings::EPSILON) {
		outNormal = d / dist;
	}
	else {
		outNormal = Vector2(1, 0);
	}
	return dist - (a->getR() + b->getR());
}

float TimeOfImpact::BoxVsCircleDistance(Box* box, const Transform& xfBox, Circle* circle, const Transform& xfCircle, Vector2& outNormal) {
	// 1. 将圆心转到 Box 的局部坐标系
	float c = cosf(-xfBox.q);
	float s = sinf(-xfBox.q);
	Vector2 relPos = xfCircle.p - xfBox.p;
	Vector2 localCenter(relPos.getX() * c - relPos.getY() * s,
		relPos.getX() * s + relPos.getY() * c);

	// 2. 找到局部最近点
	float hx = box->getWidth() * 0.5f;
	float hy = box->getHeigh() * 0.5f;
	Vector2 closest(std::max(-hx, std::min(localCenter.getX(), hx)),
		std::max(-hy, std::min(localCenter.getY(), hy)));

	Vector2 distVec = localCenter - closest;
	float d = distVec.Length();

	if (d > Settings::EPSILON) {
		// 转回世界法线
		Vector2 localNormal = distVec / d;
		float wc = cosf(xfBox.q), ws = sinf(xfBox.q);
		outNormal.setX(localNormal.getX() * wc - localNormal.getY() * ws);
		outNormal.setY(localNormal.getX() * ws + localNormal.getY() * wc);
	}
	else {
		outNormal = (xfCircle.p - xfBox.p).Normalize();
	}
	return d - circle->getR();
}

float TimeOfImpact::BoxVsBoxDistance(Box* a, const Transform& xfA, Box* b, const Transform& xfB, Vector2& outNormal) {
	Vector2 d = xfB.p - xfA.p;
	float centerDist = d.Length();

	// 安全归一化：防止中心重合导致崩溃
	if (centerDist > Settings::EPSILON) {
		outNormal = d / centerDist;
	}
	else {
		outNormal = Vector2(1, 0); // 默认法线
	}

	// 局部辅助函数：计算 Box 在法线 n 上的投影半径
	auto getProj = [](Box* box, float angle, const Vector2& n) {
		float hx = box->getWidth() * 0.5f;
		float hy = box->getHeigh() * 0.5f;

		// 计算世界坐标系下的局部轴
		Vector2 ax(cosf(angle), sinf(angle));
		Vector2 ay(-sinf(angle), cosf(angle));

		// 半径 = 投影长度之和
		return hx * std::abs(ax.Dot(n)) + hy * std::abs(ay.Dot(n));
		};

	float projA = getProj(a, xfA.q, outNormal);
	float projB = getProj(b, xfB.q, outNormal);

	// 间隙 = 中心距离 - 两个半径之和
	return centerDist - (projA + projB);
}
