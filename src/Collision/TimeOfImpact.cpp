#include "TimeOfImpact.h"
#include "../../include/physics/Common/Setting.h"
#include <Logger.h>
TOIOutput TimeOfImpact::Solve(const TOIInput& input) {
	TOIOutput output;
	output.alpha = 1.0f;
	output.state = TOIOutput::Separated;
	output.normal=(0, 0);

	float alpha = 0.0f;
	const float targetDistance = input.tolerance; // 目标间距

	for (int iter = 0; iter < input.maxIterations; ++iter) {
		Vector2 normal;
		float distance = CalculateDistance(input, alpha, normal);

		// 1. 初始重叠检查 (只有穿深超过 SLOP 才报 Overlapped)
		if (iter == 0 && distance < -Settings::LINEAR_SLOP) {
			output.state = TOIOutput::Overlapped;
			output.alpha = 0.0f;
			output.normal = normal;
			return output;
		}

		// 2. 判定是否达到撞击条件 (满足容差范围)
		if (std::abs(distance - targetDistance) < 0.0001f) {
			output.state = TOIOutput::Hit;
			output.alpha = alpha;
			output.normal = normal;
			return output;
		}

		// 3. 计算相对速度
		Vector2 vA = input.bodyA->GetVelocity();
		Vector2 vB = input.bodyB->GetVelocity();

		// 线性接近速度 (点积越大，接近越快)
		float linearComponent = (vA - vB).Dot(normal);

		// 如果线性分量已经在远离，直接判定为安全
		if (linearComponent <= 0.0001f) {
			output.state = TOIOutput::Separated;
			output.alpha = 1.0f;
			return output;
		}

		// 旋转产生的保险速度
		float maxAngularVel = std::abs(input.bodyA->getAngularVelocity()) * input.bodyA->GetShape()->GetSweepRadius() +
			std::abs(input.bodyB->getAngularVelocity()) * input.bodyB->GetShape()->GetSweepRadius();

		float closingSpeed = linearComponent + maxAngularVel;

		// 4. 计算步进
		// alpha_step = 剩余距离 / 全速。确保分子非负
		float distance_to_travel = std::max(0.0f, distance - targetDistance);
		float alpha_step = distance_to_travel / (closingSpeed * input.dt);

		// 如果步进极小，说明已经收敛到撞击面
		if (alpha_step < 0.00001f && iter > 0) {
			output.state = TOIOutput::Hit;
			output.alpha = alpha;
			output.normal = normal;
			return output;
		}

		alpha += alpha_step;

		// 5. 退出条件检查
		if (alpha > 1.0f) {
			output.state = TOIOutput::Separated;
			output.alpha = 1.0f;
			return output;
		}
	}

	// 达到最大迭代次数，返回当前结果
	output.state = TOIOutput::Hit;
	output.alpha = alpha;
	CalculateDistance(input, alpha, output.normal); // 获取最终位置的法线
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
