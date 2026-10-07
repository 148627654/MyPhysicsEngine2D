#pragma once
#include "Body.h"

// 枚举joint的类型		杆状		弹簧	旋转/铰链	   焊接    滑块     摩擦        绳子
enum class JointType { Distance, Spring, Revolute, Unknown, Weld, Prismatic, Friction, Rope
};

// 限位状态机（Revolute 角度限位与 Prismatic 平移限位共用）
enum class LimitState {
    Inactive,  // 自由区间，限位不生效
    AtLower,   // 触碰下限位（只能产生推回正方向的单侧冲量）
    AtUpper,   // 触碰上限位（只能产生推回负方向的单侧冲量）
    Equal      // 上下限相等：双向等式约束，锁死该自由度（刚性连接）
};

/// @brief 关节通用定义
struct JointDef {
    JointType type = JointType::Unknown; ///< 关节类型（工厂分发依据）
    Body* bodyA = nullptr;               ///< 连接的刚体 A
    Body* bodyB = nullptr;               ///< 连接的刚体 B
    bool collideConnected = true;        ///< 连接双方是否允许物理碰撞（true=允许，Box2D 兼容默认）
    void* userData = nullptr;            ///< 用户数据
};

/// @brief 关节基类（约束系统）
///
/// 每个关节实现三个解算阶段（由 Island 解算器驱动）：
///  - initVelocityConstraints(dt)：每帧一次，计算有效质量并热启动
///  - solveVelocityConstraints()：速度迭代每轮一次，施加约束冲量
///  - solvePositionConstraints()：位置迭代每轮一次，Baumgarte 修正几何误差
///
/// 具体关节：DistanceJoint（刚性杆/弹性弹簧）、RevoluteJoint（销钉+限位+马达）、SpringJoint
class Joint {
public:
    Joint(const JointDef* def)
        : m_type(def->type), m_bodyA(def->bodyA), m_bodyB(def->bodyB),
        m_collideConnected(def->collideConnected), m_userData(def->userData) {
    }

    virtual ~Joint() = default;

    JointType getType() const { return m_type; }
    Body* getBodyA() const { return m_bodyA; }
    Body* getBodyB() const { return m_bodyB; }
    bool getCollideConnected() const { return m_collideConnected; }

    // 初始化速度约束
    virtual void initVelocityConstraints(float dt) = 0;
	// 求解速度约束
    virtual void solveVelocityConstraints() = 0;
	// 求解位置约束，返回是否满足约束条件
    virtual bool solvePositionConstraints() = 0;

    virtual Vector2 getAnchorA() const = 0; // 获取 BodyA 上的锚点世界坐标
    virtual Vector2 getAnchorB() const = 0; // 获取 BodyB 上的锚点世界坐标
protected:
    JointType m_type;
    Body* m_bodyA;
    Body* m_bodyB;
    bool m_collideConnected;
    void* m_userData;
    bool m_islandFlag = false; // 岛屿遍历标记（World 内部使用）

    friend class World;
    friend class Island;
};

struct JointEdge {
    Body* other = nullptr;
    Joint* joint = nullptr;
};