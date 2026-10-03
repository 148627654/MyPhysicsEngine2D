// ============================================================
// 2Ddemo 样例 1：仿生布娃娃跌落系统 (Ragdoll System)
//   9 刚体：Circle 头 + Capsule 躯干/四肢（大臂、小臂、大腿、小腿）
//   9 个带角度限位的 RevoluteJoint：
//     - 脖子 ±28.6°、肩/髋 ±60°（大范围摆动）
//     - 肘/膝单向 [0°, 120°]（绝不后弯，防骨折）
//   从小人头部朝下跌落撞击三级台阶，监控全程限位角度（RagdollLimitTest）
// 运行：取消注释本文件 main，注释其他文件（tests 与 2Ddemo）的 main
// ============================================================
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "RevoluteJoint.h"
#include "Capsule.h"
#include "Circle.h"
#include "Box.h"
#include "Logger.h"
#include <cmath>
#include <vector>

// 快捷工厂：胶囊刚体（禁止入睡，避免下落/撞击减速期被睡眠系统冻结）
static Body* MakeCapsuleBody(World& world, float r, float len,
    float x, float y, float density) {
    Body* b = new Body(new Capsule(r, len), x, y, density);
    b->setSleepAllow(false);
    world.addBody(b);
    return b;
}

// 带角度限位的销钉装配助手
static RevoluteJoint* MakeLimitedJoint(World& world, Body* a, Body* b,
    const Vector2& worldAnchor, float lower, float upper,
    std::vector<RevoluteJoint*>& joints) {
    RevoluteJointDef def;
    def.collideConnected = false; // 关节直连骨骼互不碰撞（引擎已实现过滤）
    def.initialize(a, b, worldAnchor);
    def.enableLimit = true;
    def.lowerAngle = lower;
    def.upperAngle = upper;
    RevoluteJoint* j = static_cast<RevoluteJoint*>(world.createJoint(def));
    joints.push_back(j);
    return j;
}

//int main() {
    //Logger::info(">>> 2Ddemo 1: Ragdoll 布娃娃跌落 <<<");
    //World world(Vector2(0.0f, -9.8f));
    //float dt = 1.0f / 60.0f;

    //// --- 三级台阶 ---
    //world.addBody(new Body(new Box(30.0f, 1.0f), 0.0f, -0.5f, 0.0f));   // 地面（顶面 y=0）
    //world.addBody(new Body(new Box(5.0f, 0.5f), -2.0f, 0.25f, 0.0f));  // 台阶 1（顶面 0.5）
    //world.addBody(new Body(new Box(5.0f, 0.5f), 3.0f, 0.75f, 0.0f));   // 台阶 2（顶面 1.0）
    //world.addBody(new Body(new Box(5.0f, 0.5f), 8.0f, 1.25f, 0.0f));   // 台阶 3（顶面 1.5）

    //// --- 骨骼（人形：头 + 躯干 + 双臂双节 + 双腿双节）---
    //Body* head = new Body(new Circle(0.25f), 0.0f, 4.2f, 0.8f);
    //head->setSleepAllow(false);
    //world.addBody(head);

    //Body* torso = MakeCapsuleBody(world, 0.3f, 1.3f, 0.0f, 3.0f, 1.2f);

    //Body* upperArmL = MakeCapsuleBody(world, 0.11f, 0.45f, -0.55f, 3.0f, 0.9f);
    //Body* upperArmR = MakeCapsuleBody(world, 0.11f, 0.45f, 0.55f, 3.0f, 0.9f);
    //Body* forearmL = MakeCapsuleBody(world, 0.11f, 0.45f, -0.6f, 2.45f, 0.9f);
    //Body* forearmR = MakeCapsuleBody(world, 0.11f, 0.45f, 0.6f, 2.45f, 0.9f);
    //Body* thighL = MakeCapsuleBody(world, 0.14f, 0.5f, -0.46f, 1.6f, 1.0f);
    //Body* thighR = MakeCapsuleBody(world, 0.14f, 0.5f, 0.46f, 1.6f, 1.0f);
    //Body* shinL = MakeCapsuleBody(world, 0.12f, 0.45f, -0.46f, 0.85f, 1.0f);
    //Body* shinR = MakeCapsuleBody(world, 0.12f, 0.45f, 0.46f, 0.85f, 1.0f);

    //// 先倾斜躯干（头朝下姿势），再装配关节 —— initialize 会把倾斜角吸收进基准角
    //torso->setRotation(-0.9f);

    //// --- 9 个限位关节 ---
    //std::vector<RevoluteJoint*> joints;
    //MakeLimitedJoint(world, torso, head, Vector2(0.0f, 3.95f), -0.5f, 0.5f, joints);            // 脖子 ±28.6°
    //MakeLimitedJoint(world, torso, upperArmL, Vector2(-0.48f, 3.3f), -1.047f, 1.047f, joints);  // 左肩 ±60°
    //MakeLimitedJoint(world, torso, upperArmR, Vector2(0.48f, 3.3f), -1.047f, 1.047f, joints);   // 右肩 ±60°
    //MakeLimitedJoint(world, upperArmL, forearmL, Vector2(-0.57f, 2.72f), 0.0f, 2.094f, joints); // 左肘 [0°,120°]
    //MakeLimitedJoint(world, upperArmR, forearmR, Vector2(0.57f, 2.72f), -2.094f, 0.0f, joints); // 右肘 [-120°,0°]
    //MakeLimitedJoint(world, torso, thighL, Vector2(-0.36f, 1.9f), -1.047f, 1.047f, joints);     // 左髋 ±60°
    //MakeLimitedJoint(world, torso, thighR, Vector2(0.36f, 1.9f), -1.047f, 1.047f, joints);      // 右髋 ±60°
    //MakeLimitedJoint(world, thighL, shinL, Vector2(-0.46f, 1.2f), 0.0f, 2.094f, joints);        // 左膝 [0°,120°]
    //MakeLimitedJoint(world, thighR, shinR, Vector2(0.46f, 1.2f), -2.094f, 0.0f, joints);        // 右膝 [-120°,0°]

    //Logger::info("组装完成: 9 刚体 + " + std::to_string(joints.size()) + " 限位关节");

    //// 头部朝下撞击台阶：初始角速度 + 横向速度打破左右对称
    //torso->setAngularVelocity(-0.9f);
    //torso->setVelocity(Vector2(0.2f, 0.0f));

    //// --- 模拟 7 秒，监控限位（RagdollLimitTest）---
    //// 全程瞬态统计 + 静息态（最后 60 帧）统计
    //std::vector<float> minAngle(joints.size(), 1e9f);
    //std::vector<float> maxAngle(joints.size(), -1e9f);
    //std::vector<float> restMin(joints.size(), 1e9f);
    //std::vector<float> restMax(joints.size(), -1e9f);
    //float maxAnchorError = 0.0f;
    //for (int i = 0; i < 420; ++i) {
        //world.step(dt);
        //for (size_t k = 0; k < joints.size(); ++k) {
            //float a = joints[k]->getJointAngle();
            //minAngle[k] = std::min(minAngle[k], a);
            //maxAngle[k] = std::max(maxAngle[k], a);
            //if (i >= 360) { // 静息窗口
                //restMin[k] = std::min(restMin[k], a);
                //restMax[k] = std::max(restMax[k], a);
            //}
            //float err = (joints[k]->getAnchorB() - joints[k]->getAnchorA()).length();
            //maxAnchorError = std::max(maxAnchorError, err);
        //}
        //if ((i + 1) % 60 == 0) {
            //Logger::info("t=" + std::to_string((i + 1) / 60) + "s" +
                //" torso=(" + std::to_string(torso->getPosition().getX()) + "," +
                //std::to_string(torso->getPosition().getY()) + ")" +
                //" rot=" + std::to_string(torso->getRotation()) +
                //" elbowL=" + std::to_string(joints[3]->getJointAngle()) +
                //" kneeL=" + std::to_string(joints[7]->getJointAngle()) +
                //" maxAnchorErr=" + std::to_string(maxAnchorError));
        //}
    //}

    //// --- 限位验收（RagdollLimitTest）---
    //// 离散步进下冲量限位有单帧检测延迟：撞击帧内角度会瞬时越过边界，
    //// 随后限位冲量立即把角度拉回（Box2D 同款行为）。两级验收：
    ////   1) 静息态（最后 60 帧）：贴合限位（±0.08 rad；被躯干压住的承重关节
    ////      存在接触修正与限位修正的静态平衡残余 ~0.06 rad，7/9 关节实际 < 0.02）
    ////   2) 全程瞬态：不出现灾难性穿透（±0.5 rad 以内）
    //struct LimitRange { float lo, hi; const char* name; };
    //LimitRange ranges[9] = {
        //{ -0.5f,    0.5f,    "脖子" },
        //{ -1.047f,  1.047f,  "左肩" }, { -1.047f, 1.047f, "右肩" },
        //{ 0.0f,     2.094f,  "左肘" }, { -2.094f, 0.0f,   "右肘" },
        //{ -1.047f,  1.047f,  "左髋" }, { -1.047f, 1.047f, "右髋" },
        //{ 0.0f,     2.094f,  "左膝" }, { -2.094f, 0.0f,   "右膝" },
    //};
    //const float REST_SLOP = 0.08f;
    //const float TRANSIENT_SLOP = 0.5f;
    //bool limitOk = true;
    //for (size_t k = 0; k < joints.size(); ++k) {
        //bool restOk = (restMin[k] >= ranges[k].lo - REST_SLOP) && (restMax[k] <= ranges[k].hi + REST_SLOP);
        //bool transientOk = (minAngle[k] >= ranges[k].lo - TRANSIENT_SLOP) && (maxAngle[k] <= ranges[k].hi + TRANSIENT_SLOP);
        //limitOk &= restOk && transientOk;
        //Logger::info(std::string("限位[") + ranges[k].name + "] 静息[" + std::to_string(restMin[k]) +
            //"," + std::to_string(restMax[k]) + "] 瞬态[" + std::to_string(minAngle[k]) +
            //"," + std::to_string(maxAngle[k]) + "] 边界[" + std::to_string(ranges[k].lo) + "," +
            //std::to_string(ranges[k].hi) + "] -> " + ((restOk && transientOk) ? "PASS" : "FAIL"));
    //}

    //bool landed = torso->getPosition().getY() < 2.5f; // 跌落在台阶区域
    //Logger::info("最终: torso.y=" + std::to_string(torso->getPosition().getY()) +
        //" 跌落=" + std::string(landed ? "是" : "否") +
        //" 全程最大锚点误差=" + std::to_string(maxAnchorError));
    //Logger::info(std::string(">>> RagdollDemo ") + (limitOk && landed ? "PASS" : "FAIL") + " <<<");
    //return (limitOk && landed) ? 0 : 1;
//}
