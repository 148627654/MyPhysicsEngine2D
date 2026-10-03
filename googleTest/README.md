# GoogleTest 单元测试套件

用 [GoogleTest](https://github.com/google/googletest) 框架跑全量引擎测试。`tests/` 下的旧测试（每个自带 main）已全部移植为 TEST 用例并删除 —— **本目录是唯一的测试来源**。

## 目录结构

```
googleTest/
├── gtest/               vendored googletest v1.14.0 源码（include/ + src/，勿动）
├── LICENSE-gtest.txt    googletest 许可证
├── V3_001.cpp ~ V3_011.cpp  移植自 tests/V3/001~011 的 TEST 用例
└── README.md            本文档
```

配套的 `MyPhysicsEngine2DTests.vcxproj`（解决方案根目录）编译：引擎 `src/` 全部源文件 + `gtest-all.cc` + 本目录测试文件，生成 `x64/Debug/MyPhysicsEngine2DTests.exe`。

## 如何运行

| 方式 | 说明 |
|---|---|
| VS 里构建 `MyPhysicsEngine2DTests` 项目 | **PostBuild 自动运行全量测试**，测试失败 => 构建报 MSB3073 错 |
| MSBuild 命令行 | `MSBuild MyPhysicsEngine2DTests.vcxproj -p:Configuration=Debug -p:Platform=x64` |
| 直接跑 exe | `x64/Debug/MyPhysicsEngine2DTests.exe`（失败时输出完整诊断） |

常用 gtest 参数：

```
MyPhysicsEngine2DTests.exe --gtest_filter=V3_006.*     # 只跑某个套件
MyPhysicsEngine2DTests.exe --gtest_list_tests          # 列出全部用例
MyPhysicsEngine2DTests.exe --gtest_brief=1             # 简洁输出（PostBuild 已启用）
```

## 提交时自动跑（pre-commit 钩子）

`scripts/pre-commit` 已安装到 `.git/hooks/pre-commit`（项目已 `git init`）。每次 `git commit` 自动：

1. **拦截未注册的新测试文件**——`googleTest/*.cpp` 没写进 vcxproj 会静默不跑，钩子直接报错列出文件名；
2. 构建测试工程并运行全量用例；
3. 任一测试失败 => 提交被阻止。

强制跳过：`SKIP_TESTS=1 git commit`（改 `.git/hooks/pre-commit` 里的 MSBuild 查找逻辑也在这里）。

## 新增测试的流程（每次都要做）

以新增 `V3_012` 测试为例：

1. 新建 `googleTest/V3_012.cpp`，直接写 TEST 用例（无 main，参照现有文件风格）；
2. 在 `MyPhysicsEngine2DTests.vcxproj` 的 ClCompile 列表加一行：
   ```xml
   <ClCompile Include="googleTest\V3_012.cpp" />
   ```
   （漏掉这行会被 pre-commit 钩子拦截）
3. 构建测试工程（或直接提交）——自动跑全量。

> 新增引擎源文件（如新的 Joint）时，记得同时注册进主工程和 `MyPhysicsEngine2DTests.vcxproj`。

## 断言规则

| 场景代码 | gtest 宏 |
|---|---|
| `std::abs(x - y) < 1e-4f` | `EXPECT_NEAR(x, y, 1e-4f);` |
| `x > 3.0f` / `x < 0.01f` | `EXPECT_GT(x, 3.0f);` / `EXPECT_LT(x, 0.01f);` |
| `cond` | `EXPECT_TRUE(cond);` |
| `a == b` | `EXPECT_EQ(a, b);` |
| main() | 不需要（gtest_main 提供入口） |

要点：

- **场景逻辑原样保留**，只把断言拆成独立的 `EXPECT_*`（失败信息更精确）；
- 后续断言依赖前置条件成立时用 `ASSERT_*`（如 `ASSERT_TRUE(hit)` 后再读 `m.contacts`，防止读未初始化内存）；
- `Logger::info` 诊断日志可以删掉（gtest 失败时已打印断言上下文），保留也行（通过时被 gtest 吞掉）；
- **测试名用 ASCII**（`TEST(V3_012, MotorDrive)` 而不是中文）——控制台代码页是 GBK，中文测试名会乱码；
- 头文件直接用短名：`#include "World.h"`（测试工程的 include 目录已配好 Common/Collision/Dynamics/Events/Utils）。

## 已知注意点

- **接触对排序按 proxyId（加入世界的顺序）**：`Contact` 的 bodyA 恒为先加入世界的刚体，流形法线恒为 A→B。写依赖法线方向的断言时，记得"先加入者 = bodyA"。
- 位置修正是 Box2D 2.4 风格（β=0，纯位置求解）：高速弹性碰撞的瞬时穿透约 0.2，断言"未深陷"时阈值别设太紧（见 V3_002 的注释）。
