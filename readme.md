# MyPhysicsEngine2D - V2 性能巅峰篇

一个基于 C++11 构建的高仿真 2D 刚体物理引擎。在 V1 稳健的动力学基础上，V2 致力于工程化性能优化与极端场景下的数值稳定性。

## 📌 项目愿景
V2 阶段的目标是将引擎从“逻辑原型”打造为“高性能工业库”。通过空间换时间、能量管理和连续碰撞检测，支撑起上千量级刚体的实时物理沙盒。
- **极致性能**：实现 $O(N \log N)$ 复杂度的碰撞查询，支持 1000+ 物体同屏。
- **智能节电**：引入休眠机制，消除静止堆叠产生的无效计算。
- **高速防御**：攻克“穿墙”难题，确保高速运动下的物理连续性。
- **工程量化**：内置 Profiler 性能分析器，用数据驱动每一行代码的优化。

---

## 🛠 项目结构 (V2 更新)
```text
MyPhysicsEngine2D/
├── include/
│   └── physics/
│       ├── Collision/
│       │   ├── DynamicTree.h      # <--- [V2] 动态 AABB 树核心 (支持旋转与平衡)
│       │   ├── AABB.h             # <--- [V2] 增加 Union, Perimeter 计算
│       │   └── Box.h
│       ├── Dynamics/
│       │   ├── Body.h             # <--- [V2] 存储 ProxyId 
│       │   └── ...
├── src/
│   ├── Collision/
│   │   ├── DynamicTree.cpp        # <--- [V2] 插入/平衡/删除逻辑实现
│   │   └── ...
├── tests/
│   ├── TreePoolTests.cpp          # <--- [V2] 内存管理测试
│   ├── TreeInsertTests.cpp        # <--- [V2] 插入与 SAH 代价算法测试
│   └── TreeRotationTests.cpp      # <--- [V2] Day 03 专用：自平衡与删除测试
└── README.md
```

---

## 📅 进度跟踪 (V2 15天挑战)

### 第 1 阶段：空间加速——动态 AABB 树
- [x] **Day 01: 基础节点池与内存管理**
  - 实现基于索引 (`int32_t`) 的节点存储，规避指针失效与内存碎片。
  - 实现 **Free List (空闲链表)** 机制，达到 $O(1)$ 的节点分配与回收。
  - 实现数组自动动态扩容，支持大规模节点平滑增长。
  - 完成 `userData` 与 `Body` 的双向绑定。
- [x] **Day 02: 启发式插入算法 (InsertLeaf)**
  - 实现基于 **SAH (表面积启发式)** 的最佳兄弟节点搜索。
  - 实现内部节点自动生成逻辑，构建满二叉树结构。
  - 实现 **Bottom-up 更新机制**，确保父节点 AABB 实时包裹所有子孙。
  - 完成 `userData` 与 `Body` 的物理层级绑定。
- [x] **Day 03: 树旋转自平衡与代理删除 (Tree Rotations & Removal)**
  - 实现 **AVL 风格旋转算法**，在插入/删除过程中自动调整树高。
  - 实现 `RemoveLeaf` 逻辑，支持兄弟节点自动提拔与内存安全回收。
  - 验证 12 节点在高强度删除下的 $O(\log N)$ 稳定性。
## 🚀 Day 01 进展：动态树基础架构 (Node Pool)

### 1. 技术核心：索引式内存池
为了承载高频率的树重构，V2 弃用了 `new/delete`。
- **为什么使用 `int32_t` 索引？**
  - **内存减半**：在 64 位系统下，索引比指针节省 50% 的链接存储空间。
  - **搬运安全**：数组扩容导致重新分配内存时，索引依然有效，杜绝了野指针问题。
- **空闲链表 (Free List)**：
  - 利用节点内的 `next` 指针将未使用的槽位串联。
  - **回收复用**：`DestroyProxy` 时节点立即进入 Free List，下次分配优先填补空洞。

### 2. `Node` 结构设计
```cpp
struct Node {
    AABB aabb;        // 空间边界
    void* userData;   // 双向绑定 Body*
    int32_t parent;   // 父节点索引
    int32_t leftChild, rightChild; // 子节点索引
    int32_t next;     // 用于空闲列表
    int32_t height;   // 用于 AVL 平衡
    
    bool IsLeaf() const { return leftChild == -1; }
};
```

### 3. 如何验证
运行 `tests/TreePoolTests.cpp`。当前已通过以下工程校验：
- ✅ **内存复用测试**：销毁 Slot 1 后再次创建，系统精准回填 Slot 1 而非开辟新槽位。
- ✅ **自动扩容测试**：当节点数超过 16 时，数组容量翻倍至 32，且旧数据与空闲链表保持一致。
- ✅ **Body 绑定校验**：通过 `PrintPool` 实时读取 `userData` 转换后的 `Body` 坐标。

**Day 01 运行快照：**
```text
[INFO] Action: Destroying Slot 1 (0x222)...
[INFO] === DynamicTree Node Pool State ===
Capacity: 16 | Active Count: 2 | FreeList Head: 1
[Slot  1] TYPE: FREE   | NEXT:  3  <-- 索引 1 已被回收
[INFO] Action: Creating a new proxy...
[Slot  1] TYPE: ACTIVE | BODY POS: ( 0.0, 0.0) <-- 索引 1 成功复用
[INFO] SUCCESS: Slot 1 was correctly recycled!
```

---
## 🚀 Day 02 进展：启发式插入与层级构建

### 1. 技术核心：满二叉树构建
动态 AABB 树通过“二合一”插入逻辑保持满二叉树状态：
- **寻找最佳兄弟**：当新物体进入时，遍历树分支，计算将新物体塞入该分支后导致的“周长增加量 (Cost)”，选择代价最小的方向。
- **自动升舱**：当确定位置后，从池中取出一个内部节点作为“新爸爸”，将原有节点和新节点挂载其下。

### 2. AABB 向上回溯 (Bottom-up Update)
为了保证碰撞查询的准确性，实现了递归回溯：
- 任何叶子节点的变化都会触发其父辈、祖辈节点的 AABB 重新计算。
- 内部节点的 AABB 始终通过 `AABB::Union(child1, child2)` 保持最紧凑的包裹。

### 3. 如何验证
运行 `tests/TreeInsertTests.cpp`。通过 Body 坐标偏移验证：
- ✅ **层级倍增校验**：插入 3 个物体，Active Count 准确从 1 增长到 5 (3叶子 + 2内部)，证明满二叉树生成逻辑正确。
- ✅ **空间包裹校验**：验证根节点 AABB 的 Min/Max。例如插入 (10,10) 和 (20,20) 后，Root AABB 成功扩展为包围两者的巨大盒子。
- ✅ **结构健康检查**：通过 `Validate()` 函数确认所有父子索引双向匹配，无死循环。

**Day 02 运行快照：**
```text
[INFO] Inserted Body 2 at (20,20).
[INFO] === DynamicTree Node Pool State ===
Capacity: 16 | Active Count: 3 | FreeList Head: 3
[Slot  2] TYPE: ACTIVE | USERDATA: nullptr  <-- 自动生成的内部节点
[INFO] Root AABB Min: (9.5, 9.5) 
[INFO] Root AABB Max: (20.5, 20.5) 
[INFO] SUCCESS: Root AABB correctly encapsulates both children!
```
---
## 🚀 Day 03 进展：自平衡算法与动态缩减

### 1. 技术核心：AVL 旋转 (Tree Rotations)
动态树最怕物体按线性排列插入（如一排地基），这会导致树退化为 $O(N)$ 复杂度的链表。
- **旋转判定**：每当检测到左右子树高度差（Balance Factor）超过 1 时，触发旋转。
- **节点提拔**：通过交换指针，将被“压扁”的深层节点提拔至高位，降低全局搜索代价。
- **四种旋转**：支持 LL, RR, LR, RL 全套旋转逻辑，确保树高始终维持在 $\lceil \log_2 N \rceil + 1$。

### 2. 节点移除与 AABB 动态缩减
- **兄弟提拔机制**：删除一个叶子后，其父节点被标记为 FREE，其兄弟节点自动接管父节点在树中的位置。
- **实时收缩**：删除操作会触发从删除点向上的 AABB 重新计算，根节点 AABB 会随之自动收缩至仅包含剩余物体的范围。

### 3. 树结构演变图解 (根据 Day 03 运行结果)

#### **阶段 A: 12 个物体插入后的完美平衡树 (Root: 8, Height: 4)**
```text
--- Tree Hierarchy (Root: 8) ---
[Slot 8] INTERNAL (Height: 4, X: -0.5 to 22.5)
|--[Slot 4] INTERNAL (Height: 2, X: -0.5 to 6.5)
   |--[Slot 2] INTERNAL (Height: 1, X: -0.5 to 2.5)
      |--[Slot 0] LEAF (BodyX: 0)
      |--[Slot 1] LEAF (BodyX: 2)
   |--[Slot 6] INTERNAL (Height: 1, X: 3.5 to 6.5)
      |--[Slot 3] LEAF (BodyX: 4)
      |--[Slot 5] LEAF (BodyX: 6)
|--[Slot 16] INTERNAL (Height: 3, X: 7.5 to 22.5)
   |--[Slot 12] INTERNAL (Height: 2, X: 7.5 to 14.5)
      |--[Slot 10] INTERNAL (Height: 1, X: 7.5 to 10.5)
         |--[Slot 7] LEAF (BodyX: 8)
         |--[Slot 9] LEAF (BodyX: 10)
      |--[Slot 14] INTERNAL (Height: 1, X: 11.5 to 14.5)
         |--[Slot 11] LEAF (BodyX: 12)
         |--[Slot 13] LEAF (BodyX: 14)
   |--[Slot 20] INTERNAL (Height: 2, X: 15.5 to 22.5)
      |--[Slot 18] INTERNAL (Height: 1, X: 15.5 to 18.5)
         |--[Slot 15] LEAF (BodyX: 16)
         |--[Slot 17] LEAF (BodyX: 18)
      |--[Slot 22] INTERNAL (Height: 1, X: 19.5 to 22.5)
         |--[Slot 19] LEAF (BodyX: 20)
         |--[Slot 21] LEAF (BodyX: 22)
```

#### **阶段 B: 执行删除操作 (移除索引 1,3,5,7,9,11)**
当中间和边缘的节点被移除后，树通过 **旋转** 重新寻路，根节点从 Slot 8 转移到了 **Slot 16**，树高降低为 **3**。
```text
--- Tree Hierarchy (Root: 16) ---
[Slot 16] INTERNAL (Height: 3, X: -0.5 to 20.5)
|--[Slot 8] INTERNAL (Height: 2, X: -0.5 to 12.5)
   |--[Slot 4] INTERNAL (Height: 1, X: -0.5 to 4.5)
      |--[Slot 0] LEAF (BodyX: 0.0)
      |--[Slot 3] LEAF (BodyX: 4.0)
   |--[Slot 12] INTERNAL (Height: 1, X: 7.5 to 12.5)
      |--[Slot 7] LEAF (BodyX: 8.0)
      |--[Slot 11] LEAF (BodyX: 12.0)
|--[Slot 20] INTERNAL (Height: 1, X: 15.5 to 20.5)
   |--[Slot 15] LEAF (BodyX: 16.0)
   |--[Slot 19] LEAF (BodyX: 20.0)
```

### 4. 如何验证
运行 `tests/TreeRotationTests.cpp`。当前已通过以下校验：
- ✅ **高度压缩测试**：12 节点初始高度 4，删除一半后高度降为 3。
- ✅ **平衡因子校验**：删除过程中无任何节点高度差超过 1。
- ✅ **AABB 自动缩减**：删除最大 X 坐标节点后，Root AABB Max.X 从 22.5 精准收缩至 20.5。
- ✅ **内存池一致性**：`Active Count` 从 23 减少到 11，且 Free List 指针链逻辑正确。

**Day 03 运行快照：**
```text
[INFO] Action: Removing odd-indexed bodies...
[INFO] Tree Height after removal: 3
[INFO] SUCCESS: Tree remains balanced after removals!
[INFO] Root AABB Max: 20.5 (Shrunk from 22.5)
[INFO] --- Tree Hierarchy (Root: 16) ---
[Slot 16] INTERNAL (Height: 3, X: -0.5 to 20.5)
|--[Slot 8] INTERNAL (Height: 2, X: -0.5 to 12.5)
|--[Slot 20] INTERNAL (Height: 1, X: 15.5 to 20.5)
```

## 💻 编译与运行
- **环境**：Visual Studio 2019+ (C++11)
- **入口**：在 `main.cpp` 中调用 `RunRealBodyTreeTest()` 即可观察 V2 底层内存池运作。
