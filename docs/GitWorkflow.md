# Git 工作流（提交与推送）

> 仓库地址：https://github.com/148627654/MyPhysicsEngine2D
> 提交前会自动运行 GoogleTest 全量测试（pre-commit 钩子），失败则阻止提交。

## 一、提交（含自动测试）

在 Git Bash 中：

```bash
# 1. 查看改动（确认没有混入不应上传的文件）
git status --short

# 2. 暂存并提交
git add -A
git commit -m "描述本次改动"
```

`git commit` 时会自动触发 pre-commit 钩子（`scripts/pre-commit` → `.git/hooks/pre-commit`）：

1. 检查 `googleTest/*.cpp` 是否都已注册进 `MyPhysicsEngine2DTests.vcxproj`（未注册会静默不跑，直接拦截）；
2. MSBuild 构建测试工程，PostBuild 自动运行全量测试；
3. 任一测试失败 → 提交被阻止。

紧急跳过测试：`SKIP_TESTS=1 git commit -m "..."`

## 二、推送

```bash
git push
```

首次推送需要关联远程（已配置则跳过）：

```bash
git remote add origin https://github.com/148627654/MyPhysicsEngine2D
git push -u origin main
```

如果远程仓库非空（已有 README 等），push 会被拒绝，先合并再推：

```bash
git pull --rebase origin main --allow-unrelated-histories
git push
```

## 三、首次使用需要配置身份

```bash
git config user.name "你的名字"
git config user.email "你的邮箱"
```

## 四、不上传的内容（.gitignore）

| 排除项 | 说明 |
|---|---|
| `.vs/`、`*.user` | VS 本地状态 |
| `MyPhysic.*/` | VS 缓存目录（如 MyPhysic.49ad12d2、MyPhysic.8F3C2D1A） |
| `x64/`、`output/` | 构建产物 |
| `.claude/` | Claude 会话数据 |
| `MyPhysicsEngine2D.*` | 工程/解决方案文件（vcxproj、filters、user、slnx） |

验证某路径是否被忽略：

```bash
git check-ignore -v .vs x64 .claude MyPhysicsEngine2D.vcxproj MyPhysic.8F3C2D1A
```
