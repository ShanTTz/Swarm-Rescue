# Swarm-Rescue · 正式蓝方 v13.3

当前正式版 **v13.3，2026-10-15**。无地图/炸弹先验，算法入口仍为 `MyDroneRescueV13`。新增实际运动路径中的载弹优先、空载聚集/卡墙恢复，以及拥挤状态下逐步拉开间距的控制。

本次冻结源码完整测试 **43/43局真实归位10/10**；标准五图各三布局均分 **95.28 → 95.38**。完整自动化测试384项通过；上述43局包含两份用户场景与带窗口Map1复现。详情见[最新验收](docs/08_避让修复验证结果.md)。仍有局部停留，不保证任意新地图/seed/硬件条件均成功。旧版用户复现的失败原始记录已保留。

## 先这样使用

在本目录打开PowerShell：

```powershell
cd C:\Users\25923\Desktop\Swarm-Rescue
.\run.ps1 Test
.\run.ps1 Run -Map Map01 -Layout historic -Headless
.\run.ps1 Replay -Map Map01 -Layout historic
.\run.ps1 Scores
```

`run.ps1`优先使用本目录`.venv`。这台电脑尚未新建环境时，会复用已验证的原项目虚拟环境，并明确从**本目录src**导入代码；不会运行旧目录的控制器。移到其他电脑或删除原环境前，请按[安装与运行](docs/01_安装与运行.md)新建本地环境。第一次回放窗口处于暂停，按空格开始。

## 目录怎么找

| 目录/文件 | 用途 |
|---|---|
| `src/swarm_rescue/solutions/upgrade/controller_v13.py` | 当前蓝方入口实现，继承v12/v11控制模块 |
| `src/swarm_rescue/solutions/my_drone_eval.py` | 启动器选择红蓝算法的入口 |
| `src/swarm_rescue/maps/` | 模拟器实际使用的8张地图和墙体Python代码 |
| `map_data/` | 地图PNG预览、编辑器JSON、复杂图固定炸弹布局 |
| `fixtures/bombs/` | 五图各三个既有炸弹布局、Map1压力/启动检查布局 |
| `config/` | 运行配置；`rescue_Map01.yml`等开启分数保存与回放 |
| `scripts/` | 单图/批量运行、回放选择、分数复算、提交/物理检查 |
| `tests/` | 算法、接口、地图和评分测试；`run.ps1 Test`选取180项相关回归，`Test -Suite`执行全部384项 |
| `replays/v13_3/` | 当前正式版37个不同输入/seed回放 |
| `replays/v13_2/` | 历史v13.2回放 |
| `replays/v13_1/` | 历史v13.1的8局回放 |
| `replays/v4/` | 历史v4的15局回放，不是当前版本成绩 |
| `evaluation/` | 已完成评测的原始CSV、日志、布局和统计，按版本/日期区分 |
| `results/` | 以后通过统一工具生成的新测试结果 |
| `templates/` | 新地图JSON和单图评测配置模板，尚未注册到模拟器 |
| `docs/` | 本项目使用说明；`reference/`为详细资料，`history/`为旧版资料 |
| `RELEASE.json` | 当前正式版、源码哈希、采用依据和能力边界 |

## 使用与算法说明

1. [安装、单图测试、批量测试](docs/01_安装与运行.md)
2. [地图在哪、如何修改和添加地图](docs/02_地图与新增地图.md)
3. [回放怎么打开、分数怎么算](docs/03_回放与评分.md)
4. [算法组成与历史资料](docs/04_版本与评测.md)
5. [历史处置区修复](docs/05_处置区修复与稳定性验证.md)
6. [历史v13.2验收](docs/06_稳定性验收结果.md)
7. [载弹让行、空载聚集与卡墙修复](docs/07_无人机让行与空载脱困.md)
8. [当前逐局成绩与验收](docs/08_避让修复验证结果.md)

**源代码、布局和回放已核验；未把虚拟环境、Git元数据和开发试验目录复制到本项目。** 原ZIP仍是旧v4，不作为此目录最新版本的下载包。

## GitHub 协作与管理

推送更新、审核合并、队友使用和冲突处理，见[GitHub 协作与管理说明](docs/05_GitHub协作与管理说明.md)。
