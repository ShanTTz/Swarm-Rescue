# Swarm-Rescue · 正式蓝方 v13.1

当前正式版本：**v13.1，2026-10-12**。默认蓝方入口为 `MyDroneRescueV13`，A*规划。仿真源码与已检查版本完全一致；控制器无地图/炸弹先验。此文件夹可直接作为队伍工作目录。

最新8组复测：**7/8局全部归位、79/80枚归位、平均87.63分、平均健康82.25%**。Map01紧邻压力布局仍为9/10，并非所有布局已解决。旧v4成绩和回放独立保留。

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
| `tests/` | 算法、接口、地图和评分测试；`run.ps1 Test`选取129项相关回归 |
| `replays/v13_1/` | 最新版8局已保存回放 |
| `replays/v4/` | 历史v4的15局回放，不是v13.1成绩 |
| `evaluation/` | 已完成评测的原始CSV、日志、布局和统计，按版本/日期区分 |
| `results/` | 以后通过统一工具生成的新测试结果 |
| `templates/` | 新地图JSON和单图评测配置模板，尚未注册到模拟器 |
| `docs/` | 本项目使用说明；`reference/`为详细资料，`history/`为旧版资料 |
| `RELEASE.json` | 当前正式版、源码哈希、采用依据和能力边界 |

## 项目说明

1. [安装、单图测试、批量测试](docs/01_安装与运行.md)
2. [地图在哪、如何修改和添加地图](docs/02_地图与新增地图.md)
3. [回放怎么打开、分数怎么算](docs/03_回放与评分.md)
4. [算法组成、最新成绩、历史资料](docs/04_版本与评测.md)
5. [GitHub 推送、审核、队友使用与分支整合](docs/05_GitHub协作与管理说明.md)

**源代码、布局和回放已核验；未把虚拟环境、Git元数据和开发试验目录复制到本项目。** 原ZIP仍是旧v4，不作为此目录最新版本的下载包。
