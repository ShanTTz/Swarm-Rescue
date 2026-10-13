# Swarm-Rescue · GitHub 协作与管理说明

适用仓库：[ShanTTz/Swarm-Rescue](https://github.com/ShanTTz/Swarm-Rescue)

整理日期：2026-10-13｜命令环境：Windows PowerShell｜主分支：`main`

建议团队采用：**每人建立功能分支 → 提交修改 → 推送分支 → 发起 PR → 审核 → 合并 → 大家更新代码**。

## 1. 这次上传了什么

桌面 `C:\Users\25923\Desktop\Swarm-Rescue` 的项目内容已导入 GitHub，源码版本为 v13.1。首次完整导入包含 **447 个文件**，提交为 [`47729ed`](https://github.com/ShanTTz/Swarm-Rescue/commit/47729ed943c96a765adc4d1f8b719469b155c685)。文件树与本地待上传版本的 Git 哈希一致。本说明及 README 入口在随后单独补充。

上传遵循原有 `.gitignore`：保留源码、地图、配置、文档、精选回放与评测数据；跳过 240 个文件，其中 `results/` 中 213 个、`evaluation/` 中被 `*.log` 匹配的日志 23 个、Python 缓存 4 个。桌面原文件保留。

**当前桌面原文件夹没有 `.git`，不会自动同步到 GitHub。** 本次通过独立副本完成上传。后续开发请按第 3 节建立正式 Git 工作目录；只修改原文件夹不会让网页内容跟着变化。

当前仓库是公开仓库。本文的邀请成员、主分支保护、发布版本属于操作说明；本次上传未替你配置这些设置，也未新增 GitHub Actions 自动测试。

## 2. 先分清这些操作

| 名称 | 作用 | 是否改变 GitHub 上的 `main` |
|---|---|---|
| Clone / 克隆 | 首次把仓库和版本历史下载到本机 | 否 |
| Branch / 分支 | 建立一条独立的修改路线 | 否 |
| Commit / 提交 | 把修改记入本地版本历史 | 否 |
| Push / 推送 | 把本地提交上传到指定远端分支 | 只有推到 `main` 才会直接改变它 |
| Pull / 拉取 | 把远端更新带到本地当前分支 | 否 |
| PR / Pull Request | 请求把一个分支的修改合入另一个分支 | 创建 PR 时不会 |
| Approve / 批准 | 审核者表示同意这次修改 | 否 |
| Merge / 合并 | 真正把修改整合进目标分支 | 目标为 `main` 时会 |
| Fork | 在另一个 GitHub 账号下建立仓库副本 | 否 |

**“Approve”是审核同意；“Merge”完成后，主分支才更新。** Push 到功能分支也不会自动生成 PR。

## 3. 你自己的电脑：建立可持续推送的工作目录

### 3.1 新克隆一份，作为今后的开发目录

桌面已有同名文件夹，因此使用新名字 `Swarm-Rescue-git`，保留原文件夹。以下操作由你在自己的 PowerShell 中执行；如果这个新目录也已存在，请另选一个空目录名。

```powershell
Set-Location ([Environment]::GetFolderPath('Desktop'))
git clone https://github.com/ShanTTz/Swarm-Rescue.git Swarm-Rescue-git
Set-Location .\Swarm-Rescue-git
git status
git remote -v
```

看到 `On branch main`、工作区干净，且 `origin` 指向上述仓库，就可以在这份目录里开发。以后打开编辑器、运行程序和执行 Git 命令，都使用这个目录。

如果原文件夹又产生了新修改，先在新克隆中按第 4 节创建分支，再把**确实修改过的项目文件**复制到相同相对路径，检查 `git diff` 后提交。不要把虚拟环境、运行结果或其他仓库的 `.git` 一起复制过去。

### 3.2 配置提交者身份

在克隆目录中执行，先替换引号里的内容。以下设置只影响当前仓库：

```powershell
git config user.name "你的 GitHub 显示名"
git config user.email "你的 GitHub 已验证邮箱或隐私邮箱"
git config --get user.name
git config --get user.email
```

GitHub 隐私邮箱可在个人 **Settings → Emails** 查看。姓名和邮箱是提交署名，不等于登录授权。

### 3.3 推送时如何登录

Windows 可使用 Git for Windows 自带的 Git Credential Manager。首次需要认证的推送通常会引导浏览器登录；登录拥有该仓库权限的账号。若已经安装 GitHub CLI，也可运行 `gh auth login`，选择 GitHub.com、HTTPS，并同意为 Git 配置认证。本机本次检测未发现 `gh`，所以它不是本文命令的前提。[GitHub 官方认证说明](https://docs.github.com/en/get-started/git-basics/caching-your-github-credentials-in-git)

本次上传使用已连接的 GitHub 账号完成，不代表你的终端已配置好登录；以后手动推送仍可能要求首次认证。

## 4. 你和有写权限的队友：怎么推送修改

下面以修改地图导航、分支名 `feat/map-navigation` 为例。每个新任务换一个明确的分支名，如 `fix/replay-loading`、`docs/install-guide`。

### 第一步：更新主分支并创建功能分支

开始前先确认 `git status` 没有未保存到 Git 的修改。如果有，先在合适的分支提交，或按第 9 节暂存。

```powershell
git switch main
git pull --ff-only origin main
git switch -c feat/map-navigation
```

### 第二步：修改、检查、提交

改完相关代码后，在已安装项目环境的前提下运行：

```powershell
.\run.ps1 Check
.\run.ps1 Test
git status
git diff
git add --all
git diff --cached --stat
git diff --cached
git commit -m "Improve map navigation"
```

`git add --all` 会包含所有未被忽略的新增、修改和删除；提交前检查暂存区，移除不属于本次任务的内容。纯文档修改检查文字与链接即可。算法修改还应进行相关地图的完整评测，参见第 10 节。

### 第三步：推送功能分支

```powershell
git push -u origin feat/map-navigation
```

同一个分支后续继续修改时：

```powershell
git add --all
git commit -m "Address review feedback"
git push
```

### 第四步：在网页发起 PR

打开[仓库 Pull requests](https://github.com/ShanTTz/Swarm-Rescue/pulls)，点击 **New pull request**，或使用推送后出现的 **Compare & pull request**：

- `base` 选 `main`，表示改动最终进入主分支。
- `compare` 选 `feat/map-navigation`，表示改动来自你的分支。
- 填写标题、改了什么、为什么改、如何验证，然后点击 **Create pull request**。
- 提交后发给审核者 PR 链接。继续向同一分支推送，已有 PR 会随之更新。

推荐 PR 描述：

```text
目的：解决什么具体问题？
改动：主要涉及哪些文件和行为？
验证：运行了哪些命令，结果如何？
评测：如有算法变化，写明地图、布局、种子和前后成绩。
影响：已知未解决的问题或使用条件。
```

如果只是你一个人维护且 `main` 允许直接推送，也可以在更新后的 `main` 上提交，再执行 `git push origin main`。团队合作建议走上面的功能分支和 PR 流程；启用主分支保护后，直接推送可能被拒绝。

## 5. 你怎么“同意”别人，以及怎么整合进主分支

### 5.1 同意一个 PR

1. 打开 **Pull requests → 对方的 PR**，确认目标是 `main`。
2. 在 **Files changed** 查看改动；检查是否夹带不相关文件，以及测试和评测证据。
3. 点击 **Review changes**，选择 **Approve → Submit review**。
4. 如果需要修改，选择 **Request changes**，具体指出问题；普通意见可用 **Comment**。

PR 作者不能批准自己的 PR。未配置强制审核规则时，写一条“需要修改”的审核意见不一定能阻止合并；想强制执行，需要第 7 节的分支保护。[GitHub 官方审核说明](https://docs.github.com/en/pull-requests/how-tos/review-pull-requests/reviewing-proposed-changes-in-a-pull-request)

### 5.2 真正合并

审核完成、冲突已解决，并满足页面列出的检查条件后，在 PR 下方选择 **Squash and merge → Confirm squash and merge**。这里推荐 Squash，把一个功能 PR 的多次调试提交合成一个清晰的主分支提交。需要保留每次提交时，可以选择 **Create a merge commit**。[GitHub 官方合并说明](https://docs.github.com/en/pull-requests/how-tos/merge-and-close-pull-requests/merging-a-pull-request)

只有看到 PR 状态为 **Merged** 才算整合完成。可删除已完成的远端功能分支。大家随后更新本地：

```powershell
git switch main
git pull --ff-only origin main
```

下一项任务从更新后的 `main` 建立新分支，不继续复用已经 Squash 合并过的旧分支。

### 5.3 多个人同时开发时，按顺序整合

例如 A 改地图、B 改控制器：先分别开 PR；合并 A 后，让 B 按第 8 节把最新 `main` 合入自己的分支，重新检查和测试，再审核合并 B。即使 Git 没报告文本冲突，两个修改也可能在行为上互相影响。

如果确实需要先联合验证多个尚未合并的分支，维护者可从 `origin/main` 新建 `integration/某个主题`，依次合并相关远端分支并测试，再发整合 PR；明确列出包含哪些分支，避免同一批修改再被重复合入。日常小改动优先一个 PR 一个功能。

## 6. 别人怎么获取、使用和贡献

### 6.1 只下载并运行：不需要成为协作者

当前仓库公开。队友可直接克隆；也可在网页 **Code → Download ZIP** 下载，但 ZIP 没有 Git 历史，后续不能直接 `git pull`。打算持续使用或修改，推荐克隆。

在队友选择的工作目录中执行：

```powershell
git clone https://github.com/ShanTTz/Swarm-Rescue.git
Set-Location .\Swarm-Rescue
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -c constraints-windows-py311.txt -e ".[dev]"
.\run.ps1 Check
.\run.ps1 Test
.\run.ps1 Run -Map Map01 -Layout historic -Headless
.\run.ps1 Replay -Map Map01 -Layout historic
```

需要 Git、Python 3.11 和可用显卡驱动/OpenGL；`Headless` 只是关闭显示窗口，传感器仍需要图形上下文。回放初始暂停时按空格。必须创建队友自己的 `.venv`，因为 `run.ps1` 的备用环境路径只适用于原电脑。完整项目操作见[安装与运行](https://github.com/ShanTTz/Swarm-Rescue/blob/main/docs/01_%E5%AE%89%E8%A3%85%E4%B8%8E%E8%BF%90%E8%A1%8C.md)。

### 6.2 固定队友：邀请为协作者

仓库所有者打开 **Settings → Collaborators → Add people**，输入对方 GitHub 用户名并发送邀请。对方通过 GitHub 通知或邀请邮件接受后，再按第 4 节推送功能分支。这是“接受协作邀请”，和批准代码 PR 是两个操作。[GitHub 官方邀请步骤](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/inviting-collaborators-to-a-personal-repository)

个人仓库的协作者拥有代码写权限；添加协作者本身不会自动要求“每次修改先经你批准”。应配合分支保护。[GitHub 个人仓库权限](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/permission-levels-for-a-personal-account-repository)

### 6.3 没有写权限：Fork 后提 PR

对方先在网页点击 **Fork**，把仓库复制到自己的账号下。把下面的 `YOUR_GITHUB_NAME` 替换成自己的 GitHub 用户名：

```powershell
git clone https://github.com/YOUR_GITHUB_NAME/Swarm-Rescue.git
Set-Location .\Swarm-Rescue
git remote add upstream https://github.com/ShanTTz/Swarm-Rescue.git
git fetch upstream
git switch -c feat/my-change upstream/main
# 修改文件并完成相应验证
git add --all
git commit -m "Describe the change"
git push -u origin feat/my-change
```

在网页创建跨 Fork 的 PR：**base repository** 为 `ShanTTz/Swarm-Rescue`、**base** 为 `main`；**head repository** 为自己的 Fork、**compare** 为 `feat/my-change`。原仓库维护者审核合并即可。[GitHub 官方 Fork PR 步骤](https://docs.github.com/en/pull-requests/how-tos/create-pull-requests/creating-a-pull-request-from-a-fork)

这里 `origin` 指对方自己的 Fork，`upstream` 指你的主仓库。更新这个功能分支时使用 `git fetch upstream`，再在功能分支上 `git merge upstream/main`。

## 7. 怎样让 `main` 必须经过审核

这是建议配置，需要仓库所有者在网页操作。本次仅编写说明，未启用规则。

打开 **Settings → Branches → Add classic branch protection rule**，分支名称填 `main`，建议选择：

- **Require a pull request before merging**：先开 PR。
- **Require approvals**：至少 1 位有资格的审核者批准。
- **Dismiss stale pull request approvals when new commits are pushed**：代码更新后重新审核。
- **Require conversation resolution before merging**：处理完评审讨论。
- 保持不允许强制推送和删除主分支。

如需管理员也遵守规则，可启用 **Do not allow bypassing the above settings**；先确认团队有足够审核者。作者不能自己批准，只有一个可审核账号时可能把自己的 PR 卡住。以后配置并实际运行 GitHub Actions 后，再选择要求通过的状态检查；不要先要求一个尚不存在的检查。[GitHub 官方分支保护说明](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/managing-a-branch-protection-rule)

**“要求 1 人批准”不等于“必须由 ShanTTz 批准”，也不等于“只有 ShanTTz 能点合并”。** 若暂时希望完全由你把关，可让其他人使用 Fork 提 PR，先不授予主仓库写权限。长期团队可进一步配置代码所有者审核或迁移到组织仓库，按角色管理权限。

## 8. 主分支更新了，自己的分支怎么整合

在自己已经提交完本地修改的功能分支上执行：

```powershell
git switch feat/map-navigation
git fetch origin
git merge --no-edit origin/main
```

没有冲突时完成合并，重新运行相关测试，然后 `git push`。如有冲突：

1. 用 `git status` 查看冲突文件。
2. 在编辑器里找到 `<<<<<<<`、`=======`、`>>>>>>>` 标记；比较两边代码，整理成需要保留的最终逻辑，删掉标记。不要机械地对所有文件“全部接受本地”或“全部接受远端”。
3. 逐个保存处理好的文件，执行下面的命令；把示例路径换成实际冲突文件。

```powershell
git add -- "实际冲突文件的相对路径"
git status
# 所有冲突都解决后再运行
git commit -m "Merge latest main and resolve conflicts"
.\run.ps1 Check
.\run.ps1 Test
git push
```

PNG、PDF、NPZ 等二进制文件通常不能按文本自动合并；与作者确认要保留哪版，或重新生成。当前这次 merge 尚未完成且想退出时，可执行 `git merge --abort`；不要用 `git push --force` 来解决冲突。[GitHub 官方冲突处理说明](https://docs.github.com/en/pull-requests/how-tos/merge-and-close-pull-requests/resolving-a-merge-conflict-using-the-command-line)

## 9. 常见问题怎么处理

| 提示或现象 | 处理方式 |
|---|---|
| `not a git repository` | 当前目录没有 `.git`。进入正式克隆目录；最初的桌面原文件夹属于这种情况。 |
| `Authentication failed` / 403 | 检查登录账号、`git remote -v` 和是否已接受协作者邀请。浏览器登录状态不一定等于终端凭据状态；按第 3.3 节配置认证。 |
| `non-fast-forward` / `fetch first` | 远端同一分支有新提交。先 `git fetch origin`，在当前功能分支合入 `origin/该功能分支`，处理冲突再推送。若是 `main`，优先转到功能分支提 PR。 |
| `git pull --ff-only` 失败 | 本地和远端已分叉；停止机械重试。先看 `git log --oneline --graph --all -15`，把本地需要保留的提交放到功能分支再整合。 |
| `nothing to commit` | 没有新的可提交修改。检查是不是编辑了原文件夹，却在新克隆目录执行 Git。 |
| 新文件没被上传 | 用 `git status --ignored` 查看；用 `git check-ignore -v -- "文件路径"` 找出命中的忽略规则。 |
| 网页已经合并，本地还是旧代码 | 在工作区干净时执行 `git switch main` 和 `git pull --ff-only origin main`。 |
| PR 合并按钮不可用 | 查看页面提示：可能仍是 Draft、缺审核、检查没通过或有冲突；按具体原因处理。 |
| PowerShell 拒绝运行脚本 | 可对这次执行使用 `powershell -ExecutionPolicy Bypass -File .\run.ps1 Test`；其他项目动作替换最后的参数。 |

临时收起未提交修改再拉取，可使用：

```powershell
git stash push -u -m "WIP before updating"
git pull --ff-only
git stash pop
```

此处 `-u` 包含未跟踪文件，但不包含被忽略文件。若 `stash pop` 出现冲突，按第 8 节处理；如果拉取失败，先解决分支状态，不要连续盲目执行后续命令。

## 10. 这个项目的评测与文件管理约定

源码、测试、配置和文档随功能 PR 提交。`.venv`、缓存、密钥和每次运行产生的完整 `results/` 留在本地；要共享正式成绩，将整理后的摘要、必要输入和精选回放放进 `evaluation/`、`fixtures/`、`replays/`。现有 `*.log` 规则会忽略日志，确需纳入正式日志时应讨论一条明确的 `.gitignore` 例外。

修改控制器、地图或评分逻辑时，建议提供：

- `run.ps1 Check` 与 `run.ps1 Test` 的实际结果。
- 地图、炸弹布局、随机种子、代码版本、归位数与得分。
- 与相同条件下旧版的对照，以及仍未解决的布局。

```powershell
.\run.ps1 Smoke
.\run.ps1 Run -Map Map01 -Layout historic -Headless
# 涉及通用策略变化时，再按需要做完整套件
.\run.ps1 Run -Suite -Headless
```

`Smoke` 是 120 步检查，不代表完整比赛成绩；套件会运行五图三个布局共 15 局。不要同时跑多个评分仿真，负载会影响墙钟时限和结果。具体解释以[项目安装与运行文档](https://github.com/ShanTTz/Swarm-Rescue/blob/main/docs/01_%E5%AE%89%E8%A3%85%E4%B8%8E%E8%BF%90%E8%A1%8C.md)为准。

这次任务验证了上传完整性，没有重新运行仿真或声称产生新的测试成绩。

## 11. 合错了怎么撤回

优先打开已经合并的 PR，使用 **Revert** 创建一个反向修改的新 PR，检查并测试后再合并；这样保留来龙去脉。若网页无法自动撤回，或存在冲突，需要在本地准备反向提交。[GitHub 官方撤回 PR 说明](https://docs.github.com/en/pull-requests/how-tos/merge-and-close-pull-requests/reverting-a-pull-request)

日常协作不要通过重写 `main` 历史、强推旧版本来撤回。撤回也要考虑：后续功能是否依赖被撤回的改动。

## 12. 怎样留一个大家都能复现的正式版本

稳定版本应关联固定提交。在 GitHub 的 **Releases → Draft a new release** 中，选择已有 Tag 或创建新 Tag，确认它指向经过验证的代码，填写改动和评测记录，准备好后点击 **Publish release**。标签名例如 `v13.2` 只表示未来版本示例，不代表本次已经发布。若有整理好的交付包，可作为 Release 附件。[GitHub 官方发布版本说明](https://docs.github.com/en/repositories/releasing-projects-on-github/managing-releases-in-a-repository)

团队在 PR 讨论中说明测试是否通过；在正式版本说明中记录代码提交、环境、地图、布局、种子和已知限制。后续发现问题开 Issue，附上复现步骤、报错、版本和输入文件，再以修复 PR 关联处理。

## 13. 一页日常操作顺序

| 谁 | 日常动作 |
|---|---|
| 开发者开始任务 | 更新 `main` → 新建功能分支 |
| 开发者完成改动 | 检查差异 → 按改动验证 → Commit → Push |
| 开发者请求整合 | 创建 PR，写清改动和验证证据 |
| 审核者 | 查看 Files changed → 评论 / Request changes / Approve |
| 维护者 | 满足条件后 Merge，确认状态为 Merged |
| 所有人 | 拉取最新 `main`，下一任务建立新分支 |

本文内链接指向项目实际文件及 GitHub 官方操作文档。网页按钮名称可能随界面调整，遇到差异以对应官方文档为准。
