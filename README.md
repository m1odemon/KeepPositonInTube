# Keep Position in Tube

本仓库采用单仓库、多子系统目录的方式管理滚球水管项目。每个队员在独立分支上
开发自己负责的子系统，通过 Pull Request 合并到 `main`。

## 仓库结构

| 目录 | 负责人/用途 | 当前状态 |
| --- | --- | --- |
| [`f4-tube-controller/`](f4-tube-controller/) | STM32F407、QD4310 无刷电机、连杆与水管控制 | 已提交 |
| [`chassis-controller/`](chassis-controller/) | 底板控制代码 | 等待底板队友提交 |
| [`raspberry-pi/`](raspberry-pi/) | 树莓派视觉、目标位置计算与上位机程序 | 等待树莓派队友提交 |

## 协作约定

1. 开始开发前，从最新 `main` 创建自己的功能分支。
2. 只修改自己负责的子系统目录；跨目录接口变更应在 PR 中说明。
3. 构建目录、缓存、临时文件和本机配置不得提交。
4. 提交前运行各子系统自己的构建与测试。
5. 通过 Pull Request 审查后再合并到 `main`。

示例：

```powershell
git switch main
git pull
git switch -c feature/your-feature
```

F4 固件的功能、构建、接线与安全说明见
[`f4-tube-controller/README.md`](f4-tube-controller/README.md)。
