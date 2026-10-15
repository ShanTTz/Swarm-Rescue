param(
    [int]$Port = 8765,
    [switch]$NoBrowser
)
# Swarm-Rescue 本地控制台入口。与 run.ps1 使用完全相同的环境探测逻辑：
# 优先本目录 .venv，其次已验证的原项目环境；console 不改动任何核心文件。
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$localPython = Join-Path $root '.venv/Scripts/python.exe'
$verifiedPython = 'C:/Users/25923/Documents/Codex/2026-09-16/x20/swarm-eod-main/.venv/Scripts/python.exe'
if (Test-Path -LiteralPath $localPython) { $swarmPython = $localPython }
elseif (Test-Path -LiteralPath $verifiedPython) { $swarmPython = $verifiedPython }
else { throw 'No Python environment found. Follow docs/01_安装与运行.md to create .venv.' }
$env:PYTHONIOENCODING = 'utf-8'
$env:PYTHONDONTWRITEBYTECODE = '1'
& $swarmPython (Join-Path $PSScriptRoot 'server.py') --port $Port $(if ($NoBrowser) { '--no-browser' })
