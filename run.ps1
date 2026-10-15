param(
    [ValidateSet('Run','Test','Replay','Scores','MapEditor','BombEditor','Check','Smoke')]
    [string]$Action = 'Run',
    [string]$Map = 'Map01',
    [string]$Layout = 'historic',
    [string]$Version = 'v13_3',
    [string]$File,
    [int]$Seed = 12345,
    [switch]$Headless,
    [switch]$Suite,
    [switch]$List
)
$ErrorActionPreference = 'Stop'
$localPython = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
$verifiedPython = 'C:/Users/25923/Documents/Codex/2026-09-16/x20/swarm-eod-main/.venv/Scripts/python.exe'
if (Test-Path -LiteralPath $localPython) { $swarmPython = $localPython }
elseif (Test-Path -LiteralPath $verifiedPython) { $swarmPython = $verifiedPython }
else { throw 'No Python environment found. Follow docs/01_安装与运行.md to create .venv.' }
$env:PYTHONPATH = Join-Path $PSScriptRoot 'src'
$env:PYTHONIOENCODING = 'utf-8'
$env:PYTHONDONTWRITEBYTECODE = '1'
Push-Location $PSScriptRoot
try {
    switch ($Action) {
        'Run' {
            $runArgs = @('scripts/run_blue.py','--map',$Map,'--layout',$Layout,'--seed',"$Seed")
            if ($Headless) { $runArgs += '--headless' }
            if ($Suite) { $runArgs += '--suite' }
            if ($File) { $runArgs += @('--bombs',$File) }
            & $swarmPython @runArgs
        }
        'Replay' {
            $replayArgs = @('scripts/play_replay.py','--version',$Version,'--map',$Map,'--layout',$Layout,'--seed',"$Seed")
            if ($List) { $replayArgs += '--list' }
            if ($File) { $replayArgs += @('--file',$File) }
            & $swarmPython @replayArgs
        }
        'Scores' {
            if (-not $File) { $File = 'evaluation/v13_3_20261015/standard_final' }
            & $swarmPython scripts/summarize_scores.py $File
        }
        'MapEditor' { & $swarmPython -m swarm_rescue.tools.map_wall_editor }
        'BombEditor' {
            $bombArgs = @('-m','swarm_rescue.tools.bomb_editor',$Map)
            if ($File) { $bombArgs += @('--output',$File) }
            & $swarmPython @bombArgs
        }
        'Check' { & $swarmPython scripts/check_submission.py }
        'Smoke' { & $swarmPython scripts/smoke_algorithm_upgrade.py --algorithm v13 --planner astar --steps 120 }
        'Test' {
            $testArgs = @('scripts/run_tests.py')
            if ($Suite) { $testArgs += '--full' }
            & $swarmPython @testArgs
        }
    }
    $runExit = $LASTEXITCODE
} finally { Pop-Location }
exit $runExit
