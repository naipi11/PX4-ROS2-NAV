param(
    [string]$DesktopDirectory = (Join-Path ([Environment]::GetFolderPath('Desktop')) 'PX4-ROS2仿真'),
    [string]$LocalRepo = '/home/stack/projects/px4-ros2-drone-nav',
    [string]$Distribution = 'Ubuntu',
    [string]$RemoteHost = 'jd@192.168.110.35',
    [string]$RemoteRepo = '/home/jd/projects/px4-ros2-drone-nav'
)
$ErrorActionPreference = 'Stop'
$Controller = (Resolve-Path (Join-Path $PSScriptRoot 'remote_sim.ps1')).ProviderPath
# The desktop holds only adapters; business logic always runs from this checkout.
foreach ($Value in @($Controller, $LocalRepo, $Distribution, $RemoteHost, $RemoteRepo)) {
    if ($Value -match '["%\r\n]') { throw 'Adapter arguments cannot contain quotes, percent signs or newlines.' }
}
$Launchers = @(
    @{ Name = '启动仿真.cmd'; Action = 'Start'; Pause = 'if errorlevel 1 pause' },
    @{ Name = '同步并构建.cmd'; Action = 'Build'; Pause = 'pause' },
    @{ Name = '停止仿真.cmd'; Action = 'Stop'; Pause = 'if errorlevel 1 pause' },
    @{ Name = '查看状态.cmd'; Action = 'Status'; Pause = 'pause' },
    @{ Name = '查看仿真.cmd'; Action = 'View'; Pause = 'pause' }
)
[void][System.IO.Directory]::CreateDirectory($DesktopDirectory)
$Encoding = New-Object System.Text.UTF8Encoding($false)
foreach ($Launcher in $Launchers) {
    $Command = 'powershell.exe -NoProfile -ExecutionPolicy Bypass -File "{0}" -Action {1} -LocalRepo "{2}" -Distribution "{3}" -RemoteHost "{4}" -RemoteRepo "{5}"' -f `
        $Controller, $Launcher.Action, $LocalRepo, $Distribution, $RemoteHost, $RemoteRepo
    $Lines = @(
        '@echo off',
        'setlocal DisableDelayedExpansion',
        'chcp 65001 >nul',
        "title PX4 ROS2 Remote Simulation - $($Launcher.Action)",
        $Command,
        'set "workflow_exit=%errorlevel%"',
        $Launcher.Pause,
        'exit /b %workflow_exit%'
    )
    $Destination = Join-Path $DesktopDirectory $Launcher.Name
    $Content = ($Lines -join "`r`n") + "`r`n"
    if (-not [System.IO.File]::Exists($Destination) -or [System.IO.File]::ReadAllText($Destination) -cne $Content) {
        [System.IO.File]::WriteAllText($Destination, $Content, $Encoding)
    }
}
Write-Host "Installed five desktop adapters pointing to $Controller"
Write-Host 'No SSH connection, service change or credential change was made.'
Write-Host 'After confirming the adapters work, remove only the obsolete desktop remote_sim.ps1.'
