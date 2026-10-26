param(
    [ValidateSet('Start', 'Build', 'Stop', 'Status', 'View')]
    [string]$Action = 'Start',
    [string]$LocalRepo = '/home/stack/projects/px4-ros2-drone-nav',
    [string]$Distribution = 'Ubuntu',
    [string]$RemoteHost = 'jd@192.168.110.35',
    [string]$RemoteRepo = '/home/jd/projects/px4-ros2-drone-nav'
)
$ErrorActionPreference = 'Stop'
$Forward = '127.0.0.1:16087:127.0.0.1:6087'
$ViewerUrl = 'http://127.0.0.1:16087/vnc.html?autoconnect=1&resize=scale&path=websockify'
$OwnedTunnel = $null

function Invoke-Workflow([string]$Command) {
    & wsl.exe -d $Distribution --cd $LocalRepo -- env `
        "DRONE_REMOTE_HOST=$RemoteHost" "DRONE_REMOTE_REPO=$RemoteRepo" `
        'DRONE_REMOTE_SSH=/mnt/c/Windows/System32/OpenSSH/ssh.exe' `
        bash scripts/remote_sim.sh $Command
    if ($LASTEXITCODE -ne 0) { throw "Remote $Command failed (exit $LASTEXITCODE)." }
}

function Test-ViewerPort {
    $Client = New-Object System.Net.Sockets.TcpClient
    try {
        $Attempt = $Client.ConnectAsync('127.0.0.1', 16087)
        return ($Attempt.Wait(250) -and $Client.Connected)
    } catch { return $false } finally { $Client.Dispose() }
}

function Open-RemoteView {
    & ssh.exe -n -T -o BatchMode=yes -o ConnectTimeout=10 $RemoteHost `
        'systemctl --user start px4-nav-web.service'
    if ($LASTEXITCODE -ne 0) { throw 'Could not start the remote desktop viewer.' }

    if (Test-ViewerPort) {
        $Existing = Get-CimInstance Win32_Process -Filter "Name='ssh.exe'" |
            Where-Object { $_.CommandLine -like "*$Forward*" -and $_.CommandLine -like "*$RemoteHost*" }
        if (-not $Existing) { throw 'Local port 16087 is occupied by another application.' }
    } else {
        $script:OwnedTunnel = Start-Process -FilePath ssh.exe -NoNewWindow -PassThru -ArgumentList @(
            '-T', '-N', '-o', 'BatchMode=yes', '-o', 'ExitOnForwardFailure=yes',
            '-o', 'ServerAliveInterval=30', '-o', 'ServerAliveCountMax=3',
            '-L', $Forward, $RemoteHost
        )
        $Deadline = [DateTime]::UtcNow.AddSeconds(15)
        while (-not (Test-ViewerPort)) {
            if ($script:OwnedTunnel.HasExited) { throw 'The SSH display tunnel exited.' }
            if ([DateTime]::UtcNow -gt $Deadline) { throw 'The SSH display tunnel did not become ready.' }
            Start-Sleep -Milliseconds 200
        }
    }

    Write-Host 'Dedicated viewer password (not your Ubuntu login password):'
    & ssh.exe -n -T -o BatchMode=yes $RemoteHost 'cat "$HOME/.config/px4-nav/desktop-password.txt"'
    if ($LASTEXITCODE -ne 0) { throw 'Could not read the dedicated viewer credential.' }
    Start-Process $ViewerUrl
    Write-Host 'Keep the tunnel window open while viewing. Closing it does not stop the remote flight.'
}

try {
    switch ($Action) {
        'Start' {
            Invoke-Workflow 'start'
            Open-RemoteView
            Invoke-Workflow 'logs'
        }
        'Build' { Invoke-Workflow 'build' }
        'Stop' { Invoke-Workflow 'stop' }
        'Status' { Invoke-Workflow 'status' }
        'View' {
            Open-RemoteView
            if ($OwnedTunnel) { $OwnedTunnel.WaitForExit() }
        }
    }
} catch {
    Write-Host $_ -ForegroundColor Red
    exit 1
} finally {
    if ($OwnedTunnel -and -not $OwnedTunnel.HasExited) {
        Stop-Process -Id $OwnedTunnel.Id
    }
}
