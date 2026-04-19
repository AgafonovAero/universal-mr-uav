[CmdletBinding()]
param(
    [string]$DistroName = 'Ubuntu',
    [int]$MavlinkPort = 14550,
    [int]$JsonLocalPort = 9002,
    [int]$JsonRemotePort = 9003
)

$ErrorActionPreference = 'Stop'

function Get-RepoRoot {
    return (Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent)
}

function Get-WslInfo {
    $result = [ordered]@{
        HasWsl  = $false
        Distros = @()
    }

    $wslCommand = Get-Command wsl.exe -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -eq $wslCommand) {
        return [pscustomobject]$result
    }

    $result.HasWsl = $true

    try {
        $output = & wsl.exe -l -q 2>&1
        foreach ($line in $output) {
            $text = [string]$line
            if (-not [string]::IsNullOrWhiteSpace($text)) {
                $result.Distros += $text.Trim()
            }
        }
    }
    catch {
    }

    return [pscustomobject]$result
}

function Test-PortBusy {
    param([int]$Port)
    $netstat = netstat -ano -p udp
    return $netstat -match (":{0}\b" -f $Port)
}

function Find-File {
    param([string[]]$Paths)

    foreach ($path in $Paths) {
        if (-not [string]::IsNullOrWhiteSpace($path) -and (Test-Path $path)) {
            return $path
        }
    }

    return $null
}

$repoRoot = Get-RepoRoot
$wslInfo = Get-WslInfo
$ardupilotPathHint = $env:ARDUPILOT_ROOT
$missionPlannerPath = Find-File @(
    (Join-Path $env:ProgramFiles 'Mission Planner\MissionPlanner.exe'),
    (Join-Path ${env:ProgramFiles(x86)} 'Mission Planner\MissionPlanner.exe')
)
$qgcPath = Find-File @(
    (Join-Path $env:ProgramFiles 'QGroundControl\QGroundControl.exe'),
    (Join-Path ${env:ProgramFiles(x86)} 'QGroundControl\QGroundControl.exe'),
    (Join-Path $env:LOCALAPPDATA 'Programs\QGroundControl\QGroundControl.exe')
)

$scriptsToCheck = @(
    'tools/ardupilot/windows/Install-MissionPlanner.ps1'
    'tools/ardupilot/windows/Install-QGroundControl.ps1'
    'tools/ardupilot/windows/Start-ArduPilotStand.ps1'
    'tools/ardupilot/windows/Stop-ArduPilotStand.ps1'
    'tools/ardupilot/windows/Test-ArduPilotStand.ps1'
    'tools/ardupilot/wsl/setup_ardupilot_wsl.sh'
    'tools/ardupilot/wsl/start_arducopter_json_sitl.sh'
    'tools/ardupilot/wsl/check_ardupilot_wsl.sh'
)

Write-Host 'Проверка готовности стенда ArduPilot SITL'
Write-Host ("  WSL                                   : {0}" -f $(if ($wslInfo.HasWsl) { 'да' } else { 'нет' }))
Write-Host ("  Ubuntu                                : {0}" -f $(if ($wslInfo.Distros -contains $DistroName) { 'да' } else { 'нет' }))
Write-Host ("  ArduPilot path hint                   : {0}" -f $(if ($ardupilotPathHint) { $ardupilotPathHint } else { '<не задан>' }))
Write-Host ("  Mission Planner                       : {0}" -f $(if ($missionPlannerPath) { $missionPlannerPath } else { '<не найден>' }))
Write-Host ("  QGroundControl                        : {0}" -f $(if ($qgcPath) { $qgcPath } else { '<не найден>' }))
Write-Host ("  порт MAVLink {0} занят                : {1}" -f $MavlinkPort, $(if (Test-PortBusy $MavlinkPort) { 'да' } else { 'нет' }))
Write-Host ("  порт JSON local {0} занят             : {1}" -f $JsonLocalPort, $(if (Test-PortBusy $JsonLocalPort) { 'да' } else { 'нет' }))
Write-Host ("  порт JSON remote {0} занят            : {1}" -f $JsonRemotePort, $(if (Test-PortBusy $JsonRemotePort) { 'да' } else { 'нет' }))
Write-Host ''
Write-Host 'Наличие сценариев:'
foreach ($relativePath in $scriptsToCheck) {
    $exists = Test-Path (Join-Path $repoRoot $relativePath)
    Write-Host ("  {0} -> {1}" -f $relativePath, $(if ($exists) { 'да' } else { 'нет' }))
}

exit 0
