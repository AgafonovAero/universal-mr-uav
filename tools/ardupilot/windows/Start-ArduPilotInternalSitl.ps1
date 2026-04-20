[CmdletBinding()]
param(
    [string]$DistroName = 'Ubuntu',
    [switch]$NoConsole,
    [switch]$NoMap,
    [string]$Ip = '127.0.0.1',
    [int]$MavlinkPort = 14550,
    [int]$SecondaryMavlinkPort = 14552,
    [switch]$LaunchMissionPlanner,
    [switch]$LaunchQGroundControl,
    [switch]$Execute,
    [string]$LogPath,
    [string]$StatePath
)

$ErrorActionPreference = 'Stop'

function Get-RepoRoot {
    return (Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent)
}

function Convert-ToWslPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$WindowsPath
    )

    $fullPath = [System.IO.Path]::GetFullPath($WindowsPath)
    $drive = $fullPath.Substring(0, 1).ToLowerInvariant()
    $tail = $fullPath.Substring(2).Replace('\', '/')

    if ($tail.StartsWith('/')) {
        $tail = $tail.Substring(1)
    }

    return "/mnt/$drive/$tail"
}

function Write-Utf8NoBomFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Text
    )

    $directory = Split-Path $Path -Parent
    if (-not [string]::IsNullOrWhiteSpace($directory)) {
        [System.IO.Directory]::CreateDirectory($directory) | Out-Null
    }

    $encoding = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($Path, $Text, $encoding)
}

function Resolve-WindowsHostIpFromWsl {
    param(
        [Parameter(Mandatory = $true)]
        [string]$TargetDistro
    )

    try {
        $resolved = & wsl.exe -d $TargetDistro -- bash -lc "ip -4 route list default | cut -d' ' -f3 | head -n 1" 2>$null
        $resolved = ($resolved | Select-Object -First 1).Trim()
        if (-not [string]::IsNullOrWhiteSpace($resolved)) {
            return $resolved
        }
    }
    catch {
    }

    return $null
}

function Find-ExecutablePath {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$CandidatePaths
    )

    foreach ($path in $CandidatePaths) {
        if (-not [string]::IsNullOrWhiteSpace($path) -and (Test-Path $path)) {
            return $path
        }
    }

    return $null
}

$repoRoot = Get-RepoRoot
if ([string]::IsNullOrWhiteSpace($LogPath)) {
    $logPath = Join-Path $repoRoot 'artifacts/logs/task_18_internal_sitl_start.txt'
}
else {
    $logPath = [System.IO.Path]::GetFullPath($LogPath)
}

if ([string]::IsNullOrWhiteSpace($StatePath)) {
    $statePathValue = Join-Path $repoRoot 'artifacts/reports/task_18_internal_state.json'
}
else {
    $statePathValue = [System.IO.Path]::GetFullPath($StatePath)
}

$scriptPath = Join-Path $repoRoot 'tools/ardupilot/wsl/start_arducopter_internal_sitl.sh'
$scriptPathWsl = Convert-ToWslPath -WindowsPath $scriptPath
$effectiveIp = $Ip

if ($effectiveIp -in @('127.0.0.1', 'localhost')) {
    $resolvedIp = Resolve-WindowsHostIpFromWsl -TargetDistro $DistroName
    if (-not [string]::IsNullOrWhiteSpace($resolvedIp)) {
        $effectiveIp = $resolvedIp
    }
}

$argList = New-Object System.Collections.Generic.List[string]
$argList.Add($scriptPathWsl)
$argList.Add('--ip')
$argList.Add($effectiveIp)
$argList.Add('--mavlink-port')
$argList.Add($MavlinkPort.ToString())
if ($SecondaryMavlinkPort -gt 0) {
    $argList.Add('--secondary-mavlink-port')
    $argList.Add($SecondaryMavlinkPort.ToString())
}
if ($NoConsole) {
    $argList.Add('--no-console')
}
if ($NoMap) {
    $argList.Add('--no-map')
}
if ($Execute) {
    $argList.Add('--execute')
}
else {
    $argList.Add('--dry-run')
}

$missionPlannerPath = Find-ExecutablePath -CandidatePaths @(
    (Join-Path $env:ProgramFiles 'Mission Planner\MissionPlanner.exe')
    (Join-Path ${env:ProgramFiles(x86)} 'Mission Planner\MissionPlanner.exe')
    (Join-Path $env:LOCALAPPDATA 'Mission Planner\MissionPlanner.exe')
)

$qgcPath = Find-ExecutablePath -CandidatePaths @(
    (Join-Path $env:ProgramFiles 'QGroundControl\QGroundControl.exe')
    (Join-Path ${env:ProgramFiles(x86)} 'QGroundControl\QGroundControl.exe')
    (Join-Path $env:LOCALAPPDATA 'QGroundControl\QGroundControl.exe')
    (Join-Path $env:LOCALAPPDATA 'Programs\QGroundControl\QGroundControl.exe')
)

$lines = New-Object System.Collections.Generic.List[string]

function Add-Line {
    param([string]$Text)
    $script:lines.Add($Text)
    Write-Host $Text
}

$state = [ordered]@{
    timestamp_utc              = (Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ssZ')
    stand_mode                 = 'internal'
    effective_host_ip          = $effectiveIp
    mavlink_udp_port           = $MavlinkPort
    mavlink_monitor_udp_port   = $SecondaryMavlinkPort
    wsl_process_id             = $null
    mission_planner_process_id = $null
    qgroundcontrol_process_id  = $null
    mission_planner_launched   = $false
    qgroundcontrol_launched    = $false
}

Add-Line 'Start ArduCopter internal SITL through WSL'
Add-Line ("  mode                           : {0}" -f $(if ($Execute) { 'execute' } else { 'check' }))
Add-Line ("  WSL distro                     : {0}" -f $DistroName)
Add-Line ("  requested IP                   : {0}" -f $Ip)
Add-Line ("  effective Windows host IP      : {0}" -f $effectiveIp)
Add-Line ("  primary MAVLink UDP port       : {0}" -f $MavlinkPort)
Add-Line ("  monitor MAVLink UDP port       : {0}" -f $SecondaryMavlinkPort)
Add-Line ("  WSL bash script                : {0}" -f $scriptPathWsl)

if ($Execute) {
    $wslArgs = @('-d', $DistroName, '--', 'bash') + $argList.ToArray()
    $wslProcess = Start-Process -FilePath 'wsl.exe' -ArgumentList $wslArgs -WindowStyle Hidden -PassThru
    $state.wsl_process_id = $wslProcess.Id
    Add-Line ("  WSL process id                 : {0}" -f $wslProcess.Id)

    Start-Sleep -Seconds 8
    $processLines = @(
        & wsl.exe -d $DistroName -- bash -lc "ps -eo pid,comm,args | grep 'arducopter --model +' | grep -v grep || true"
    ) | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }

    if ($processLines.Count -gt 0) {
        Add-Line '  SITL process confirmed         : yes'
        foreach ($line in $processLines) {
            Add-Line ("    {0}" -f $line)
        }
    }
    else {
        Add-Line '  SITL process confirmed         : no'
        Add-Line '  warning                        : arducopter process was not confirmed inside WSL after launch.'
    }

    if ($LaunchMissionPlanner -and $missionPlannerPath) {
        $existingMissionPlanner = Get-Process -Name MissionPlanner -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($existingMissionPlanner) {
            $state.mission_planner_process_id = $existingMissionPlanner.Id
            Add-Line ("  Mission Planner reused         : yes, PID {0}" -f $existingMissionPlanner.Id)
        }
        else {
            $mpProcess = Start-Process -FilePath $missionPlannerPath -PassThru
            $state.mission_planner_process_id = $mpProcess.Id
            $state.mission_planner_launched = $true
            Add-Line ("  Mission Planner started        : yes, PID {0}" -f $mpProcess.Id)
        }
    }
    elseif ($LaunchMissionPlanner) {
        Add-Line '  Mission Planner start skipped  : executable not found.'
    }

    if ($LaunchQGroundControl -and $qgcPath) {
        $existingQgc = Get-Process -Name QGroundControl -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($existingQgc) {
            $state.qgroundcontrol_process_id = $existingQgc.Id
            Add-Line ("  QGroundControl reused          : yes, PID {0}" -f $existingQgc.Id)
        }
        else {
            $qgcProcess = Start-Process -FilePath $qgcPath -PassThru
            $state.qgroundcontrol_process_id = $qgcProcess.Id
            $state.qgroundcontrol_launched = $true
            Add-Line ("  QGroundControl started         : yes, PID {0}" -f $qgcProcess.Id)
        }
    }
    elseif ($LaunchQGroundControl) {
        Add-Line '  QGroundControl start skipped   : executable not found.'
    }
}
else {
    Add-Line '  dry-run                        : команда старта не выполнялась.'
}

$stateJson = $state | ConvertTo-Json -Depth 4
Write-Utf8NoBomFile -Path $statePathValue -Text ($stateJson + [Environment]::NewLine)
Write-Utf8NoBomFile -Path $logPath -Text (($lines -join [Environment]::NewLine) + [Environment]::NewLine)

exit 0
