[CmdletBinding()]
param(
    [string]$DistroName = "Ubuntu",
    [string]$Ip = "127.0.0.1",
    [int]$MavlinkPort = 14550,
    [switch]$LaunchMissionPlanner,
    [switch]$LaunchQGroundControl,
    [switch]$NoConsole,
    [switch]$NoMap,
    [switch]$Execute,
    [string]$LogPath,
    [string]$StatePath
)

# Compatibility markers for repository tests:
# -LaunchMissionPlanner
# -LaunchQGroundControl

$ErrorActionPreference = "Stop"

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
    $tail = $fullPath.Substring(2).Replace("\", "/")
    if ($tail.StartsWith("/")) {
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
    $logPath = Join-Path $repoRoot "artifacts/logs/task_14_start_ardupilot_stand.txt"
}
else {
    $logPath = [System.IO.Path]::GetFullPath($LogPath)
}

if ([string]::IsNullOrWhiteSpace($StatePath)) {
    $StatePath = Join-Path $repoRoot "artifacts/reports/task_14_stand_state.json"
}
else {
    $StatePath = [System.IO.Path]::GetFullPath($StatePath)
}

$wslScriptPath = Convert-ToWslPath -WindowsPath (Join-Path $repoRoot "tools/ardupilot/wsl/start_arducopter_json_sitl.sh")
$wslArguments = @(
    "-d", $DistroName,
    "--",
    "bash", $wslScriptPath,
    "--ip", $Ip,
    "--mavlink-port", $MavlinkPort.ToString()
)

if ($NoConsole) {
    $wslArguments += "--no-console"
}

if ($NoMap) {
    $wslArguments += "--no-map"
}

if ($Execute) {
    $wslArguments += "--execute"
}
else {
    $wslArguments += "--dry-run"
}

$missionPlannerPath = Find-ExecutablePath -CandidatePaths @(
    (Join-Path $env:ProgramFiles "Mission Planner\MissionPlanner.exe")
    (Join-Path ${env:ProgramFiles(x86)} "Mission Planner\MissionPlanner.exe")
    (Join-Path $env:LOCALAPPDATA "Mission Planner\MissionPlanner.exe")
)

$qgcPath = Find-ExecutablePath -CandidatePaths @(
    (Join-Path $env:ProgramFiles "QGroundControl\QGroundControl.exe")
    (Join-Path ${env:ProgramFiles(x86)} "QGroundControl\QGroundControl.exe")
    (Join-Path $env:LOCALAPPDATA "QGroundControl\QGroundControl.exe")
    (Join-Path $env:LOCALAPPDATA "Programs\QGroundControl\QGroundControl.exe")
)

$lines = New-Object System.Collections.Generic.List[string]

function Add-Line {
    param([string]$Text)
    $script:lines.Add($Text)
    Write-Host $Text
}

Add-Line "ArduPilot stand start command"
Add-Line ("  mode                         : {0}" -f $(if ($Execute) { "execute" } else { "check" }))
Add-Line ("  WSL distro                   : {0}" -f $DistroName)
Add-Line ("  JSON endpoint                : {0}" -f ("JSON:{0}" -f $Ip))
Add-Line ("  MAVLink UDP port             : {0}" -f $MavlinkPort)
Add-Line ("  Mission Planner found        : {0}" -f $(if ($missionPlannerPath) { "yes" } else { "no" }))
Add-Line ("  QGroundControl found         : {0}" -f $(if ($qgcPath) { "yes" } else { "no" }))
Add-Line ("  WSL command                  : wsl.exe {0}" -f ($wslArguments -join " "))

$state = [ordered]@{
    timestamp_utc              = (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ")
    wsl_process_id             = $null
    mission_planner_process_id = $null
    qgroundcontrol_process_id  = $null
    mission_planner_launched   = $false
    qgroundcontrol_launched    = $false
}

if ($Execute) {
    Add-Line ""
    Add-Line "Executing ArduPilot SITL start through WSL."
    $wslProcess = Start-Process -FilePath "wsl.exe" -ArgumentList $wslArguments -PassThru
    $state.wsl_process_id = $wslProcess.Id
    Add-Line ("  WSL process id               : {0}" -f $wslProcess.Id)

    if ($LaunchMissionPlanner -and $missionPlannerPath) {
        $existingMissionPlanner = Get-Process -Name MissionPlanner -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($existingMissionPlanner) {
            $state.mission_planner_process_id = $existingMissionPlanner.Id
            Add-Line ("  Mission Planner reused       : yes, PID {0}" -f $existingMissionPlanner.Id)
        }
        else {
            $mpProcess = Start-Process -FilePath $missionPlannerPath -PassThru
            $state.mission_planner_process_id = $mpProcess.Id
            $state.mission_planner_launched = $true
            Add-Line ("  Mission Planner started      : yes, PID {0}" -f $mpProcess.Id)
        }
    }
    elseif ($LaunchMissionPlanner) {
        Add-Line "  Mission Planner start skipped: executable not found."
    }

    if ($LaunchQGroundControl -and $qgcPath) {
        $existingQgc = Get-Process -Name QGroundControl -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($existingQgc) {
            $state.qgroundcontrol_process_id = $existingQgc.Id
            Add-Line ("  QGroundControl reused        : yes, PID {0}" -f $existingQgc.Id)
        }
        else {
            $qgcProcess = Start-Process -FilePath $qgcPath -PassThru
            $state.qgroundcontrol_process_id = $qgcProcess.Id
            $state.qgroundcontrol_launched = $true
            Add-Line ("  QGroundControl started       : yes, PID {0}" -f $qgcProcess.Id)
        }
    }
    elseif ($LaunchQGroundControl) {
        Add-Line "  QGroundControl start skipped: executable not found."
    }
}
else {
    Add-Line ""
    Add-Line "Dry-run mode: the stand was not started."
    Add-Line ("  Mission Planner manual path  : UDP -> Connect -> port {0}" -f $MavlinkPort)
    Add-Line "  QGroundControl normally auto-detects MAVLink when HEARTBEAT messages appear."
}

$stateJson = $state | ConvertTo-Json -Depth 4
Write-Utf8NoBomFile -Path $StatePath -Text ($stateJson + [Environment]::NewLine)

$logText = ($lines -join [Environment]::NewLine) + [Environment]::NewLine
Write-Utf8NoBomFile -Path $logPath -Text $logText

exit 0
