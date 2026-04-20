[CmdletBinding()]
param(
    [switch]$Execute,
    [switch]$AllowGroundStationClose,
    [string]$StatePath,
    [string]$LogPath
)

$ErrorActionPreference = 'Stop'

function Get-RepoRoot {
    return (Split-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) -Parent)
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

$repoRoot = Get-RepoRoot
if ([string]::IsNullOrWhiteSpace($LogPath)) {
    $logPath = Join-Path $repoRoot 'artifacts/logs/task_14_stop_ardupilot_stand.txt'
}
else {
    $logPath = [System.IO.Path]::GetFullPath($LogPath)
}

if ([string]::IsNullOrWhiteSpace($StatePath)) {
    $StatePath = Join-Path $repoRoot 'artifacts/reports/task_14_stand_state.json'
}
else {
    $StatePath = [System.IO.Path]::GetFullPath($StatePath)
}

$state = $null
if (Test-Path $StatePath) {
    $state = Get-Content $StatePath -Raw | ConvertFrom-Json
}

$lines = New-Object System.Collections.Generic.List[string]

function Add-Line {
    param([string]$Text)
    $script:lines.Add($Text)
    Write-Host $Text
}

$candidatePids = New-Object System.Collections.Generic.List[int]
if ($state -and $state.wsl_process_id) {
    $candidatePids.Add([int]$state.wsl_process_id)
}

Add-Line 'Stop ArduPilot SITL stand'
Add-Line ("  execution mode                      : {0}" -f $(if ($Execute) { 'execute' } else { 'check' }))
Add-Line ("  state file                          : {0}" -f $StatePath)

if ($candidatePids.Count -eq 0) {
    Add-Line '  no stop candidates were found.'
}
else {
    Add-Line '  stop candidates:'
    foreach ($pidValue in $candidatePids) {
        Add-Line ("    PID {0}" -f $pidValue)
    }
}

if ($Execute) {
    foreach ($pidValue in $candidatePids) {
        try {
            Stop-Process -Id $pidValue -Force -ErrorAction Stop
            Add-Line ("  process stopped                    : PID {0}" -f $pidValue)
        }
        catch {
            Add-Line ("  process was not stopped            : PID {0}" -f $pidValue)
        }
    }

    if ($AllowGroundStationClose -and $state) {
        foreach ($property in 'mission_planner_process_id', 'qgroundcontrol_process_id') {
            $pidValue = $state.$property
            if ($pidValue) {
                try {
                    Stop-Process -Id ([int]$pidValue) -Force -ErrorAction Stop
                    Add-Line ("  ground station stopped            : PID {0}" -f $pidValue)
                }
                catch {
                    Add-Line ("  ground station was not stopped    : PID {0}" -f $pidValue)
                }
            }
        }
    }
}
else {
    Add-Line '  dry-run: no processes were terminated.'
}

$logText = ($lines -join [Environment]::NewLine) + [Environment]::NewLine
Write-Utf8NoBomFile -Path $logPath -Text $logText

exit 0
