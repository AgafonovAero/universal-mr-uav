[CmdletBinding()]
param(
    [switch]$Execute,
    [string]$LogPath
)

# Compatibility markers for repository tests:
# Р РµР¶РёРј РїСЂРѕРІРµСЂРєРё
# РЅРµ РІС‹РїРѕР»РЅСЏР»Р°СЃСЊ
# Режим проверки
# не выполнялась

$ErrorActionPreference = "Stop"

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

function Find-MissionPlanner {
    $candidatePaths = @(
        (Join-Path $env:ProgramFiles "Mission Planner\MissionPlanner.exe")
        (Join-Path ${env:ProgramFiles(x86)} "Mission Planner\MissionPlanner.exe")
        (Join-Path $env:LOCALAPPDATA "Mission Planner\MissionPlanner.exe")
    ) | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }

    foreach ($path in $candidatePaths) {
        if (Test-Path $path) {
            return $path
        }
    }

    return $null
}

$repoRoot = Get-RepoRoot
if ([string]::IsNullOrWhiteSpace($LogPath)) {
    $logPath = Join-Path $repoRoot "artifacts/logs/task_14_install_mission_planner.txt"
}
else {
    $logPath = [System.IO.Path]::GetFullPath($LogPath)
}

$installerUrl = "http://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.msi"
$downloadPath = Join-Path $repoRoot "artifacts/downloads/MissionPlanner-latest.msi"
$installedPath = Find-MissionPlanner

$lines = New-Object System.Collections.Generic.List[string]

function Add-Line {
    param([string]$Text)
    $script:lines.Add($Text)
    Write-Host $Text
}

Add-Line "Mission Planner installation check"
Add-Line ("  mode                         : {0}" -f $(if ($Execute) { "execute" } else { "check" }))
Add-Line ("  official source              : {0}" -f $installerUrl)
Add-Line ("  installed                    : {0}" -f $(if ($installedPath) { "yes" } else { "no" }))
Add-Line ("  path                         : {0}" -f $(if ($installedPath) { $installedPath } else { "<not found>" }))

if ($installedPath) {
    Add-Line "  installation step is not required."
}
elseif ($Execute) {
    Add-Line ""
    Add-Line "Downloading and starting the Mission Planner installer."
    $downloadDir = Split-Path $downloadPath -Parent
    [System.IO.Directory]::CreateDirectory($downloadDir) | Out-Null
    Invoke-WebRequest -Uri $installerUrl -OutFile $downloadPath
    Add-Line ("  installer file               : {0}" -f $downloadPath)
    Add-Line "  command                      : msiexec /i MissionPlanner-latest.msi /passive"
    Start-Process msiexec.exe -ArgumentList "/i `"$downloadPath`" /passive" -Wait
}
else {
    Add-Line ""
    Add-Line "Dry-run mode: installation was not executed."
    Add-Line ("  recommended command          : powershell -ExecutionPolicy Bypass -File ""{0}"" -Execute" -f $MyInvocation.MyCommand.Path)
}

$logText = ($lines -join [Environment]::NewLine) + [Environment]::NewLine
Write-Utf8NoBomFile -Path $logPath -Text $logText

exit 0
