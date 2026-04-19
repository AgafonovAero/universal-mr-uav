[CmdletBinding()]
param(
    [string]$DistroName = 'Ubuntu',
    [switch]$Execute
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

$repoRoot = Get-RepoRoot
$logPath = Join-Path $repoRoot 'artifacts/logs/task_13_invoke_ardupilot_wsl_setup.txt'
$scriptPath = Join-Path $repoRoot 'tools/ardupilot/wsl/setup_ardupilot_wsl.sh'
$scriptPathWsl = Convert-ToWslPath -WindowsPath $scriptPath
$wslCommand = 'wsl.exe -d "{0}" -- bash "{1}"' -f $DistroName, $scriptPathWsl

$lines = New-Object System.Collections.Generic.List[string]

function Add-Line {
    param([string]$Text)
    $script:lines.Add($Text)
    Write-Host $Text
}

Add-Line 'ArduPilot preparation inside WSL'
Add-Line ("  distro                               : {0}" -f $DistroName)
Add-Line ("  Bash script                          : {0}" -f $scriptPathWsl)
Add-Line ("  execution mode                       : {0}" -f $(if ($Execute) { 'execute' } else { 'check' }))
Add-Line ("  WSL command                          : {0}" -f $wslCommand)

if ($Execute) {
    Add-Line ''
    Add-Line 'Executing WSL preparation command:'
    & wsl.exe -d $DistroName -- bash $scriptPathWsl 2>&1 | ForEach-Object {
        Add-Line ("  {0}" -f $_)
    }
}
else {
    Add-Line ''
    Add-Line 'Check mode: Bash preparation script was not executed.'
}

$logText = ($lines -join [Environment]::NewLine) + [Environment]::NewLine
Write-Utf8NoBomFile -Path $logPath -Text $logText

exit 0
