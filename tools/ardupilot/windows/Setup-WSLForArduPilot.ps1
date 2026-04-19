[CmdletBinding()]
param(
    [switch]$Execute,
    [string]$DistroName = "Ubuntu",
    [int]$PreferWSLVersion = 2,
    [string]$LogPath
)

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

function Test-IsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Get-WslDistroInfo {
    $result = [ordered]@{
        HasWslCommand = $false
        Distros       = @()
    }

    $wslCommand = Get-Command wsl.exe -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -eq $wslCommand) {
        return [pscustomobject]$result
    }

    $result.HasWslCommand = $true

    try {
        $output = & wsl.exe -l -v 2>&1
        foreach ($line in $output) {
            $text = [string]$line
            if ([string]::IsNullOrWhiteSpace($text)) {
                continue
            }

            if ($text -match "NAME\s+STATE\s+VERSION") {
                continue
            }

            if ($text -match "Windows Subsystem for Linux") {
                continue
            }

            $trimmed = $text.Trim()
            $isDefault = $false
            if ($trimmed.StartsWith("*")) {
                $isDefault = $true
                $trimmed = $trimmed.Substring(1).Trim()
            }

            if ($trimmed -match "^(?<Name>.+?)\s{2,}(?<State>\S+)\s+(?<Version>\d+)$") {
                $result.Distros += [pscustomobject]@{
                    Name      = $Matches.Name.Trim()
                    State     = $Matches.State
                    Version   = [int]$Matches.Version
                    IsDefault = $isDefault
                }
            }
        }
    }
    catch {
    }

    return [pscustomobject]$result
}

$repoRoot = Get-RepoRoot
if ([string]::IsNullOrWhiteSpace($LogPath)) {
    $logPath = Join-Path $repoRoot "artifacts/logs/task_13_setup_wsl_for_ardupilot.txt"
}
else {
    $logPath = [System.IO.Path]::GetFullPath($LogPath)
}

$lines = New-Object System.Collections.Generic.List[string]

function Add-Line {
    param([string]$Text)
    $script:lines.Add($Text)
    Write-Host $Text
}

$isAdmin = Test-IsAdministrator
$wslInfo = Get-WslDistroInfo
$targetDistro = $wslInfo.Distros | Where-Object { $_.Name -eq $DistroName } | Select-Object -First 1
$commands = New-Object System.Collections.Generic.List[string]

if (-not $wslInfo.HasWslCommand) {
    $commands.Add("wsl --install -d $DistroName")
}
elseif ($null -eq $targetDistro) {
    $commands.Add("wsl --install -d $DistroName")
}
elseif ($targetDistro.Version -ne $PreferWSLVersion) {
    $commands.Add("wsl --set-version $DistroName $PreferWSLVersion")
}

Add-Line "WSL setup check for ArduPilot SITL"
Add-Line ("  mode                         : {0}" -f $(if ($Execute) { "execute" } else { "check" }))
Add-Line ("  target distro                : {0}" -f $DistroName)
Add-Line ("  preferred WSL version        : {0}" -f $PreferWSLVersion)
Add-Line ("  administrator rights         : {0}" -f $(if ($isAdmin) { "yes" } else { "no" }))
Add-Line ("  wsl.exe available            : {0}" -f $(if ($wslInfo.HasWslCommand) { "yes" } else { "no" }))

if ($wslInfo.Distros.Count -gt 0) {
    Add-Line "  detected distros:"
    foreach ($distro in $wslInfo.Distros) {
        Add-Line ("    - {0}, version {1}, state {2}" -f $distro.Name, $distro.Version, $distro.State)
    }
}
else {
    Add-Line "  detected distros             : none"
}

if ($commands.Count -eq 0) {
    Add-Line "  follow-up commands           : not required"
}
else {
    Add-Line "  recommended commands:"
    foreach ($commandText in $commands) {
        Add-Line ("    {0}" -f $commandText)
    }
}

if ($Execute) {
    if (-not $isAdmin -and $commands.Count -gt 0) {
        Add-Line ""
        Add-Line "Administrator rights are required for the requested WSL changes."
        $selfCommand = @(
            "-ExecutionPolicy Bypass"
            ("-File ""{0}""" -f $MyInvocation.MyCommand.Path)
            "-Execute"
            ("-DistroName ""{0}""" -f $DistroName)
            ("-PreferWSLVersion {0}" -f $PreferWSLVersion)
        ) -join " "
        Add-Line ("Suggested elevated command: Start-Process powershell -Verb RunAs -ArgumentList '{0}'" -f $selfCommand)
    }
    elseif ($commands.Count -gt 0) {
        Add-Line ""
        Add-Line "Executing approved WSL preparation commands:"
        foreach ($commandText in $commands) {
            Add-Line ("  > {0}" -f $commandText)
            cmd /c $commandText | ForEach-Object {
                Add-Line ("    {0}" -f $_)
            }
        }
    }
    else {
        Add-Line ""
        Add-Line "No additional WSL preparation command is required."
    }
}
else {
    Add-Line ""
    Add-Line "Dry-run mode: no system changes were applied."
}

$logText = ($lines -join [Environment]::NewLine) + [Environment]::NewLine
Write-Utf8NoBomFile -Path $logPath -Text $logText

exit 0
