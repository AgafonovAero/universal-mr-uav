[CmdletBinding()]
param(
    [string]$DistroName = 'Ubuntu',
    [switch]$NoConsole,
    [switch]$NoMap,
    [string]$Ip = '127.0.0.1',
    [int]$MavlinkPort = 14550,
    [int]$SecondaryMavlinkPort = 14552,
    [string]$ExtraDefaultsParmPath = '',
    [switch]$Execute,
    [string]$LogPath
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

$repoRoot = Get-RepoRoot
if ([string]::IsNullOrWhiteSpace($LogPath)) {
    $logPath = Join-Path $repoRoot 'artifacts/logs/task_13_start_ardupilot_json_sitl.txt'
}
else {
    $logPath = [System.IO.Path]::GetFullPath($LogPath)
}

$scriptPath = Join-Path $repoRoot 'tools/ardupilot/wsl/start_arducopter_json_sitl.sh'
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

if (-not [string]::IsNullOrWhiteSpace($ExtraDefaultsParmPath)) {
    $resolvedExtraDefaults = [System.IO.Path]::GetFullPath($ExtraDefaultsParmPath)
    $argList.Add('--defaults-extra')
    $argList.Add((Convert-ToWslPath -WindowsPath $resolvedExtraDefaults))
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

$displayArgs = ($argList.ToArray() -join ' ')
$lines = New-Object System.Collections.Generic.List[string]

function Add-Line {
    param([string]$Text)

    $script:lines.Add($Text)
    Write-Host $Text
}

Add-Line 'Start ArduCopter JSON SITL through WSL'
Add-Line ("  distro                         : {0}" -f $DistroName)
Add-Line ("  bash script                    : {0}" -f $scriptPathWsl)
Add-Line ("  mode                           : {0}" -f $(if ($Execute) { 'execute' } else { 'check' }))
Add-Line ("  requested IP                   : {0}" -f $Ip)
Add-Line ("  effective Windows host IP      : {0}" -f $effectiveIp)
Add-Line ("  MAVLink UDP port               : {0}" -f $MavlinkPort)
if ($SecondaryMavlinkPort -gt 0) {
    Add-Line ("  дополнительный MAVLink порт   : {0}" -f $SecondaryMavlinkPort)
}
if (-not [string]::IsNullOrWhiteSpace($ExtraDefaultsParmPath)) {
    Add-Line ("  дополнительный defaults parm   : {0}" -f $ExtraDefaultsParmPath)
}
Add-Line ("  arguments                      : {0}" -f $displayArgs)

if ($Execute) {
    Add-Line ''
    Add-Line 'Starting ArduCopter SITL in non-blocking mode.'

    $wslArgs = @('-d', $DistroName, '--', 'bash') + $argList.ToArray()
    $wslProcess = Start-Process -FilePath 'wsl.exe' -ArgumentList $wslArgs -WindowStyle Hidden -PassThru
    Add-Line ("  wsl.exe process id             : {0}" -f $wslProcess.Id)

    Start-Sleep -Seconds 6

    $bashCheck = 'ps -eo pid,comm,args | grep arducopter | grep -v grep || true'
    $processLines = @(
        & wsl.exe -d $DistroName -- bash -lc $bashCheck
    ) | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }

    if ($processLines.Count -gt 0) {
        Add-Line '  SITL process confirmed         : yes'
        Add-Line '  detected WSL processes:'
        foreach ($line in $processLines) {
            Add-Line ("    {0}" -f $line)
        }
    }
    else {
        Add-Line '  SITL process confirmed         : no'
        Add-Line '  warning                        : arducopter was not confirmed inside WSL after launch.'
    }
}
else {
    Add-Line ''
    Add-Line 'Dry-run mode: launch command was not executed.'
    Add-Line ("  manual command                 : wsl.exe -d ""{0}"" -- bash {1}" -f $DistroName, $displayArgs)
}

$logText = ($lines -join [Environment]::NewLine) + [Environment]::NewLine
Write-Utf8NoBomFile -Path $logPath -Text $logText

exit 0
