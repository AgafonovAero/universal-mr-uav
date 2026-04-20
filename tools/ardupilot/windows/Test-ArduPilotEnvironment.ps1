[CmdletBinding()]
param()

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

function Get-ToolPath {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Names
    )

    foreach ($name in $Names) {
        $command = Get-Command $name -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($null -ne $command) {
            return $command.Source
        }
    }

    return $null
}

function Get-WslDistroInfo {
    $result = [ordered]@{
        HasWslCommand = $false
        Distros       = @()
        Messages      = @()
    }

    $wslCommand = Get-Command wsl.exe -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -eq $wslCommand) {
        $result.Messages += 'wsl.exe command was not found.'
        return [pscustomobject]$result
    }

    $result.HasWslCommand = $true

    try {
        $output = & wsl.exe -l -v 2>&1
        foreach ($line in $output) {
            $text = ([string]$line).Replace([string][char]0, '')
            if ([string]::IsNullOrWhiteSpace($text)) {
                continue
            }

            if ($text -match 'NAME\s+STATE\s+VERSION') {
                continue
            }

            if ($text -match 'Windows Subsystem for Linux') {
                continue
            }

            $isDefault = $false
            $trimmed = $text.Trim()
            if ($trimmed.StartsWith('*')) {
                $isDefault = $true
                $trimmed = $trimmed.Substring(1).Trim()
            }

            if ($trimmed -match '^(?<Name>.+?)\s{2,}(?<State>\S+)\s+(?<Version>\d+)$') {
                $result.Distros += [pscustomobject]@{
                    Name      = $Matches.Name.Trim()
                    State     = $Matches.State
                    Version   = [int]$Matches.Version
                    IsDefault = $isDefault
                }
            }
        }

        if ($result.Distros.Count -eq 0) {
            $result.Messages += 'WSL was detected, but distro list was empty.'
        }
        else {
            $names = ($result.Distros | ForEach-Object { $_.Name }) -join ', '
            $result.Messages += "WSL was detected. Distros: $names"
        }
    }
    catch {
        $result.Messages += 'Failed to query WSL distros. No installed WSL distro was reported.'
    }

    return [pscustomobject]$result
}

function Test-WslTargetDistro {
    param(
        [Parameter(Mandatory = $true)]
        [string]$DistroName
    )

    try {
        $output = & wsl.exe -d $DistroName -- bash -lc "printf ok" 2>&1
        $text = (($output | ForEach-Object { ([string]$_).Replace([string][char]0, '') }) -join "")
        return $text.Contains('ok')
    }
    catch {
        return $false
    }
}

$repoRoot = Get-RepoRoot
$jsonReportPath = Join-Path $repoRoot 'artifacts/reports/task_13_windows_environment.json'
$pathHintFile = Join-Path $repoRoot 'tools/ardupilot/windows/ardupilot_root_wsl.txt'

$lines = New-Object System.Collections.Generic.List[string]

function Add-Line {
    param([string]$Text)
    $script:lines.Add($Text)
    Write-Host $Text
}

$wslInfo = Get-WslDistroInfo
$hasUbuntu = $false
if ($wslInfo.Distros.Count -gt 0) {
    $hasUbuntu = ($wslInfo.Distros | Where-Object { $_.Name -like 'Ubuntu*' }).Count -gt 0
}
if (-not $hasUbuntu -and $wslInfo.HasWslCommand) {
    $hasUbuntu = Test-WslTargetDistro -DistroName 'Ubuntu'
    if ($hasUbuntu) {
        $wslInfo.Messages += 'Ubuntu was confirmed by a direct WSL probe.'
    }
}

$gitPath = Get-ToolPath -Names @('git.exe', 'git')
$pythonPath = Get-ToolPath -Names @('python.exe', 'python')
$matlabPath = Get-ToolPath -Names @('matlab.exe', 'matlab')
$ghPath = Get-ToolPath -Names @('gh.exe', 'gh')

$pathHint = $null
foreach ($envName in 'ARDUPILOT_ROOT', 'ARDUPILOT_HOME', 'ARDUPILOT_DIR') {
    $envValue = [Environment]::GetEnvironmentVariable($envName)
    if (-not [string]::IsNullOrWhiteSpace($envValue)) {
        $pathHint = $envValue
        break
    }
}

if (-not $pathHint -and (Test-Path $pathHintFile)) {
    $pathHint = (Get-Content $pathHintFile -Raw).Trim()
}

$isReady = $wslInfo.HasWslCommand -and $hasUbuntu -and `
    (-not [string]::IsNullOrWhiteSpace($gitPath)) -and `
    (-not [string]::IsNullOrWhiteSpace($pythonPath))

$missingItems = New-Object System.Collections.Generic.List[string]
if (-not $wslInfo.HasWslCommand) {
    $missingItems.Add('WSL')
}
if (-not $hasUbuntu) {
    $missingItems.Add('Ubuntu distro in WSL')
}
if ([string]::IsNullOrWhiteSpace($gitPath)) {
    $missingItems.Add('Git on Windows')
}
if ([string]::IsNullOrWhiteSpace($pythonPath)) {
    $missingItems.Add('Python on Windows')
}
if ([string]::IsNullOrWhiteSpace($pathHint)) {
    $missingItems.Add('ArduPilot path hint inside WSL')
}

$report = [ordered]@{
    timestamp_utc            = (Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ssZ')
    windows_version          = [System.Environment]::OSVersion.VersionString
    has_wsl_command          = $wslInfo.HasWslCommand
    wsl_distros              = $wslInfo.Distros
    has_ubuntu               = $hasUbuntu
    has_git                  = -not [string]::IsNullOrWhiteSpace($gitPath)
    git_path                 = $gitPath
    has_python               = -not [string]::IsNullOrWhiteSpace($pythonPath)
    python_path              = $pythonPath
    has_matlab               = -not [string]::IsNullOrWhiteSpace($matlabPath)
    matlab_path              = $matlabPath
    has_github_cli           = -not [string]::IsNullOrWhiteSpace($ghPath)
    github_cli_path          = $ghPath
    ardupilot_path_hint      = $pathHint
    ardupilot_path_hint_file = $(if (Test-Path $pathHintFile) { $pathHintFile } else { $null })
    missing_items            = $missingItems.ToArray()
    is_ready                 = $isReady
    messages                 = $wslInfo.Messages
}

$jsonText = $report | ConvertTo-Json -Depth 6
Write-Utf8NoBomFile -Path $jsonReportPath -Text ($jsonText + [Environment]::NewLine)

Add-Line 'Windows and WSL environment check for ArduPilot SITL'
Add-Line ("  Windows version                      : {0}" -f $report.windows_version)
Add-Line ("  wsl.exe available                    : {0}" -f $(if ($report.has_wsl_command) { 'yes' } else { 'no' }))
Add-Line ("  Ubuntu distro present                : {0}" -f $(if ($report.has_ubuntu) { 'yes' } else { 'no' }))
Add-Line ("  Git on Windows                       : {0}" -f $(if ($report.has_git) { 'yes' } else { 'no' }))
Add-Line ("  Python on Windows                    : {0}" -f $(if ($report.has_python) { 'yes' } else { 'no' }))
Add-Line ("  MATLAB available                     : {0}" -f $(if ($report.has_matlab) { 'yes' } else { 'no' }))
Add-Line ("  GitHub CLI available                 : {0}" -f $(if ($report.has_github_cli) { 'yes' } else { 'no' }))
Add-Line ("  ArduPilot path hint                  : {0}" -f $(if ($report.ardupilot_path_hint) { $report.ardupilot_path_hint } else { '<not set>' }))
Add-Line ("  overall readiness                    : {0}" -f $(if ($report.is_ready) { 'yes' } else { 'no' }))

Add-Line ''
Add-Line 'WSL distros'
if ($report.wsl_distros.Count -eq 0) {
    Add-Line '  no distros were detected.'
}
else {
    foreach ($distro in $report.wsl_distros) {
        Add-Line ("  - {0}, state: {1}, WSL version: {2}, default: {3}" -f `
            $distro.Name, $distro.State, $distro.Version, $(if ($distro.IsDefault) { 'yes' } else { 'no' }))
    }
}

Add-Line ''
Add-Line 'Missing items'
if ($report.missing_items.Count -eq 0) {
    Add-Line '  no missing items were detected.'
}
else {
    foreach ($item in $report.missing_items) {
        Add-Line ("  - {0}" -f $item)
    }
}

Add-Line ''
Add-Line ("JSON report was written to {0}" -f $jsonReportPath)

exit 0
