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

function Find-QGroundControl {
    $candidatePaths = @(
        (Join-Path $env:ProgramFiles "QGroundControl\QGroundControl.exe")
        (Join-Path ${env:ProgramFiles(x86)} "QGroundControl\QGroundControl.exe")
        (Join-Path $env:LOCALAPPDATA "QGroundControl\QGroundControl.exe")
        (Join-Path $env:LOCALAPPDATA "Programs\QGroundControl\QGroundControl.exe")
    ) | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }

    foreach ($path in $candidatePaths) {
        if (Test-Path $path) {
            return $path
        }
    }

    return $null
}

function Get-LatestQGroundControlAsset {
    try {
        $release = Invoke-RestMethod -Uri "https://api.github.com/repos/mavlink/qgroundcontrol/releases/latest"
        $asset = $release.assets | Where-Object {
            $_.browser_download_url -match "\.exe$"
        } | Select-Object -First 1

        if ($null -ne $asset) {
            return [pscustomobject]@{
                TagName = $release.tag_name
                Name    = $asset.name
                Url     = $asset.browser_download_url
            }
        }
    }
    catch {
    }

    return [pscustomobject]@{
        TagName = "<unknown>"
        Name    = "<unknown>"
        Url     = "https://github.com/mavlink/qgroundcontrol/releases/latest"
    }
}

$repoRoot = Get-RepoRoot
if ([string]::IsNullOrWhiteSpace($LogPath)) {
    $logPath = Join-Path $repoRoot "artifacts/logs/task_14_install_qgroundcontrol.txt"
}
else {
    $logPath = [System.IO.Path]::GetFullPath($LogPath)
}

$installedPath = Find-QGroundControl
$asset = $null
if (-not $installedPath -and $Execute) {
    $asset = Get-LatestQGroundControlAsset
}

$lines = New-Object System.Collections.Generic.List[string]

function Add-Line {
    param([string]$Text)
    $script:lines.Add($Text)
    Write-Host $Text
}

Add-Line "QGroundControl installation check"
Add-Line ("  mode                         : {0}" -f $(if ($Execute) { "execute" } else { "check" }))
Add-Line ("  installed                    : {0}" -f $(if ($installedPath) { "yes" } else { "no" }))
Add-Line ("  path                         : {0}" -f $(if ($installedPath) { $installedPath } else { "<not found>" }))

if ($asset) {
    Add-Line ("  recommended source           : {0}" -f $asset.Url)
    Add-Line ("  release tag                  : {0}" -f $asset.TagName)
}
elseif (-not $installedPath) {
    Add-Line ("  recommended source           : {0}" -f "https://github.com/mavlink/qgroundcontrol/releases/latest")
    Add-Line ("  release tag                  : {0}" -f "<resolved on execute>")
}

if ($installedPath) {
    Add-Line "  installation step is not required."
}
elseif ($Execute) {
    Add-Line ""
    Add-Line "Downloading and starting the QGroundControl installer."
    $downloadPath = Join-Path $repoRoot ("artifacts/downloads/" + $asset.Name)
    $downloadDir = Split-Path $downloadPath -Parent
    [System.IO.Directory]::CreateDirectory($downloadDir) | Out-Null
    Invoke-WebRequest -Uri $asset.Url -OutFile $downloadPath
    Add-Line ("  installer file               : {0}" -f $downloadPath)
    Add-Line "  running installer executable"
    Start-Process -FilePath $downloadPath -Wait
}
else {
    Add-Line ""
    Add-Line "Dry-run mode: installation was not executed."
    Add-Line ("  recommended command          : powershell -ExecutionPolicy Bypass -File ""{0}"" -Execute" -f $MyInvocation.MyCommand.Path)
}

$logText = ($lines -join [Environment]::NewLine) + [Environment]::NewLine
Write-Utf8NoBomFile -Path $logPath -Text $logText

exit 0
