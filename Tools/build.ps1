param(
    [Parameter(Mandatory=$false, Position=0)]
    [string]$KspPath = "",

    [Parameter(Mandatory=$false)]
    [string]$OutputDir = "",

    [Parameter(Mandatory=$false)]
    [string]$InstallGameDataPath = "",

    [Parameter(Mandatory=$false)]
    [switch]$NoInstall,

    [Parameter(Mandatory=$false)]
    [switch]$NoZip
)

$ErrorActionPreference = "Stop"

function Get-SteamLibraryPaths {
    $paths = New-Object System.Collections.Generic.List[string]
    $registryKeys = @(
        "HKCU:\Software\Valve\Steam",
        "HKLM:\SOFTWARE\WOW6432Node\Valve\Steam",
        "HKLM:\SOFTWARE\Valve\Steam"
    )

    foreach ($key in $registryKeys) {
        try {
            $props = Get-ItemProperty -Path $key -ErrorAction Stop
            $installPath = $props.SteamPath
            if (-not $installPath) {
                $installPath = $props.InstallPath
            }
            if ($installPath -and (Test-Path $installPath)) {
                [void]$paths.Add($installPath)
            }
        } catch {
        }
    }

    $defaultSteam = "C:\Program Files (x86)\Steam"
    if (Test-Path $defaultSteam) {
        [void]$paths.Add($defaultSteam)
    }

    $uniqueSteamPaths = $paths | Select-Object -Unique
    foreach ($steamPath in $uniqueSteamPaths) {
        $libraryFile = Join-Path $steamPath "steamapps\libraryfolders.vdf"
        if (Test-Path $libraryFile) {
            $content = Get-Content -Raw -LiteralPath $libraryFile
            foreach ($match in [regex]::Matches($content, '"path"\s+"([^"]+)"')) {
                $libraryPath = $match.Groups[1].Value -replace "\\\\", "\"
                if (Test-Path $libraryPath) {
                    [void]$paths.Add($libraryPath)
                }
            }
        }
    }

    return $paths | Select-Object -Unique
}

function Find-KspPath {
    param([string]$RequestedPath)

    $candidates = New-Object System.Collections.Generic.List[string]
    if ($RequestedPath) {
        [void]$candidates.Add($RequestedPath)
    }
    if ($env:KSPDIR) {
        [void]$candidates.Add($env:KSPDIR)
    }

    foreach ($libraryPath in Get-SteamLibraryPaths) {
        $commonPath = Join-Path $libraryPath "steamapps\common"
        if (Test-Path $commonPath) {
            Get-ChildItem -LiteralPath $commonPath -Directory -Filter "Kerbal Space Program*test" -ErrorAction SilentlyContinue |
                ForEach-Object { [void]$candidates.Add($_.FullName) }
        }
        [void]$candidates.Add((Join-Path $libraryPath "steamapps\common\Kerbal Space Program"))
    }

    [void]$candidates.Add("C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program")
    [void]$candidates.Add("C:\Program Files\Steam\steamapps\common\Kerbal Space Program")
    [void]$candidates.Add("D:\SteamLibrary\steamapps\common\Kerbal Space Program")
    [void]$candidates.Add("D:\Games\Kerbal Space Program")

    foreach ($candidate in ($candidates | Select-Object -Unique)) {
        if (-not $candidate) {
            continue
        }

        $managed64 = Join-Path $candidate "KSP_x64_Data\Managed"
        $managed32 = Join-Path $candidate "KSP_Data\Managed"
        if (Test-Path (Join-Path $managed64 "Assembly-CSharp.dll")) {
            return @{
                KspPath = (Resolve-Path -LiteralPath $candidate).Path
                ManagedPath = (Resolve-Path -LiteralPath $managed64).Path
            }
        }
        if (Test-Path (Join-Path $managed32 "Assembly-CSharp.dll")) {
            return @{
                KspPath = (Resolve-Path -LiteralPath $candidate).Path
                ManagedPath = (Resolve-Path -LiteralPath $managed32).Path
            }
        }
    }

    return $null
}

function Get-MSBuildCommand {
    $msbuild = Get-Command msbuild.exe -ErrorAction SilentlyContinue
    if ($msbuild) {
        return $msbuild.Source
    }

    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $installPath = & $vswhere -latest -requires Microsoft.Component.MSBuild -property installationPath
        if ($installPath) {
            $candidate = Join-Path $installPath "MSBuild\Current\Bin\MSBuild.exe"
            if (Test-Path $candidate) {
                return $candidate
            }
        }
    }

    return "dotnet"
}

function Get-RoslynCompiler {
    $sdkRoot = Join-Path $env:ProgramFiles "dotnet\sdk"
    if (-not (Test-Path $sdkRoot)) {
        return $null
    }

    $candidates = Get-ChildItem -Path $sdkRoot -Filter csc.dll -Recurse -ErrorAction SilentlyContinue |
        Where-Object { $_.FullName -like "*\Roslyn\bincore\csc.dll" } |
        Sort-Object FullName -Descending

    if ($candidates) {
        return $candidates[0].FullName
    }

    return $null
}

function Invoke-DirectBuild {
    param(
        [string]$CompilerPath,
        [string]$ManagedPath,
        [string]$OutputDll,
        [string]$SourceFile
    )

    if (-not $CompilerPath) {
        return $false
    }

    $referenceNames = New-Object System.Collections.Generic.List[string]
    foreach ($name in @(
        "mscorlib.dll",
        "System.dll",
        "System.Core.dll",
        "Assembly-CSharp.dll",
        "Assembly-CSharp-firstpass.dll"
    )) {
        [void]$referenceNames.Add($name)
    }

    Get-ChildItem -LiteralPath $ManagedPath -Filter "UnityEngine*.dll" |
        ForEach-Object { [void]$referenceNames.Add($_.Name) }

    $referenceArgs = New-Object System.Collections.Generic.List[string]
    foreach ($referenceName in ($referenceNames | Select-Object -Unique)) {
        $referencePath = Join-Path $ManagedPath $referenceName
        if (Test-Path $referencePath) {
            [void]$referenceArgs.Add("/reference:$referencePath")
        }
    }

    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $OutputDll) | Out-Null

    & dotnet $CompilerPath `
        /nologo `
        /target:library `
        /optimize+ `
        /deterministic+ `
        /langversion:7.3 `
        /nostdlib+ `
        /out:$OutputDll `
        $referenceArgs `
        $SourceFile

    if ($LASTEXITCODE -ne 0) {
        throw "Direct compiler failed with exit code $LASTEXITCODE."
    }

    return $true
}

$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$project = Join-Path $root "Source\AutoTransferWindowPlanner.csproj"
$modName = "AutoTransferWindowPlanner"
$modGameData = Join-Path $root "GameData\$modName"
$pluginDll = Join-Path $modGameData "Plugins\$modName.dll"
$versionFile = Join-Path $modGameData "$modName.version"
$versionData = Get-Content -Raw -LiteralPath $versionFile | ConvertFrom-Json
$modVersion = "$($versionData.VERSION.MAJOR).$($versionData.VERSION.MINOR).$($versionData.VERSION.PATCH)"

if (-not $OutputDir) {
    $OutputDir = Join-Path $root "dist"
}

$ksp = Find-KspPath -RequestedPath $KspPath
if (-not $ksp) {
    Write-Host "KSP installation was not found." -ForegroundColor Yellow
    Write-Host "Run from the project root with your game folder path:"
    Write-Host '  build.bat "D:\Games\Kerbal Space Program"'
    Write-Host "The folder must contain KSP_x64_Data\Managed\Assembly-CSharp.dll."
    exit 1
}

$requiredDlls = @(
    "Assembly-CSharp.dll",
    "Assembly-CSharp-firstpass.dll",
    "UnityEngine.dll"
)

foreach ($requiredDll in $requiredDlls) {
    $fullPath = Join-Path $ksp.ManagedPath $requiredDll
    if (-not (Test-Path $fullPath)) {
        Write-Host "Missing KSP dependency: $fullPath" -ForegroundColor Yellow
        exit 1
    }
}

$env:KSPDIR = $ksp.KspPath

Write-Host "KSP path: $($ksp.KspPath)"
Write-Host "KSP managed DLLs: $($ksp.ManagedPath)"
Write-Host "Building Release DLL..."

$sourceFile = Join-Path $root "Source\AutoTransferWindowPlanner.cs"
$roslynCompiler = Get-RoslynCompiler
$directBuildSucceeded = $false

try {
    $directBuildSucceeded = Invoke-DirectBuild `
        -CompilerPath $roslynCompiler `
        -ManagedPath $ksp.ManagedPath `
        -OutputDll $pluginDll `
        -SourceFile $sourceFile
} catch {
    Write-Host "Direct compiler path failed: $($_.Exception.Message)" -ForegroundColor Yellow
}

if (-not $directBuildSucceeded) {
    Write-Host "Falling back to MSBuild..."
    $msbuild = Get-MSBuildCommand
    if ($msbuild -eq "dotnet") {
        & dotnet msbuild $project /p:Configuration=Release /p:KSPDIR="$($ksp.KspPath)" /p:KSPManaged="$($ksp.ManagedPath)"
    } else {
        & $msbuild $project /p:Configuration=Release /p:KSPDIR="$($ksp.KspPath)" /p:KSPManaged="$($ksp.ManagedPath)"
    }

    if ($LASTEXITCODE -ne 0) {
        Write-Host "MSBuild failed with exit code $LASTEXITCODE." -ForegroundColor Yellow
        exit $LASTEXITCODE
    }
}

if (-not (Test-Path $pluginDll)) {
    Write-Host "Build finished, but the mod DLL was not created: $pluginDll" -ForegroundColor Yellow
    exit 1
}

New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
$packageRoot = Join-Path $OutputDir "$modName-package"
if (Test-Path $packageRoot) {
    Remove-Item -LiteralPath $packageRoot -Recurse -Force
}

$packageGameData = Join-Path $packageRoot "GameData"
New-Item -ItemType Directory -Force -Path $packageGameData | Out-Null
Copy-Item -LiteralPath $modGameData -Destination $packageGameData -Recurse -Force

$packagedModGameData = Join-Path $packageGameData $modName
$packagedPlaceholder = Join-Path $packagedModGameData "Plugins\README_PUT_DLL_HERE.txt"
if (Test-Path $packagedPlaceholder) {
    Remove-Item -LiteralPath $packagedPlaceholder -Force
}

if (-not $NoZip) {
    $zipPath = Join-Path $OutputDir "$modName-$modVersion.zip"
    if (Test-Path $zipPath) {
        Remove-Item -LiteralPath $zipPath -Force
    }
    Compress-Archive -Path (Join-Path $packageRoot "GameData") -DestinationPath $zipPath -Force
    Write-Host "Package: $zipPath" -ForegroundColor Green
}

if (-not $NoInstall) {
    $defaultTestGameData = ""
    $defaultSteamCommon = "F:\SteamLibrary\steamapps\common"
    if (Test-Path $defaultSteamCommon) {
        $defaultTestKsp = Get-ChildItem -LiteralPath $defaultSteamCommon -Directory -Filter "Kerbal Space Program*test" -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($defaultTestKsp) {
            $defaultTestGameData = Join-Path $defaultTestKsp.FullName "GameData"
        }
    }
    if ($InstallGameDataPath) {
        $targetGameData = $InstallGameDataPath
    } elseif ($defaultTestGameData -and (Test-Path $defaultTestGameData)) {
        $targetGameData = $defaultTestGameData
    } else {
        $targetGameData = Join-Path $ksp.KspPath "GameData"
    }

    if (-not (Test-Path $targetGameData)) {
        Write-Host "KSP GameData folder was not found: $targetGameData" -ForegroundColor Yellow
        exit 1
    }

    $targetModPath = Join-Path $targetGameData $modName
    if (Test-Path $targetModPath) {
        Remove-Item -LiteralPath $targetModPath -Recurse -Force
    }

    Copy-Item -LiteralPath $packagedModGameData -Destination $targetGameData -Recurse -Force
    Write-Host "Installed to: $targetModPath" -ForegroundColor Green
}

Write-Host "Done: $pluginDll" -ForegroundColor Green
