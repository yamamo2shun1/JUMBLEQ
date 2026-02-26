param(
    [Parameter(Mandatory = $false)]
    [string]$ProjectDir = "C:\Users\shuni\JUMBLEQ\STM32CubeIDE\JUMBLEQ\Appli"
)

$ErrorActionPreference = "Stop"

function Add-Result {
    param(
        [string]$Check,
        [bool]$Passed,
        [string[]]$Details = @()
    )
    [PSCustomObject]@{
        Check   = $Check
        Passed  = $Passed
        Details = $Details
    }
}

function Read-XmlFile {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "File not found: $Path"
    }
    [xml](Get-Content -LiteralPath $Path -Raw)
}

$results = @()
$projectFile = Join-Path $ProjectDir ".project"
$cprojectFile = Join-Path $ProjectDir ".cproject"
$sourcesMk = Join-Path $ProjectDir "Debug\sources.mk"
$objectsList = Join-Path $ProjectDir "Debug\objects.list"

if (-not (Test-Path -LiteralPath $ProjectDir)) {
    Write-Error "ProjectDir not found: $ProjectDir"
    exit 2
}

try {
    # Check 1: .project linkedResources
    $proj = Read-XmlFile -Path $projectFile
    $links = @($proj.projectDescription.linkedResources.link)
    $names = $links | ForEach-Object { $_.name }

    $hasDriversFolder = $links | Where-Object { $_.name -eq "Drivers" -and $_.type -eq "2" }
    $hasMiddlewaresFolder = $links | Where-Object { $_.name -eq "Middlewares" -and $_.type -eq "2" }

    $badFileLinks = $links | Where-Object {
        $_.type -eq "1" -and (
            $_.name -like "Drivers/STM32H7RSxx_HAL_Driver/*" -or
            $_.name -like "Middlewares/Third_Party/FreeRTOS/*"
        )
    }

    $ok1 = ($null -ne $hasDriversFolder) -and ($null -ne $hasMiddlewaresFolder) -and (@($badFileLinks).Count -eq 0)
    $d1 = @()
    if (-not $hasDriversFolder) { $d1 += "Missing folder link: Drivers (type=2)" }
    if (-not $hasMiddlewaresFolder) { $d1 += "Missing folder link: Middlewares (type=2)" }
    if (@($badFileLinks).Count -gt 0) {
        $d1 += "Found forbidden file links under HAL/FreeRTOS:"
        $d1 += @($badFileLinks | ForEach-Object { "  - $($_.name)" })
    }
    $results += Add-Result -Check "1) .project linkedResources" -Passed $ok1 -Details $d1

    # Check 2: .cproject sourceEntries (Debug/Release)
    $cp = Read-XmlFile -Path $cprojectFile
    $configs = @(
        $cp.SelectNodes("//storageModule[@moduleId='org.eclipse.cdt.core.settings']/cconfiguration/storageModule[@moduleId='cdtBuildSystem']/configuration")
    )
    if ($configs.Count -eq 0) {
        throw "No build configurations found in .cproject"
    }

    $required = @(
        "Drivers/STM32H7RSxx_HAL_Driver/Src",
        "Middlewares/Third_Party/FreeRTOS/Source"
    )
    $forbidden = @(
        "Drivers/STM32H7RSxx_HAL_Driver",
        "Middlewares/Third_Party/FreeRTOS"
    )

    $ok2 = $true
    $d2 = @()
    foreach ($cfg in $configs) {
        $cfgName = [string]$cfg.name
        if ([string]::IsNullOrWhiteSpace($cfgName)) {
            $cfgName = "configuration"
        }
        $entries = @($cfg.sourceEntries.entry | ForEach-Object { [string]$_.name })
        foreach ($need in $required) {
            if ($entries -notcontains $need) {
                $ok2 = $false
                $d2 += ("{0}: missing sourcePath '{1}'" -f $cfgName, $need)
            }
        }
        foreach ($ng in $forbidden) {
            if ($entries -contains $ng) {
                $ok2 = $false
                $d2 += ("{0}: forbidden sourcePath '{1}'" -f $cfgName, $ng)
            }
        }
    }
    $results += Add-Result -Check "2) .cproject sourceEntries" -Passed $ok2 -Details $d2

    # Check 3: Debug/sources.mk SUBDIRS
    if (-not (Test-Path -LiteralPath $sourcesMk)) {
        $results += Add-Result -Check "3) Debug/sources.mk SUBDIRS" -Passed $false -Details @("File not found: $sourcesMk")
    } else {
        $srcText = Get-Content -LiteralPath $sourcesMk -Raw
        $badDirs = @()
        if ($srcText -match "(?m)^\s*Drivers/STM32H7RSxx_HAL_Driver\s*\\") {
            $badDirs += "Drivers/STM32H7RSxx_HAL_Driver"
        }
        if ($srcText -match "(?m)^\s*Middlewares/Third_Party/FreeRTOS\s*\\") {
            $badDirs += "Middlewares/Third_Party/FreeRTOS"
        }
        $ok3 = ($badDirs.Count -eq 0)
        $d3 = @()
        if (-not $ok3) {
            $d3 += "Forbidden root entries in SUBDIRS:"
            $d3 += @($badDirs | ForEach-Object { "  - $_" })
        }
        $results += Add-Result -Check "3) Debug/sources.mk SUBDIRS" -Passed $ok3 -Details $d3
    }

    # Check 4: Debug/objects.list duplicate path variants
    if (-not (Test-Path -LiteralPath $objectsList)) {
        $results += Add-Result -Check "4) Debug/objects.list duplicates" -Passed $false -Details @("File not found: $objectsList")
    } else {
        $objs = Get-Content -LiteralPath $objectsList | Where-Object { $_.Trim() -ne "" }
        $objs = $objs | ForEach-Object { $_.Trim('"') }

        $driversSrc = @($objs | Where-Object { $_ -like "./Drivers/STM32H7RSxx_HAL_Driver/Src/*.o" })
        $driversRoot = @($objs | Where-Object { $_ -like "./Drivers/STM32H7RSxx_HAL_Driver/*.o" })
        $mwSrc = @($objs | Where-Object { $_ -like "./Middlewares/Third_Party/FreeRTOS/Source/*.o" })
        $mwRoot = @($objs | Where-Object { $_ -like "./Middlewares/Third_Party/FreeRTOS/*.o" })

        $badPairs = @()
        $driversSrcBase = $driversSrc | ForEach-Object { [System.IO.Path]::GetFileName($_) }
        $driversRootBase = $driversRoot | ForEach-Object { [System.IO.Path]::GetFileName($_) }
        $mwSrcBase = $mwSrc | ForEach-Object { [System.IO.Path]::GetFileName($_) }
        $mwRootBase = $mwRoot | ForEach-Object { [System.IO.Path]::GetFileName($_) }

        foreach ($b in $driversSrcBase) {
            if ($driversRootBase -contains $b) {
                $badPairs += "Drivers duplicated: $b (Src and root)"
            }
        }
        foreach ($b in $mwSrcBase) {
            if ($mwRootBase -contains $b) {
                $badPairs += "FreeRTOS duplicated: $b (Source and root)"
            }
        }

        $ok4 = ($badPairs.Count -eq 0)
        $results += Add-Result -Check "4) Debug/objects.list duplicates" -Passed $ok4 -Details $badPairs
    }
}
catch {
    Write-Error $_
    exit 2
}

$failed = @($results | Where-Object { -not $_.Passed })

Write-Host "Project: $ProjectDir"
Write-Host ""
foreach ($r in $results) {
    $status = if ($r.Passed) { "PASS" } else { "FAIL" }
    Write-Host ("[{0}] {1}" -f $status, $r.Check)
    if ($r.Details.Count -gt 0) {
        $r.Details | ForEach-Object { Write-Host ("  " + $_) }
    }
}

Write-Host ""
if ($failed.Count -eq 0) {
    Write-Host "Overall: PASS"
    exit 0
}
else {
    Write-Host ("Overall: FAIL ({0} check(s) failed)" -f $failed.Count)
    exit 1
}
