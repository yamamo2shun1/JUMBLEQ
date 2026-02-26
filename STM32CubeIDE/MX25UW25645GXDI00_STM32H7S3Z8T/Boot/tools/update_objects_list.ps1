param(
    [string]$ObjectsList = "objects.list"
)

$extraGlobs = @(
    "./Drivers/STM32H7RSxx_HAL_Driver/*.o",
    "./Middlewares/ST/STM32_ExtMem_Manager/*.o"
)

if (Test-Path $ObjectsList) {
    $existing = Get-Content $ObjectsList
} else {
    New-Item -Path $ObjectsList -ItemType File -Force | Out-Null
    $existing = @()
}

$set = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
foreach ($line in $existing) {
    $trimmed = $line.Trim()
    if ($trimmed.Length -gt 0) {
        [void]$set.Add($trimmed)
    }
}

foreach ($glob in $extraGlobs) {
    Get-ChildItem -Path $glob -File -ErrorAction SilentlyContinue |
        Sort-Object Name |
        ForEach-Object {
            $rel = (Resolve-Path -Relative $_.FullName) -replace '\\', '/'
            if ($rel -notmatch '^\./') {
                $rel = "./" + $rel.TrimStart('.', '/')
            }

            $entry = '"' + $rel + '"'
            if ($set.Add($entry)) {
                Add-Content -Path $ObjectsList -Value $entry
            }
        }
}
