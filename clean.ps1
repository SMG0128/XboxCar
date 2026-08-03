[CmdletBinding(SupportsShouldProcess = $true)]
param()

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$project = Join-Path $root 'XboxCar_STM32_Display'
$projectFull = (Resolve-Path -LiteralPath $project -ErrorAction Stop).Path.TrimEnd('\')
$targets = @(
    (Join-Path $projectFull 'build\Debug'),
    (Join-Path $projectFull 'build\Release'),
    (Join-Path $projectFull 'build\tests')
)

foreach ($target in $targets) {
    $targetFull = [System.IO.Path]::GetFullPath($target).TrimEnd('\')
    $allowed = $targetFull.StartsWith($projectFull + '\', [System.StringComparison]::OrdinalIgnoreCase)
    $leaf = Split-Path -Leaf $targetFull
    if (-not $allowed -or $leaf -notin @('Debug', 'Release', 'tests')) {
        throw "Refusing to remove non-build path: $targetFull"
    }
    if (Test-Path -LiteralPath $targetFull) {
        if ($PSCmdlet.ShouldProcess($targetFull, 'Remove build artifacts')) {
            Remove-Item -LiteralPath $targetFull -Recurse -Force
            Write-Host "Cleaned: $targetFull"
        }
    } else {
        Write-Host "Not present, skipped: $targetFull"
    }
}
exit 0