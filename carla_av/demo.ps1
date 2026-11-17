# One-command live demo (our pipeline + Prithiv's ideas), with the instrument cluster.
#   .\demo.ps1                 obstacle stop + overtake on Town04
#   .\demo.ps1 city            pedestrian crossing + ambulance making us yield (Town03_Opt)
#   .\demo.ps1 navigation      route through junctions, stopping at red lights (Town01)
#   .\demo.ps1 moving          lead car, braking car, oncoming overtake (Town03_Opt)
#   .\demo.ps1 curves          sharp curves (Town04)
#   add -Traffic 12 for Traffic-Manager cars (lorry, police, cars) and -Night for night driving
param([string]$what = "all", [int]$Traffic = 0, [switch]$Night)

$towns = @{ all = "Town04"; city = "Town03_Opt"; navigation = "Town01"; moving = "Town03_Opt"; curves = "Town04" }
$town = $towns[$what]
if (-not $town) { Write-Host "Use: all | city | navigation | moving | curves"; exit 1 }

Set-Location $PSScriptRoot
$env:PYTHONPATH = "$PSScriptRoot\.venv\Lib\site-packages"
# The venv's own python.exe is blocked by Windows App Control; use the uv Python it was built from.
$py = "$env:APPDATA\uv\python\cpython-3.11.15-windows-x86_64-none\python.exe"

if (-not (Get-Process CarlaUE4* -ErrorAction SilentlyContinue)) {
    Write-Host "Starting CARLA..."
    Start-Process -FilePath "C:\CARLA\CarlaUE4.exe" -ArgumentList "-quality-level=Low"
    for ($i = 0; $i -lt 24; $i++) {
        Start-Sleep -Seconds 5
        & $py -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(10); c.get_world()" 2>$null
        if ($LASTEXITCODE -eq 0) { break }
    }
}
Write-Host "Driving $what on $town (watch the CARLA window and the instrument cluster)..."
& $py -m scenarios.run_scenarios --scenario $what --towns $town --runs 1 `
    --obstacle lidar --adjacent lidar --realtime --hud --cluster --out results_demo_prof `
    --traffic $Traffic @(if ($Night) { "--night" })
