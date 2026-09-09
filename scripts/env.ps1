# Dot-source this in PowerShell on Windows before running make, uv, node or npm:
#   . .\scripts\env.ps1
$candidates = @("$env:USERPROFILE\.local\bin", "$env:LOCALAPPDATA\Programs\nodejs")
foreach ($dir in $candidates) {
    if ((Test-Path $dir) -and (($env:Path -split ';') -notcontains $dir)) {
        $env:Path = "$dir;$env:Path"
    }
}
