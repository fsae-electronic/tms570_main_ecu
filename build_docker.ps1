param(
    [switch]$BenchTest
)

$ErrorActionPreference = "Continue"

Write-Host "Iniciando compilacion con Docker ($('tms570-toolchain'))..." -ForegroundColor Cyan

# Eliminar contenedor previo si quedo colgado
docker rm -f tms_builder > $null 2>&1

$makeArgs = "make"
if ($BenchTest) {
    $makeArgs = "make BENCH_TEST_SIMULATE_CAN=1"
}

# 1. Crear contenedor
docker create --name tms_builder -w /project tms570-toolchain sh -c $makeArgs | Out-Null

# 2. Copiar archivos del proyecto al contenedor
docker cp . tms_builder:/project | Out-Null

# 3. Compilar dentro del contenedor
docker start -a tms_builder

# 4. Extraer los binarios generados
docker cp tms_builder:/project/tms570_main_ecu.elf . | Out-Null
docker cp tms_builder:/project/tms570_main_ecu.bin . | Out-Null

# 5. Limpiar contenedor
docker rm -f tms_builder | Out-Null

Write-Host "`nCompilacion exitosa!" -ForegroundColor Green
Get-Item tms570_main_ecu.elf, tms570_main_ecu.bin | Select-Object Name, Length, LastWriteTime | Format-Table
