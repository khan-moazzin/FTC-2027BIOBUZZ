param([switch]$Demo)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskJava = if ($env:JAVA_HOME) { Join-Path $env:JAVA_HOME 'bin' } else { 'C:/Program Files/Android/Android Studio/jbr/bin' }
$taskClasses = Join-Path $taskRoot 'build/physics-generator'
New-Item -ItemType Directory -Force -Path $taskClasses | Out-Null
$taskSource = Join-Path $taskRoot 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
$taskFiles = @('config/PhysicsShotConfig.java','config/VisionConfig.java','config/ShotConfig.java','config/MechanismConfig.java','lib/control/Ballistics.java','lib/control/ShotPolynomial.java','lib/math/Vec3.java','lib/math/LeastSquares.java','lib/field/Field.java','lib/calibration/ConfigSource.java') | ForEach-Object { Join-Path $taskSource $_ }
& (Join-Path $taskJava 'javac.exe') -d $taskClasses $taskFiles (Join-Path $taskRoot 'tools/physics/GenerateShotModel.java')
if ($LASTEXITCODE -ne 0) { throw 'Generator compilation failed' }
$taskOutput = Join-Path $taskRoot $(if ($Demo) {'build/physics-demo'} else {'build/generated-physics'})
$taskArguments = @('-cp', $taskClasses, 'GenerateShotModel', $taskOutput)
if ($Demo) { $taskArguments += '--demo' }
& (Join-Path $taskJava 'java.exe') $taskArguments
if ($LASTEXITCODE -ne 0) { throw 'Model generation rejected; inspect the error above' }
