[CmdletBinding(PositionalBinding = $true)]
param(
    [Parameter(Position = 0)]
    [string] $RobotAddress = "192.168.43.1",

    [switch] $Full
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

# Use the same configured Java/Android tools as tools/run_gradle.py.
$localDeploy = Join-Path $PSScriptRoot "tools/deploy.py"
if (Test-Path -LiteralPath $localDeploy -PathType Leaf) {
    $pythonCommand = $null
    foreach ($candidate in @("python3", "py", "python")) {
        $candidateCommand = Get-Command $candidate -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($null -eq $candidateCommand) { continue }
        $probeArgs = @()
        if ($candidateCommand.Name -match '^py(?:\.exe)?$') { $probeArgs += "-3" }
        try {
            # Windows Store aliases can exist without an installed interpreter.
            & $candidateCommand.Source @probeArgs -c "import sys; sys.exit(0 if sys.version_info[0] == 3 else 1)" *> $null
            if ($LASTEXITCODE -eq 0) {
                $pythonCommand = $candidateCommand
                break
            }
        }
        catch {
            # An unusable launcher must not prevent trying another installation.
        }
    }
    if ($null -eq $pythonCommand) {
        [Console]::Error.WriteLine("Python 3 is required to use the local FTC deployment tools.")
        exit 2
    }
    $pythonArgs = @()
    if ($pythonCommand.Name -match '^py(?:\.exe)?$') { $pythonArgs += "-3" }
    $pythonArgs += $localDeploy
    if ($Full) { $pythonArgs += "--full" }
    $pythonArgs += $RobotAddress
    & $pythonCommand.Source @pythonArgs
    exit $LASTEXITCODE
}

function Resolve-RobotTarget {
    param([string] $Address)

    if ($Address -notmatch '^(?<Host>[A-Za-z0-9.-]+)(?::(?<Port>[0-9]+))?$') {
        throw "Robot address must be a host or host:port."
    }

    $robotPort = 5555
    if ($Matches.ContainsKey("Port") -and $Matches["Port"]) {
        try {
            $robotPort = [int] $Matches["Port"]
        }
        catch {
            throw "Robot port must be an integer from 1 to 65535."
        }
    }
    if ($robotPort -lt 1 -or $robotPort -gt 65535) {
        throw "Robot port must be an integer from 1 to 65535."
    }

    return "{0}:{1}" -f $Matches["Host"], $robotPort
}

try {
    $target = Resolve-RobotTarget $RobotAddress
}
catch {
    [Console]::Error.WriteLine($_.Exception.Message)
    exit 2
}

$gradleTask = if ($Full) { "installDebug" } else { "deploySloth" }
$isWindowsHost = [System.Environment]::OSVersion.Platform -eq [System.PlatformID]::Win32NT
$gradleWrapper = Join-Path $PSScriptRoot $(if ($isWindowsHost) { "gradlew.bat" } else { "gradlew" })
$hadAndroidSerial = Test-Path Env:ANDROID_SERIAL
$previousAndroidSerial = if ($hadAndroidSerial) {
    (Get-Item Env:ANDROID_SERIAL).Value
}
else {
    $null
}
$connected = $false
$deployExitCode = 1

try {
    $env:ANDROID_SERIAL = $target

    & adb connect $target
    $connectExitCode = $LASTEXITCODE
    if ($connectExitCode -ne 0) {
        $deployExitCode = $connectExitCode
    }
    else {
        $connected = $true
        $deviceState = (& adb -s $target get-state | Out-String).Trim()
        $stateExitCode = $LASTEXITCODE

        if ($stateExitCode -ne 0) {
            $deployExitCode = $stateExitCode
        }
        elseif ($deviceState -ne "device") {
            [Console]::Error.WriteLine(
                'ADB target {0} is in state "{1}", not "device".' -f $target, $deviceState
            )
            $deployExitCode = 1
        }
        else {
            Push-Location $PSScriptRoot
            try {
                & $gradleWrapper $gradleTask
                $deployExitCode = $LASTEXITCODE
            }
            finally {
                Pop-Location
            }
        }
    }
}
catch {
    [Console]::Error.WriteLine($_.Exception.Message)
    $deployExitCode = 1
}
finally {
    if ($connected) {
        try {
            $null = & adb disconnect $target 2>$null
        }
        catch {
            # Cleanup must not replace the deployment result.
        }
    }

    if ($hadAndroidSerial) {
        $env:ANDROID_SERIAL = $previousAndroidSerial
    }
    else {
        Remove-Item Env:ANDROID_SERIAL -ErrorAction SilentlyContinue
    }
}

exit $deployExitCode
