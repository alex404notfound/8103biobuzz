# Dependency audit — 2026-09-18

Versions below were checked against the publishers' live Maven metadata,
published Gradle modules/POMs, and upstream source. All versions are pinned in
`TeamCode/build.gradle`.

| Library | Previous | Selected | Published evidence |
|---|---|---|---|
| Pedro robot integration | `com.pedropathing:revhub:3.0.0` | `3.0.1` | [Maven Central](https://repo.maven.apache.org/maven2/com/pedropathing/revhub/maven-metadata.xml), [release](https://github.com/Pedro-Pathing/PedroPathing/releases/tag/v3.0.1) |
| Pedro tuning | `com.pedropathing:tuning:1.0.0` | `1.0.1` | [Maven Central](https://repo.maven.apache.org/maven2/com/pedropathing/tuning/maven-metadata.xml), [published POM](https://repo.maven.apache.org/maven2/com/pedropathing/tuning/1.0.1/tuning-1.0.1.pom) |
| Ivy | `com.pedropathing.ivy:core:1.1.0` and `com.pedropathing.ivy:pedro:1.1.0` | Both `1.1.1` | [Core metadata](https://repo.maven.apache.org/maven2/com/pedropathing/ivy/core/maven-metadata.xml), [Pedro integration metadata](https://repo.maven.apache.org/maven2/com/pedropathing/ivy/pedro/maven-metadata.xml), [release](https://github.com/Pedro-Pathing/Ivy/releases/tag/v1.1.1) |
| Sloth Load plugin | `0.3.0` | `0.3.2` | [Dairy metadata](https://repo.dairy.foundation/releases/dev/frozenmilk/Load/maven-metadata.xml) |
| Sloth runtime | `0.3.0` | `0.3.2` | [Dairy metadata](https://repo.dairy.foundation/releases/dev/frozenmilk/sinister/Sloth/maven-metadata.xml) |
| Slothboard | `0.3.0+0.6.0` | `0.3.2+0.6.0` | [Dairy metadata](https://repo.dairy.foundation/releases/com/acmerobotics/slothboard/dashboard/maven-metadata.xml) |
| Sloth Panels | `0.2.4.1+1.0.12` | `0.3.2+1.0.13` | [Dairy metadata](https://repo.dairy.foundation/releases/com/bylazar/sloth/fullpanels/maven-metadata.xml), [original Panels metadata](https://mymaven.bylazar.com/releases/com/bylazar/fullpanels/maven-metadata.xml) |
| FTCLib core | `2.1.1` | `2.1.1` | [Maven Central](https://repo.maven.apache.org/maven2/org/ftclib/ftclib/core/maven-metadata.xml), [release](https://github.com/FTCLib/FTCLib/releases/tag/v2.1.1) |
| JUnit | `4.13.2` | `4.13.2` | [Maven Central](https://repo.maven.apache.org/maven2/junit/junit/maven-metadata.xml) |
| Mockito subclass | `5.23.0` | `5.23.0` | [Maven Central](https://repo.maven.apache.org/maven2/org/mockito/mockito-subclass/maven-metadata.xml), [release](https://github.com/mockito/mockito/releases/tag/v5.23.0) |
| Test JSON implementation | `20260814` | `20260814` | [Maven Central](https://repo.maven.apache.org/maven2/org/json/json/maven-metadata.xml), [release](https://github.com/stleary/JSON-java/releases/tag/20260814) |

FTCLib, JUnit, Mockito subclass, and JSON remain their latest published versions
under these coordinates. Pedro `revhub:3.0.1` supplies `core:3.0.1` transitively.
The new tuning artifact requests Pedro `3.0.0` and Sloth `0.3.2`; Gradle selects
the explicitly declared newer Pedro `3.0.1`.

## Pedro and Ivy compatibility

[Pedro 3.0.1](https://github.com/Pedro-Pathing/PedroPathing/releases/tag/v3.0.1)
fixes path-completion calculation and waits 500 ms for Pinpoint IMU calibration.
Its PoseFactory API now uses explicit angle units; TeamCode does not use that
API. The existing Pedro 3 drivetrain/path API remains in use. Verify autonomous
completion and rerun relevant tuning checks on the robot after installation.

[Ivy 1.1.1](https://github.com/Pedro-Pathing/Ivy/releases/tag/v1.1.1) exposes its
core library as an API dependency of its Pedro integration. Both direct
declarations remain on the same version.

## Pedro Quickstart alignment

This repository already adapts the official Pedro Quickstart's tuning procedures
within Team 8103's robot framework. Pedro's
[manual installation instructions](https://pedropathing.com/docs/pathing/installation)
support adding its dependencies and copying the Quickstart's `pedro` package into
an existing project. A separate Quickstart checkout is not required to use Pedro.

The setup was compared with official Quickstart commit
[`79670fb9`](https://github.com/Pedro-Pathing/Quickstart/tree/79670fb9c432a3f5342054d69358f3826ac07f1c).
Its SDK 12, Pedro `3.0.1`, and AutoTune `1.0.1` match this update. Our
`Constants.createGuardedFollower` composes the configured Pinpoint localizer,
mecanum drivetrain, and Foresight algorithm as described in the
[official constants guide](https://pedropathing.com/docs/pathing/tuning/constants).
`Tuning.java` registers the mecanum, Pinpoint, Foresight, and path-test procedures
using static `@Tuner` factories returning `Procedure`, matching the
[official AutoTune setup](https://pedropathing.com/docs/pathing/tuning/foresight).

Recent [Quickstart changes](https://github.com/Pedro-Pathing/Quickstart/compare/3738d5bfce54d281f3dde7eef0a396f74c34512a...79670fb9c432a3f5342054d69358f3826ac07f1c)
add startup settling and pose reset/update after START. Our Foresight procedures
already reset and sample the pose after `SafeTuningOpMode` waits for START and
explicit arming with healthy Pinpoint data. The shared lifecycle retains STOP,
timeout, and motor cleanup handling. Robot measurements still need to be entered
in Constants using the workflow in [TUNING.md](TUNING.md).

## Matching Sloth wrappers

The current [fullpanels wrapper](https://repo.dairy.foundation/releases/com/bylazar/sloth/fullpanels/0.3.2+1.0.13/fullpanels-0.3.2+1.0.13.module)
is published and includes Panels `1.0.13`. Its
[configurables](https://repo.dairy.foundation/releases/com/bylazar/sloth/configurables/0.3.2+1.0.5/configurables-0.3.2+1.0.5.module)
and [opmodecontrol](https://repo.dairy.foundation/releases/com/bylazar/sloth/opmodecontrol/0.3.2+1.0.3/opmodecontrol-0.3.2+1.0.3.module)
modules strictly request Sloth `0.3.2`, matching
[Slothboard](https://repo.dairy.foundation/releases/com/acmerobotics/slothboard/dashboard/0.3.2+0.6.0/dashboard-0.3.2+0.6.0.module)
and the direct Sloth dependency. The former exclusion that forced an older
Panels wrapper onto a newer Sloth runtime has been removed.

The Dashboard and Panels wrappers provide the original Java package names.
The exclusions for `com.acmerobotics.dashboard` and `com.bylazar` keep the
original implementations from entering the APK alongside their Sloth variants.

Dashboard 0.6 also starts Limelight proxies on ports `5800`, `5801`, `5805`,
and `5807`. Exclude `com.bylazar.sloth:limelightproxy` from fullpanels so only
Dashboard owns those ports. Including both causes an `Address already in use`
startup crash on the Control Hub, leaving the status light blue and the Robot
Controller console unavailable. This dependency change requires a full APK install.

Sloth's recent fixes improve
[error handling and Java compatibility](https://github.com/Dairy-Foundation/Sloth/commit/5f4ee9457d84d474b3b44ca6481bef7a4518661d),
prevent a [hardwareMap initialization race](https://github.com/Dairy-Foundation/Sloth/commit/4ec6afd0df2efed81d7f3e388cafa1b8420ef8fb),
and improve [old reload removal](https://github.com/Dairy-Foundation/Sloth/commit/7b8ba6c914366b61b9d12402da6b0257ca02d007).
Panels configuration and OpMode hot reload still need an on-robot smoke test
after the full APK installation.

## Java and deployment

[Load 0.3.2 metadata](https://repo.dairy.foundation/releases/dev/frozenmilk/Load/0.3.2/Load-0.3.2.module)
targets Java 17; the former Java 25 requirement from Load 0.3.0 no longer
applies. The local runner selects an installed Java 21 or Java 25 JDK. Load
0.3.2 uses AGP `8.13.2`, matching FIRST's SDK 12 build.

The SDK-owned files retain FIRST's [SDK 12 build settings](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/build.gradle):
AGP `8.13.2`, [Gradle `9.1.0`](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/gradle/wrapper/gradle-wrapper.properties),
SDK components `12.0.0`, and [AppCompat `1.2.0`](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/build.dependencies.gradle).

The [Load deployment protocol](https://github.com/Dairy-Foundation/Sloth/blob/4731f3fcd291f9081d9e176559cbd3cc952272d8/Load/src/main/kotlin/dev/frozenmilk/sinister/sloth/DeploySloth.kt)
uses a new numbered jar filename for every reload. Our task actions preserve
that protocol, require `ANDROID_SERIAL`, pass `adb -s` explicitly, and disable
the plugin's automatic connection handling. They check the runtime's actual
`sloth.lock` filename. Before a full install they remove only numbered reload
jars, `loaded.jar`, `to_load.jar`, and `sloth.lock`; unrelated files are retained.

Run a full APK install after changing these libraries. A Sloth-only reload
cannot update the runtime dependencies already installed on the robot.

Mockito 5 defaults to inline instrumentation and requires Java 11 or later.
The published subclass variant brings `mockito-core:5.23.0` transitively while
preserving the existing tests' subclass mock behavior. See the
[Mockito migration notes](https://github.com/mockito/mockito/blob/v5.23.0/README.md).

## Tooling verification

Validated this update on Windows with JDK 21, Android platform 30, Build Tools
35.0.0, and the project's Gradle 9.1 wrapper:

- The runtime dependency graph resolves the selected versions together.
- `:TeamCode:testDebugUnitTest`: 115 tests passed, with no failures or skips.
- `:TeamCode:assembleDebug` and `:TeamCode:assembleSloth`: both passed.
- Python host tests: 3 passed; 31 POSIX-only or opt-in integration tests were
  skipped on this host. The toolchain doctor and `git diff --check` passed.

These checks do not replace robot measurements or an on-hardware smoke test.

The host tests include the real Gradle task classes with every device command
redirected to a fake ADB executable. They check the selected serial, lock/push
sequence, cleanup whitelist, failure propagation, and cleanup-before-install
ordering. The POSIX and PowerShell wrappers are also executed against fake
tools. No robot is contacted by these tests.

```sh
FTC_TOOLS_HOME=/path/to/shared/.tools \
FTC_SLOTH_TEST_SDK=/path/to/8103biobuzz \
python3 -m unittest discover -s tests -v
```
