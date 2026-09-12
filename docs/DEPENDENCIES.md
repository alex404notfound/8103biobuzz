# Dependency audit — 2026-09-12

Versions below were checked against the publishers' Maven metadata, published
artifacts and upstream source. All versions are pinned in `TeamCode/build.gradle`.

| Library | Previous | Selected | Published evidence |
|---|---|---|---|
| Pedro robot integration | `com.pedropathing:ftc:2.1.0-alpha.2` | `com.pedropathing:revhub:3.0.0` | [Maven Central](https://repo.maven.apache.org/maven2/com/pedropathing/revhub/maven-metadata.xml) |
| Pedro tuning | `com.pedropathing:telemetry:1.0.0` | `com.pedropathing:tuning:1.0.0` | [Published POM](https://repo.maven.apache.org/maven2/com/pedropathing/tuning/1.0.0/tuning-1.0.0.pom) |
| Ivy | `com.pedropathing:ivy:1.0.0` | `com.pedropathing.ivy:core:1.1.0` and `com.pedropathing.ivy:pedro:1.1.0` | [Core metadata](https://repo.maven.apache.org/maven2/com/pedropathing/ivy/core/maven-metadata.xml), [Pedro integration metadata](https://repo.maven.apache.org/maven2/com/pedropathing/ivy/pedro/maven-metadata.xml) |
| Sloth Load plugin | `0.2.4` | `0.3.0` | [Dairy metadata](https://repo.dairy.foundation/releases/dev/frozenmilk/Load/maven-metadata.xml) |
| Sloth runtime | `0.2.4` | `0.3.0` | [Dairy metadata](https://repo.dairy.foundation/releases/dev/frozenmilk/sinister/Sloth/maven-metadata.xml) |
| Slothboard | `0.2.4+0.5.1` | `0.3.0+0.6.0` | [Dairy metadata](https://repo.dairy.foundation/releases/com/acmerobotics/slothboard/dashboard/maven-metadata.xml) |
| Sloth Panels | `0.2.4.1+1.0.12` | `0.2.4.1+1.0.12` | [Dairy metadata](https://repo.dairy.foundation/releases/com/bylazar/sloth/fullpanels/maven-metadata.xml) |
| FTCLib core | `2.1.1` | `2.1.1` | [Maven Central](https://repo.maven.apache.org/maven2/org/ftclib/ftclib/core/maven-metadata.xml), [release](https://github.com/FTCLib/FTCLib/releases/tag/v2.1.1) |
| JUnit | `4.13.2` | `4.13.2` | [Maven Central](https://repo.maven.apache.org/maven2/junit/junit/maven-metadata.xml) |
| Mockito | `mockito-core:4.11.0` | `mockito-subclass:5.23.0` | [Release](https://github.com/mockito/mockito/releases/tag/v5.23.0), [subclass POM](https://repo.maven.apache.org/maven2/org/mockito/mockito-subclass/5.23.0/mockito-subclass-5.23.0.pom) |
| Test JSON implementation | `20240303` | `20260814` | [Maven Central](https://repo.maven.apache.org/maven2/org/json/json/maven-metadata.xml), [release](https://github.com/stleary/JSON-java/releases/tag/20260814) |

FTCLib 2.1.1 and JUnit 4.13.2 remain their latest published versions under these
coordinates. The FTCLib default branch is still the
[2.1.1 release commit](https://github.com/FTCLib/FTCLib/commit/1c8995d09413b406e0f4aff238ea4edc2bb860c4).
An independent FTCLib v2.1.1 source checkout is installed on this computer at
`~/Downloads/8103Template/.tools/ftclib` (tag commit
`98e921bf9eac506c2bbe80a0cddf51ae33f1cbb6`). It provides reference source/examples;
the robot's installed dependency remains the published Gradle artifact.

## Panels compatibility

The [Sloth README](https://github.com/Dairy-Foundation/Sloth) advertises
`com.bylazar.sloth:fullpanels:0.3.0+1.0.12`, but its Maven POM returned HTTP 404
on the audit date. The repository metadata lists `0.2.4.1+1.0.12` as the latest
published wrapper. The original [Panels metadata](https://mymaven.bylazar.com/releases/com/bylazar/fullpanels/maven-metadata.xml)
also still lists underlying Panels `1.0.12`.

The existing wrapper's [configurables metadata](https://repo.dairy.foundation/releases/com/bylazar/sloth/configurables/0.2.4.1+1.0.5/configurables-0.2.4.1+1.0.5.module)
and [opmodecontrol metadata](https://repo.dairy.foundation/releases/com/bylazar/sloth/opmodecontrol/0.2.4.1+1.0.3/opmodecontrol-0.2.4.1+1.0.3.module)
strictly request Sloth `0.2.4`. That transitive dependency is excluded on
`fullpanels`; the project supplies Sloth `0.3.0` explicitly.

The published configurables and opmodecontrol binaries contain ten distinct
Dairy member references. All ten resolve against the published Sloth `0.3.0`,
Sinister `2.3.0`, and Dairy Util `1.2.2` binaries. Their source also contains no
reference to the removed `TeleopAutonomousOpModeScanner` class. This checks
binary linkage; on-robot Panels configuration and OpMode hot reload still need
a smoke test after the full APK installation.

The Dashboard and Panels wrappers provide the original Java package names.
The exclusions for `com.acmerobotics.dashboard` and `com.bylazar` keep the
original implementations from entering the APK alongside their Sloth variants.

## Java and deployment

Load `0.3.0` is published for [Java 25](https://repo.dairy.foundation/releases/dev/frozenmilk/Load/0.3.0/Load-0.3.0.module).
The local runner therefore selects `jdk25` for this official SDK project and
reports a missing Java 25 installation before Gradle or ADB starts. The
development machine already has this JDK under the shared `.tools` directory.

The SDK-owned files retain FIRST's [SDK 12 build settings](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/build.gradle):
AGP `8.13.2`, [Gradle `9.1.0`](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/gradle/wrapper/gradle-wrapper.properties),
SDK components `12.0.0`, and [AppCompat `1.2.0`](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/build.dependencies.gradle).

The [Load 0.3 deployment protocol](https://github.com/Dairy-Foundation/Sloth/blob/master/Load/src/main/kotlin/dev/frozenmilk/sinister/sloth/DeploySloth.kt)
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
