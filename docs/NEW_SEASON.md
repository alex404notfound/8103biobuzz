# 8103 BIOBUZZ project

Work in your local **8103biobuzz checkout**. This is the GitHub fork
[alex404notfound/8103biobuzz](https://github.com/alex404notfound/8103biobuzz),
running FTC SDK 12.0.0 with Pedro 3 and Ivy. See [START_HERE.md](../START_HERE.md)
for building and [TUNING.md](TUNING.md) for actual robot setup.

[BIOBUZZ software requirements](BIOBUZZ_SOFTWARE_REQUIREMENTS.md) maps the
TU01 game manual to the remaining mechanism, vision, autonomous, and competition
configuration work.

Robot source is in TeamCode/src/main/java. TeamCode/build.gradle pins the team
libraries; FIRST's root build files supply the SDK. Build and tune this folder.

The origin remote is your fork; upstream is FIRST's official SDK. Commit your
robot changes here and push to origin. No Git merge is needed to use this project.

Development tools are machine-specific and are not included in the checkout.
Install Python 3, JDK 21, and the Android SDK packages described in
[START_HERE.md](../START_HERE.md). The runner defaults to `.tools/jdk21` and
`.tools/sdk` in this project, with `.tools/jdk25` supported as a fallback. To use
a shared tools directory with that same layout, set `FTC_TOOLS_HOME` or the
`tools_home` absolute path in an ignored
`.ftc-tools.json` at the project root.

Android Studio's local `.idea/` settings and `local.properties` SDK path are
ignored too; configure the Gradle JDK and SDK location on each computer. No
separate template or FTCLib source checkout is needed: Gradle resolves the pinned
libraries from their Maven repositories.
