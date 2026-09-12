#!/usr/bin/env python3
"""Run this checkout's Gradle wrapper using project-local FTC development tools."""
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys


class SetupError(ValueError):
    """A local development-tool configuration needs attention."""


def tools_home(project, environment):
    override = environment.get("FTC_TOOLS_HOME")
    if override:
        value, source = override, "FTC_TOOLS_HOME"
    else:
        config = project / ".ftc-tools.json"
        if not config.exists():
            return project / ".tools"
        try:
            data = json.loads(config.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            raise SetupError(f"Cannot read {config}; expected JSON with an absolute tools_home path.") from None
        value = data.get("tools_home") if isinstance(data, dict) else None
        source = str(config)
    if not isinstance(value, str) or not value.strip() or not Path(value).is_absolute():
        raise SetupError(f"{source} must specify an absolute tools_home directory.")
    return Path(value).resolve()


def find_java_home(directory, windows=None):
    windows = os.name == "nt" if windows is None else windows
    suffix = ".exe" if windows else ""
    # Also support an unmodified vendor archive inside jdk21/ (or jdk8/25).
    candidates = [directory / "Contents/Home", directory]
    if directory.is_dir():
        for child in sorted(directory.iterdir()):
            if child.is_dir():
                candidates.extend([child / "Contents/Home", child])
    for candidate in candidates:
        binaries = [candidate / "bin" / (name + suffix) for name in ("java", "javac")]
        if all(binary.is_file() and (windows or os.access(binary, os.X_OK)) for binary in binaries):
            return candidate
    return None


def load_setup(project, environment):
    tools = tools_home(project, environment)
    official = (project / "settings.gradle").is_file() and (project / "FtcRobotController").is_dir()
    homes = {v: find_java_home(tools / f"jdk{v}") for v in (8, 21, 25)}
    missing = []
    for version in ((21,) if official else (21, 8)):
        if homes[version] is None:
            missing.append(f"JDK {version} with bin/java and bin/javac under {tools / ('jdk' + str(version))}")
    sdk = tools / "sdk"
    if not (sdk / "platforms/android-30/android.jar").is_file():
        missing.append(f"Android SDK package platforms;android-30 under {sdk}")
    build_tools = sdk / "build-tools"
    if not build_tools.is_dir() or not any((p / "source.properties").is_file() for p in build_tools.iterdir()):
        missing.append(f"Android SDK build-tools under {build_tools} (install the version required by the project)")
    if not (sdk / "platform-tools" / ("adb.exe" if os.name == "nt" else "adb")).is_file():
        missing.append(f"Android SDK package platform-tools under {sdk}")
    wrapper = project / ("gradlew.bat" if os.name == "nt" else "gradlew")
    if not wrapper.is_file():
        missing.append(f"Gradle wrapper {wrapper}")
    if missing:
        raise SetupError("Missing development tools:\n  - " + "\n  - ".join(missing)
                         + "\nInstall them here, or select an existing installation with FTC_TOOLS_HOME"
                         + " or .ftc-tools.json (tools_home).")
    child_env = dict(environment)
    child_env.update(JAVA_HOME=str(homes[21]), ANDROID_HOME=str(sdk), ANDROID_SDK_ROOT=str(sdk),
                     ANDROID_USER_HOME=str(tools / "android-user"), GRADLE_USER_HOME=str(tools / "gradle-home"))
    child_env["PATH"] = os.pathsep.join([str(homes[21] / "bin"), environment.get("PATH", ""), str(sdk / "platform-tools")])
    return tools, homes, official, wrapper, child_env


def read_property(path, name):
    if not path.is_file():
        return None
    value, logical = None, ""
    for physical in path.read_text(encoding="utf-8").splitlines():
        logical += physical.lstrip() if logical else physical
        if len(logical) - len(logical.rstrip("\\")) & 1:
            logical = logical[:-1]
            continue
        match = re.match(r"^\s*" + re.escape(name) + r"\s*[=:]\s*(.*)$", logical)
        if match:
            value = match.group(1)
        logical = ""
    return value


def gradle_arguments(project, tools, homes, arguments, environment, official):
    key = "-Dorg.gradle.jvmargs="
    value = "-Xmx1024M"
    for path in (project / "gradle.properties", tools / "gradle-home/gradle.properties"):
        configured = read_property(path, "org.gradle.jvmargs")
        if configured is not None:
            value = configured
    for variable in ("JAVA_OPTS", "GRADLE_OPTS"):
        try:
            options = shlex.split(environment.get(variable, ""))
        except ValueError:
            raise SetupError(f"Cannot parse {variable}; check its quoting.") from None
        for option in options:
            if option.startswith(key):
                value = option[len(key):]
    remaining = []
    for argument in arguments:
        if argument.startswith(key):
            value = argument[len(key):]
        else:
            remaining.append(argument)
    user_home = str(tools / "android-user")
    if "'" in user_home:
        raise SetupError("The tools directory cannot contain an apostrophe; select another FTC_TOOLS_HOME.")
    # Gradle splits JVM options itself; single quotes protect paths containing spaces.
    value += " '-Duser.home=" + user_home + "'"
    flags = [key + value, "-Porg.gradle.java.installations.paths="
             + ",".join(str(home) for home in homes.values() if home is not None),
             "-Porg.gradle.java.installations.auto-download=false"]
    if not arguments:
        remaining = ([":TeamCode:testDebugUnitTest", ":TeamCode:assembleDebug"] if official
                     else ["testDebugUnitTest", "assembleDebug"])
    return flags + remaining


def windows_command(wrapper, arguments, environment):
    # cmd.exe is required for .bat. Quote every argument, disable delayed expansion,
    # and reject characters that cmd could expand or use to break those quotes.
    words = [str(wrapper), *arguments]
    if any(any(character in word for character in '\r\n\x00"%!') for word in words):
        raise SetupError('Windows batch arguments and paths cannot contain quotes, %, !, or line breaks.')
    command = '"' + " ".join('"' + word + '"' for word in words) + '"'
    # Keep the /c payload verbatim: list2cmdline would escape its quotes for a
    # C runtime, but cmd.exe parses its command string with different rules.
    prefix = subprocess.list2cmdline([environment.get("COMSPEC", "cmd.exe"), "/d", "/v:off", "/s", "/c"])
    return prefix + " " + command


def doctor(project, tools, homes, official, environment):
    print("Project:", project)
    print("Layout:", "official FTC SDK" if official else "Dairy TeamCode")
    print("Tools:", tools)
    for key in ("JAVA_HOME", "ANDROID_HOME", "ANDROID_USER_HOME", "GRADLE_USER_HOME"):
        print(key + ":", environment[key])
    for version, home in homes.items():
        if home is None:
            continue
        executable = home / "bin" / ("java.exe" if os.name == "nt" else "java")
        result = subprocess.run([str(executable), "-version"], env=environment, cwd=project,
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if result.returncode:
            raise SetupError(f"JDK {version} could not report its version (exit {result.returncode}).")
        print(result.stdout.strip())
    for properties in sorted((tools / "sdk").glob("*/source.properties")):
        revision = read_property(properties, "Pkg.Revision")
        if revision:
            print(f"Android {properties.parent.name}: {revision}")
    for kind in ("build-tools", "platforms"):
        for directory in sorted((tools / "sdk" / kind).iterdir()):
            if directory.is_dir():
                print(f"Android {kind}: {directory.name}")
    print("Development tools are ready. No Gradle build or ADB command was run.")
    return 0


def main(arguments=None):
    arguments = list(sys.argv[1:] if arguments is None else arguments)
    if arguments == ["--help"]:
        print("Usage: python3 tools/run_gradle.py [--doctor | Gradle arguments...]\n"
              "No arguments runs unit tests and assembles the debug APK.\n"
              "Tools: FTC_TOOLS_HOME, then .ftc-tools.json tools_home, then project/.tools.")
        return 0
    try:
        if "--doctor" in arguments and arguments != ["--doctor"]:
            raise SetupError("Use --doctor on its own; otherwise pass Gradle arguments directly.")
        project = Path(__file__).resolve().parents[1]
        tools, homes, official, wrapper, environment = load_setup(project, os.environ)
        if arguments == ["--doctor"]:
            return doctor(project, tools, homes, official, environment)
        arguments = gradle_arguments(project, tools, homes, arguments, environment, official)
        command = (windows_command(wrapper, arguments, environment) if os.name == "nt"
                   else [str(wrapper), *arguments])
        for directory in ("android-user", "gradle-home"):
            (tools / directory).mkdir(parents=True, exist_ok=True)
        return subprocess.run(command, cwd=project, env=environment).returncode
    except SetupError as error:
        print("FTC tools: " + str(error), file=sys.stderr)
        return 2
    except OSError as error:
        print(f"FTC tools: cannot access or launch the local tools ({error.strerror}). "
              "Check paths and executable permissions; run --doctor.", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
