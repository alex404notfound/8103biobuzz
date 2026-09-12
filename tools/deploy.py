#!/usr/bin/env python3
"""Deploy with the same project-local Java/Android tools used for building."""
import os
from pathlib import Path
import re
import subprocess
import sys

import run_gradle


def parse_target(arguments):
    full = False
    address = None
    for argument in arguments:
        if argument == "--full" or argument.lower() == "-full":
            if full:
                raise run_gradle.SetupError("--full may only be specified once.")
            full = True
        elif argument.startswith("-"):
            raise run_gradle.SetupError(f"Unknown option: {argument}")
        elif address is not None:
            raise run_gradle.SetupError("Only one robot address may be specified.")
        else:
            address = argument
    address = "192.168.43.1" if address is None else address
    match = re.fullmatch(r"([A-Za-z0-9](?:[A-Za-z0-9.-]*[A-Za-z0-9])?)(?::([0-9]{1,5}))?", address)
    if match is None:
        raise run_gradle.SetupError("Robot address must be a host or host:port.")
    port = int(match.group(2) or "5555")
    if not 1 <= port <= 65535:
        raise run_gradle.SetupError("Robot port must be an integer from 1 to 65535.")
    return full, f"{match.group(1)}:{port}"


def deploy(project, arguments, environment):
    # Validate both CLI arguments and tool setup before contacting the selected robot.
    full, target = parse_target(arguments)
    tools, homes, official, wrapper, child_env = run_gradle.load_setup(project, environment)
    task = (":TeamCode:" if official else "") + ("installDebug" if full else "deploySloth")
    gradle_args = run_gradle.gradle_arguments(project, tools, homes, [task], child_env, official)
    command = (run_gradle.windows_command(wrapper, gradle_args, child_env) if os.name == "nt"
               else [str(wrapper), *gradle_args])
    adb = str(Path(child_env["ANDROID_HOME"]) / "platform-tools" / ("adb.exe" if os.name == "nt" else "adb"))
    child_env["ANDROID_SERIAL"] = target
    for directory in ("android-user", "gradle-home"):
        (tools / directory).mkdir(parents=True, exist_ok=True)
    connected = False
    try:
        result = subprocess.run([adb, "connect", target], cwd=project, env=child_env)
        if result.returncode:
            return result.returncode
        connected = True
        state = subprocess.run([adb, "-s", target, "get-state"], cwd=project, env=child_env,
                               stdout=subprocess.PIPE, text=True)
        if state.returncode:
            return state.returncode
        if state.stdout.strip() != "device":
            print(f'ADB target {target} is in state "{state.stdout.strip()}", not "device".', file=sys.stderr)
            return 1
        return subprocess.run(command, cwd=project, env=child_env).returncode
    finally:
        if connected:
            try:
                subprocess.run([adb, "disconnect", target], cwd=project, env=child_env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except (OSError, KeyboardInterrupt):
                pass  # Cleanup never replaces the original deployment status.


def main(arguments=None):
    arguments = list(sys.argv[1:] if arguments is None else arguments)
    if arguments == ["--help"]:
        print("Usage: python3 tools/deploy.py [--full] [robot-address]\n"
              "Uses FTC_TOOLS_HOME, .ftc-tools.json, or project/.tools.\n"
              "Run --full for the first install and after SDK, library, or resource changes.")
        return 0
    try:
        return deploy(Path(__file__).resolve().parents[1], arguments, os.environ)
    except run_gradle.SetupError as error:
        print("FTC deploy: " + str(error), file=sys.stderr)
        return 2
    except OSError as error:
        print(f"FTC deploy: cannot access or launch local tools ({error.strerror}).", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
