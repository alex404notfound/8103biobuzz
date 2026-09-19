"""Exercise the local Gradle launcher without downloading or building anything."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

RUNNER = Path(__file__).resolve().parents[1] / "tools" / "run_gradle.py"


@unittest.skipIf(os.name == "nt", "POSIX fake executables; Windows quoting has separate tests")
class LocalGradleRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ftc runner ")
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name).resolve()
        self.project = self.base / "project"
        (self.project / "tools").mkdir(parents=True)
        if RUNNER.exists():
            shutil.copy2(RUNNER, self.project / "tools/run_gradle.py")
        (self.project / "build.gradle.kts").write_text("// Dairy project\n")
        (self.project / "gradle.properties").write_text("org.gradle.jvmargs=-Xmx1536M -Dfile.encoding=UTF-8\n")
        self.capture = self.base / "gradle-call.json"
        self.executable(self.project / "gradlew", """
import json, os, pathlib, sys
pathlib.Path(os.environ["FTC_TEST_CAPTURE"]).write_text(json.dumps({
 "args": sys.argv[1:], "cwd": os.getcwd(),
 "env": {key: os.environ.get(key) for key in (
  "JAVA_HOME", "ANDROID_HOME", "ANDROID_SDK_ROOT", "ANDROID_USER_HOME",
  "GRADLE_USER_HOME", "PATH", "HOME")}}))
raise SystemExit(int(os.environ.get("FTC_TEST_EXIT", "0")))
""")
        self.tools = self.project / ".tools"
        self.make_tools(self.tools)
        self.env = os.environ.copy()
        self.env.pop("FTC_TOOLS_HOME", None)
        self.env.pop("GRADLE_OPTS", None)
        self.env.pop("JAVA_OPTS", None)
        self.env["FTC_TEST_CAPTURE"] = str(self.capture)

    def executable(self, path, body):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("#!" + sys.executable + "\n" + body)
        path.chmod(0o755)

    def make_tools(self, tools):
        for version, home in ((8, tools / "jdk8/Contents/Home"), (21, tools / "jdk21"), (25, tools / "jdk25")):
            for binary in ("java", "javac"):
                self.executable(home / "bin" / binary, f'print("fake {binary} {version}")\n')
        for file in ("sdk/platforms/android-30/android.jar", "sdk/build-tools/34.0.0/source.properties", "sdk/platform-tools/adb"):
            target = tools / file
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text("Pkg.Revision=34.0.0\n")
        (tools / "sdk/platform-tools/source.properties").write_text("Pkg.Revision=37.0.1\n")

    def run_runner(self, *args):
        return subprocess.run([sys.executable, str(self.project / "tools/run_gradle.py"), *args],
                              cwd=self.base, env=self.env, text=True, capture_output=True)

    def captured(self):
        return json.loads(self.capture.read_text())

    def test_default_dairy_build_uses_local_tools_and_script_project(self):
        result = self.run_runner()
        self.assertEqual(result.returncode, 0, result.stderr)
        call = self.captured()
        self.assertEqual(call["cwd"], str(self.project))
        self.assertEqual(call["args"][-2:], ["testDebugUnitTest", "assembleDebug"])
        env = call["env"]
        self.assertEqual(env["JAVA_HOME"], str(self.tools / "jdk21"))
        self.assertEqual(env["ANDROID_HOME"], str(self.tools / "sdk"))
        self.assertEqual(env["ANDROID_SDK_ROOT"], str(self.tools / "sdk"))
        self.assertEqual(env["ANDROID_USER_HOME"], str(self.tools / "android-user"))
        self.assertEqual(env["GRADLE_USER_HOME"], str(self.tools / "gradle-home"))
        self.assertEqual(env["HOME"], self.env.get("HOME"))
        self.assertIn(str(self.tools / "sdk/platform-tools"), env["PATH"].split(os.pathsep))
        toolchains = next(a for a in call["args"] if a.startswith("-Porg.gradle.java.installations.paths="))
        self.assertIn(str(self.tools / "jdk8/Contents/Home"), toolchains)
        self.assertIn(str(self.tools / "jdk21"), toolchains)
        jvmargs = next(a for a in call["args"] if a.startswith("-Dorg.gradle.jvmargs="))
        self.assertIn("-Xmx1536M", jvmargs)
        self.assertIn("-Dfile.encoding=UTF-8", jvmargs)
        self.assertIn("-Duser.home=" + str(self.tools / "android-user"), jvmargs)

    def test_official_sdk_uses_java21_without_requiring_java8_or_java25(self):
        (self.project / "settings.gradle").write_text("include ':FtcRobotController', ':TeamCode'\n")
        (self.project / "FtcRobotController").mkdir()
        shutil.rmtree(self.tools / "jdk8")
        shutil.rmtree(self.tools / "jdk25")
        result = self.run_runner()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.captured()["args"][-2:], [":TeamCode:testDebugUnitTest", ":TeamCode:assembleDebug"])
        self.assertEqual(self.captured()["env"]["JAVA_HOME"], str(self.tools / "jdk21"))

    def test_official_sdk_falls_back_to_existing_java25(self):
        (self.project / "settings.gradle").write_text("include ':FtcRobotController', ':TeamCode'\n")
        (self.project / "FtcRobotController").mkdir()
        shutil.rmtree(self.tools / "jdk8")
        shutil.rmtree(self.tools / "jdk21")
        result = self.run_runner()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.captured()["env"]["JAVA_HOME"], str(self.tools / "jdk25"))

    def test_official_sdk_rejects_missing_java21_and_java25_before_starting_gradle(self):
        (self.project / "settings.gradle").write_text("include ':FtcRobotController', ':TeamCode'\n")
        (self.project / "FtcRobotController").mkdir()
        shutil.rmtree(self.tools / "jdk21")
        shutil.rmtree(self.tools / "jdk25")
        result = self.run_runner("help")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("JDK 21", result.stderr)
        self.assertIn("JDK 25", result.stderr)
        self.assertFalse(self.capture.exists())

    def test_explicit_arguments_and_nonzero_gradle_exit_are_preserved(self):
        self.env["FTC_TEST_EXIT"] = "42"
        result = self.run_runner(":custom:check", "-Pmessage=a value", "--stacktrace")
        self.assertEqual(result.returncode, 42, result.stderr)
        self.assertEqual(self.captured()["args"][-3:], [":custom:check", "-Pmessage=a value", "--stacktrace"])

    def test_config_selects_shared_tools(self):
        shared = self.base / "shared tools"
        self.tools.rename(shared)
        (self.project / ".ftc-tools.json").write_text(json.dumps({"tools_home": str(shared)}))
        result = self.run_runner("help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.captured()["env"]["JAVA_HOME"], str(shared / "jdk21"))

    def test_environment_override_takes_precedence_over_config(self):
        shared = self.base / "env tools"
        self.tools.rename(shared)
        (self.project / ".ftc-tools.json").write_text("invalid configuration ignored with override")
        self.env["FTC_TOOLS_HOME"] = str(shared)
        result = self.run_runner("help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.captured()["env"]["GRADLE_USER_HOME"], str(shared / "gradle-home"))

    def test_missing_java21_gives_actionable_error_without_starting_gradle(self):
        shutil.rmtree(self.tools / "jdk21")
        result = self.run_runner("help")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("JDK 21", result.stderr)
        self.assertIn("FTC_TOOLS_HOME", result.stderr)
        self.assertFalse(self.capture.exists())

    def test_dairy_requires_java8_compiler(self):
        (self.tools / "jdk8/Contents/Home/bin/javac").unlink()
        result = self.run_runner("help")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("JDK 8", result.stderr)
        self.assertFalse(self.capture.exists())

    def test_missing_android_platform_stops_before_gradle(self):
        (self.tools / "sdk/platforms/android-30/android.jar").unlink()
        result = self.run_runner("help")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("platforms;android-30", result.stderr)
        self.assertFalse(self.capture.exists())

    def test_malformed_or_relative_config_is_not_silently_ignored(self):
        for contents in ("{broken", "[]", '{"tools_home":"relative/path"}', '{"tools_home":42}', "{}"):
            with self.subTest(contents=contents):
                (self.project / ".ftc-tools.json").write_text(contents)
                result = self.run_runner("help")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(".ftc-tools.json", result.stderr)
                self.assertFalse(self.capture.exists())

    def test_doctor_is_read_only_and_never_launches_gradle_or_adb(self):
        result = self.run_runner("--doctor")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("fake java 21", result.stdout)
        self.assertIn("37.0.1", result.stdout)
        self.assertIn(str(self.tools), result.stdout)
        self.assertFalse(self.capture.exists())
        self.assertFalse((self.tools / "gradle-home").exists())
        self.assertFalse((self.tools / "android-user").exists())

    def test_explicit_jvm_heap_setting_is_preserved_without_duplicate_override(self):
        result = self.run_runner("-Dorg.gradle.jvmargs=-Xmx2g -Dcustom=private-value", "help")
        self.assertEqual(result.returncode, 0, result.stderr)
        jvm = [a for a in self.captured()["args"] if a.startswith("-Dorg.gradle.jvmargs=")]
        self.assertEqual(len(jvm), 1)
        self.assertIn("-Xmx2g", jvm[0])
        self.assertNotIn("-Xmx1536M", jvm[0])
        self.assertIn("-Duser.home=", jvm[0])
        self.assertNotIn("private-value", result.stdout + result.stderr)

    def test_only_explicit_jvm_arguments_do_not_invent_build_tasks(self):
        result = self.run_runner("-Dorg.gradle.jvmargs=-Xmx2g")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("assembleDebug", self.captured()["args"])
        self.assertNotIn("testDebugUnitTest", self.captured()["args"])

    def test_empty_build_tools_installation_is_rejected(self):
        (self.tools / "sdk/build-tools/34.0.0/source.properties").unlink()
        result = self.run_runner("help")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("build-tools", result.stderr)
        self.assertFalse(self.capture.exists())

    def test_nonexecutable_java_is_rejected_before_gradle(self):
        (self.tools / "jdk21/bin/java").chmod(0o644)
        result = self.run_runner("help")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("JDK 21", result.stderr)
        self.assertFalse(self.capture.exists())

    def test_shared_gradle_properties_heap_overrides_project_heap(self):
        (self.tools / "gradle-home").mkdir()
        (self.tools / "gradle-home/gradle.properties").write_text("org.gradle.jvmargs=-Xmx3g\n")
        result = self.run_runner("help")
        self.assertEqual(result.returncode, 0, result.stderr)
        jvm = next(a for a in self.captured()["args"] if a.startswith("-Dorg.gradle.jvmargs="))
        self.assertIn("-Xmx3g", jvm)
        self.assertNotIn("-Xmx1536M", jvm)


class WindowsBoundaryTests(unittest.TestCase):
    def module(self):
        self.assertTrue(RUNNER.exists(), "local Gradle runner is not implemented")
        spec = importlib.util.spec_from_file_location("run_gradle_test_module", RUNNER)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_windows_arguments_are_quoted_and_cmd_expansion_is_rejected(self):
        module = self.module()
        wrapper = Path("C:/team robot/gradlew.bat")
        command = module.windows_command(wrapper, ["help", "-Pmessage=a&b"], {"COMSPEC": "cmd.exe"})
        self.assertIsInstance(command, str)
        self.assertEqual(command, f'cmd.exe /d /v:off /s /c ""{wrapper}" "help" "-Pmessage=a&b""')
        for unsafe in ('%PATH%', 'hello\nwhoami', 'a"&whoami'):
            with self.subTest(unsafe=unsafe):
                with self.assertRaises(ValueError):
                    module.windows_command(Path("gradlew.bat"), [unsafe], {})

    def test_windows_java_home_detection_accepts_exe_layout(self):
        module = self.module()
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp) / "jdk21"
            (home / "bin").mkdir(parents=True)
            (home / "bin/java.exe").touch()
            (home / "bin/javac.exe").touch()
            self.assertEqual(module.find_java_home(home, windows=True), home)

    def test_official_sdk_java_selection_on_current_platform(self):
        module = self.module()
        for available, selected in (((21, 25), 21), ((21,), 21), ((25,), 25), ((), None)):
            with self.subTest(available=available), tempfile.TemporaryDirectory() as temp:
                project = Path(temp)
                tools = project / ".tools"
                (project / "settings.gradle").touch()
                (project / "FtcRobotController").mkdir()
                (project / ("gradlew.bat" if os.name == "nt" else "gradlew")).touch()
                suffix = ".exe" if os.name == "nt" else ""
                files = ["sdk/platforms/android-30/android.jar",
                         "sdk/build-tools/35.0.0/source.properties", "sdk/platform-tools/adb" + suffix]
                files += [f"jdk{version}/bin/{binary}{suffix}"
                          for version in available for binary in ("java", "javac")]
                for filename in files:
                    path = tools / filename
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.touch()
                    path.chmod(0o755)
                if selected is None:
                    with self.assertRaisesRegex(module.SetupError, "JDK 21 .* or JDK 25"):
                        module.load_setup(project, {})
                else:
                    _, _, official, _, environment = module.load_setup(project, {})
                    self.assertTrue(official)
                    self.assertEqual(environment["JAVA_HOME"], str(tools / f"jdk{selected}"))
                self.assertFalse((tools / "jdk8").exists())


if __name__ == "__main__":
    unittest.main()
