"""Run deployment paths against fake SDK adb and Gradle; never contact a robot."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_TARGET = '192.168.43.1:5555'
PWSH = shutil.which('pwsh')
if not PWSH and os.environ.get('FTC_TOOLS_HOME'):
    candidate = Path(os.environ['FTC_TOOLS_HOME']) / 'pwsh/pwsh'
    if candidate.is_file() and os.access(candidate, os.X_OK):
        PWSH = str(candidate)


@unittest.skipIf(os.name == 'nt', 'POSIX fake executables; PowerShell delegation is exercised where pwsh is available')
class LocalDeployTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='ftc deploy ')
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name).resolve()
        self.project = self.base / 'project with spaces'
        (self.project / 'tools').mkdir(parents=True)
        for name in ('tools/deploy.py', 'tools/run_gradle.py', 'push.sh', 'push.ps1'):
            if (PROJECT / name).exists():
                shutil.copy2(PROJECT / name, self.project / name)
        (self.project / 'settings.gradle').write_text("include ':FtcRobotController', ':TeamCode'\n")
        (self.project / 'FtcRobotController').mkdir()
        self.tools = self.base / 'shared tools'
        (self.project / '.ftc-tools.json').write_text(json.dumps({'tools_home': str(self.tools)}))
        self.log = self.base / 'calls.jsonl'
        for version in (8, 21, 25):
            for name in ('java', 'javac'):
                self.executable(self.tools / f'jdk{version}/bin/{name}', 'raise SystemExit(0)\n')
        for name in ('sdk/platforms/android-30/android.jar', 'sdk/build-tools/35.0.0/source.properties'):
            path = self.tools / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('placeholder\n')
        self.executable(self.tools / 'sdk/platform-tools/adb', r"""
import json, os, pathlib, sys
args = sys.argv[1:]
with pathlib.Path(os.environ['TEST_LOG']).open('a') as log:
    log.write(json.dumps({'tool': 'adb', 'args': args, 'serial': os.environ.get('ANDROID_SERIAL'), 'cwd': os.getcwd()}) + '\n')
if args[0] == 'connect':
    raise SystemExit(int(os.environ.get('CONNECT_EXIT', '0')))
if args[0] == 'disconnect':
    raise SystemExit(int(os.environ.get('DISCONNECT_EXIT', '0')))
if args[0] == '-s' and args[2] == 'get-state':
    print(os.environ.get('DEVICE_STATE', 'device'))
    raise SystemExit(int(os.environ.get('STATE_EXIT', '0')))
raise SystemExit(79)
""")
        self.executable(self.project / 'gradlew', r"""
import json, os, pathlib, sys
with pathlib.Path(os.environ['TEST_LOG']).open('a') as log:
    log.write(json.dumps({'tool': 'gradle', 'args': sys.argv[1:], 'serial': os.environ.get('ANDROID_SERIAL'), 'cwd': os.getcwd(), 'sdk': os.environ.get('ANDROID_HOME'), 'java': os.environ.get('JAVA_HOME')}) + '\n')
raise SystemExit(int(os.environ.get('GRADLE_EXIT', '0')))
""")
        self.env = os.environ.copy()
        self.env.pop('FTC_TOOLS_HOME', None)
        self.env.pop('GRADLE_OPTS', None)
        self.env.pop('JAVA_OPTS', None)
        self.env.update(TEST_LOG=str(self.log), PATH='', ANDROID_SERIAL='unrelated-device')

    @staticmethod
    def executable(path, body):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('#!' + sys.executable + '\n' + body)
        path.chmod(0o755)

    def run_deploy(self, *arguments, command=None):
        command = command or [sys.executable, str(self.project / 'tools/deploy.py')]
        return subprocess.run([*command, *arguments], cwd=self.base, env=self.env, capture_output=True, text=True)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_official_deploy_uses_configured_sdk_without_ambient_adb_from_any_cwd(self):
        result = self.run_deploy()
        self.assertEqual(0, result.returncode, result.stderr)
        calls = self.calls()
        self.assertEqual(['adb', 'adb', 'gradle', 'adb'], [call['tool'] for call in calls])
        self.assertEqual(['connect', DEFAULT_TARGET], calls[0]['args'])
        self.assertEqual(['-s', DEFAULT_TARGET, 'get-state'], calls[1]['args'])
        self.assertEqual(':TeamCode:deploySloth', calls[2]['args'][-1])
        self.assertEqual(['disconnect', DEFAULT_TARGET], calls[3]['args'])
        self.assertTrue(all(call['serial'] == DEFAULT_TARGET for call in calls))
        self.assertTrue(all(call['cwd'] == str(self.project) for call in calls))
        self.assertEqual(str(self.tools / 'sdk'), calls[2]['sdk'])
        self.assertEqual(str(self.tools / 'jdk21'), calls[2]['java'])
        self.assertEqual('unrelated-device', self.env['ANDROID_SERIAL'])

    def test_full_mode_uses_explicit_official_install_task_and_selected_port(self):
        result = self.run_deploy('--full', '10.8.1.2:6200')
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(':TeamCode:installDebug', self.calls()[2]['args'][-1])
        self.assertTrue(all(call['serial'] == '10.8.1.2:6200' for call in self.calls()))

    def test_dairy_layout_uses_unqualified_task(self):
        (self.project / 'settings.gradle').unlink()
        shutil.rmtree(self.project / 'FtcRobotController')
        result = self.run_deploy('-Full', 'robot.local')
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual('installDebug', self.calls()[2]['args'][-1])
        self.assertEqual('robot.local:5555', self.calls()[2]['serial'])

    def test_gradle_exit_42_survives_disconnect_failure(self):
        self.env.update(GRADLE_EXIT='42', DISCONNECT_EXIT='19')
        result = self.run_deploy()
        self.assertEqual(42, result.returncode)
        self.assertEqual(['disconnect', DEFAULT_TARGET], self.calls()[-1]['args'])

    def test_failed_connect_returns_original_status_without_gradle_or_disconnect(self):
        self.env['CONNECT_EXIT'] = '23'
        result = self.run_deploy()
        self.assertEqual(23, result.returncode)
        self.assertEqual([['connect', DEFAULT_TARGET]], [call['args'] for call in self.calls()])

    def test_offline_or_failed_state_disconnects_only_target_without_gradle(self):
        for values, status in (({'DEVICE_STATE': 'offline'}, 1), ({'STATE_EXIT': '24'}, 24)):
            with self.subTest(values=values):
                self.log.unlink(missing_ok=True)
                self.env.pop('DEVICE_STATE', None)
                self.env.pop('STATE_EXIT', None)
                self.env.update(values)
                result = self.run_deploy()
                self.assertEqual(status, result.returncode, result.stderr)
                self.assertEqual(['adb', 'adb', 'adb'], [call['tool'] for call in self.calls()])
                self.assertEqual(['disconnect', DEFAULT_TARGET], self.calls()[-1]['args'])

    def test_invalid_arguments_fail_before_any_network_or_gradle_command(self):
        invalid = [('--fast',), ('a', 'b'), ('bad/address',), ('robot:0',), ('robot:65536',),
                   ('robot:abc',), ('',), ('::1',), ('--full', '--full'), ('robot:000001',)]
        for arguments in invalid:
            with self.subTest(arguments=arguments):
                result = self.run_deploy(*arguments)
                self.assertEqual(2, result.returncode, result.stderr)
                self.assertEqual([], self.calls())

    def test_missing_local_tools_stops_before_connect(self):
        (self.tools / 'jdk21/bin/java').unlink()
        (self.tools / 'jdk25/bin/java').unlink()
        result = self.run_deploy()
        self.assertEqual(2, result.returncode)
        self.assertIn('JDK 21', result.stderr)
        self.assertIn('JDK 25', result.stderr)
        self.assertEqual([], self.calls())

    def test_posix_push_delegates_to_local_tools_helper(self):
        python_dir = self.base / 'python-bin'
        python_dir.mkdir()
        (python_dir / 'python3').symlink_to(sys.executable)
        self.env['PATH'] = str(python_dir) + os.pathsep + '/usr/bin:/bin'
        result = self.run_deploy('--full', command=['/bin/sh', str(self.project / 'push.sh')])
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(':TeamCode:installDebug', self.calls()[2]['args'][-1])

    @unittest.skipUnless(PWSH, 'PowerShell Core is not installed')
    def test_powershell_push_delegates_to_local_tools_helper(self):
        python_dir = self.base / 'python-bin'
        python_dir.mkdir()
        (python_dir / 'python3').symlink_to(sys.executable)
        self.env['PATH'] = str(python_dir) + os.pathsep + '/usr/bin:/bin'
        result = self.run_deploy('-Full', command=[PWSH, '-NoProfile', '-File', str(self.project / 'push.ps1')])
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(':TeamCode:installDebug', self.calls()[2]['args'][-1])


if __name__ == '__main__':
    unittest.main()
