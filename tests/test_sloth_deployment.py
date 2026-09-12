"""Opt-in Gradle integration tests; every ADB/install action is replaced by a fake.

Set FTC_SLOTH_TEST_SDK to an official SDK checkout and FTC_TOOLS_HOME to local
installed tools. These tests never connect to or install on a real device.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

PROJECT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('FTC_SLOTH_TEST_SDK') and os.environ.get('FTC_TOOLS_HOME'),
                     'Set FTC_SLOTH_TEST_SDK and FTC_TOOLS_HOME for fake-only Gradle integration')
class Sloth03DeploymentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='sloth gradle9 ')
        cls.base = Path(cls.temp.name).resolve()
        cls.sdk = cls.base / 'sdk'
        cls.sdk.mkdir()
        original = Path(os.environ['FTC_SLOTH_TEST_SDK'])
        for name in ('build.gradle', 'settings.gradle', 'build.common.gradle', 'build.dependencies.gradle',
                     'gradle.properties', 'gradlew', 'gradlew.bat', 'FtcRobotController/build.gradle',
                     'FtcRobotController/src/main/AndroidManifest.xml', 'TeamCode/src/main/AndroidManifest.xml'):
            target = cls.sdk / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original / name, target)
        for name in ('gradle', 'libs'):
            shutil.copytree(original / name, cls.sdk / name)
        shutil.copy2(PROJECT / 'TeamCode/build.gradle', cls.sdk / 'TeamCode/build.gradle')
        (cls.sdk / 'tools').mkdir()
        shutil.copy2(PROJECT / 'tools/run_gradle.py', cls.sdk / 'tools/run_gradle.py')
        (cls.sdk / 'TeamCode/build/libs').mkdir(parents=True)
        (cls.sdk / 'TeamCode/build/libs/to_load.jar').write_bytes(b'fake packaged code')
        cls.log = cls.base / 'commands.jsonl'
        cls.fake = cls.base / 'fake-adb'
        cls.fake.write_text('#!' + sys.executable + '\n' + r"""
import json, os, pathlib, sys
args = sys.argv[1:]
with pathlib.Path(os.environ['FTC_TEST_LOG']).open('a') as log:
    log.write(json.dumps({'args': args, 'serial': os.environ.get('ANDROID_SERIAL')}) + '\n')
mode = os.environ.get('FTC_TEST_FAILURE', '')
if any(arg.startswith('if [ -d ') for arg in args):
    print('123456.jar\n-789.jar\nloaded.jar\nto_load.jar\nsloth.lock\nnotes.txt\n123notes.jar\nhistory')
if 'push' in args and mode == 'push':
    print('simulated push failure', file=sys.stderr)
    raise SystemExit(42)
if any(arg.startswith('rm -f ') for arg in args) and mode == 'remove':
    print('simulated remove failure', file=sys.stderr)
    raise SystemExit(43)
if any(arg.startswith('test ! -f ') for arg in args) and mode == 'lock':
    raise SystemExit(1)
""")
        cls.fake.chmod(0o755)
        cls.init = cls.base / 'fake-only.init.gradle'
        cls.init.write_text(r"""
gradle.projectsEvaluated {
    def team = rootProject.project(':TeamCode')
    def fakeAdb = new File(System.getenv('FTC_TEST_ADB'))
    assert fakeAdb.isFile()
    ['deploySloth', 'removeSlothRemote'].each { name ->
        team.tasks.named(name).get().adbExecutable.fileValue(fakeAdb)
    }
    assert team.extensions.getByName('load').autoconnect.name() == 'NEVER'
    def deploy = team.tasks.named('deploySloth').get()
    assert deploy.taskDependencies.getDependencies(deploy)*.name.contains('assembleSloth')
    def install = team.tasks.named('installDebug').get()
    assert install.taskDependencies.getDependencies(install)*.name.contains('removeSlothRemote')
    install.setActions([])
    install.doLast {
        new File(System.getenv('FTC_TEST_LOG')).append('{"install": true}\n')
    }
}
gradle.taskGraph.whenReady { graph ->
    graph.allTasks.each { task ->
        if (!(task.path in [':TeamCode:deploySloth', ':TeamCode:removeSlothRemote', ':TeamCode:installDebug'])) {
            task.enabled = false
        }
    }
}
""")

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_fake(self, task, failure='', serial='192.0.2.81:5555'):
        self.log.unlink(missing_ok=True)
        env = os.environ.copy()
        env.update(FTC_TEST_ADB=str(self.fake), FTC_TEST_LOG=str(self.log), FTC_TEST_FAILURE=failure)
        if serial is None:
            env.pop('ANDROID_SERIAL', None)
        else:
            env['ANDROID_SERIAL'] = serial
        result = subprocess.run([sys.executable, str(self.sdk / 'tools/run_gradle.py'),
                                 '--offline', '--console=plain', '-I', str(self.init), task],
                                cwd=self.base, env=env, capture_output=True, text=True)
        calls = [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []
        return result, calls

    def test_deploy_runs_exact_lock_push_lock_protocol_with_selected_serial(self):
        result, calls = self.run_fake(':TeamCode:deploySloth')
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        self.assertEqual(3, len(calls))
        self.assertTrue(all(call['args'][:2] == ['-s', '192.0.2.81:5555'] for call in calls))
        self.assertTrue(all(call['serial'] == '192.0.2.81:5555' for call in calls))
        self.assertEqual(['shell', 'test ! -f /storage/emulated/0/FIRST/dairy/sloth/sloth.lock'], calls[0]['args'][2:])
        self.assertEqual(['push', str(self.sdk / 'TeamCode/build/libs/to_load.jar')], calls[1]['args'][2:4])
        self.assertRegex(calls[1]['args'][4], r'^/storage/emulated/0/FIRST/dairy/sloth/-?[0-9]+\.jar$')
        self.assertEqual(calls[0]['args'], calls[2]['args'])

    def test_install_debug_removes_all_old_reload_files_before_stubbed_install(self):
        result, calls = self.run_fake(':TeamCode:installDebug')
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        self.assertEqual(3, len(calls), result.stdout + result.stderr)
        self.assertTrue(calls[0]['args'][3].startswith('if [ -d '))
        self.assertEqual(['-s', '192.0.2.81:5555', 'shell',
                          "rm -f '/storage/emulated/0/FIRST/dairy/sloth/123456.jar' "
                          "'/storage/emulated/0/FIRST/dairy/sloth/-789.jar' "
                          "'/storage/emulated/0/FIRST/dairy/sloth/loaded.jar' "
                          "'/storage/emulated/0/FIRST/dairy/sloth/to_load.jar' "
                          "'/storage/emulated/0/FIRST/dairy/sloth/sloth.lock'"], calls[1]['args'])
        self.assertEqual({'install': True}, calls[2])

    def test_push_failure_and_existing_lock_stop_before_further_device_commands(self):
        for failure, count in (('push', 2), ('lock', 1)):
            with self.subTest(failure=failure):
                result, calls = self.run_fake(':TeamCode:deploySloth', failure=failure)
                self.assertNotEqual(0, result.returncode)
                self.assertEqual(count, len(calls), result.stdout + result.stderr)
                if failure == 'push':
                    self.assertIn('simulated push failure', result.stdout + result.stderr)

    def test_failed_removal_prevents_install_and_missing_serial_prevents_adb(self):
        result, calls = self.run_fake(':TeamCode:installDebug', failure='remove')
        self.assertNotEqual(0, result.returncode)
        self.assertEqual(2, len(calls))
        self.assertIn('simulated remove failure', result.stdout + result.stderr)
        result, calls = self.run_fake(':TeamCode:deploySloth', serial=None)
        self.assertNotEqual(0, result.returncode)
        self.assertEqual([], calls)
        self.assertIn('ANDROID_SERIAL', result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
