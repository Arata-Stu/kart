"""Build wrapper contracts with a fake colcon; no ROS, Docker or compilation."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

SCRIPT=Path(__file__).resolve().parents[1]/'build.sh'


class BuildTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        (self.root/'scripts').mkdir();(self.root/'ros2_ws').mkdir()
        shutil.copyfile(SCRIPT,self.root/'scripts/build.sh')
        self.bin=self.root/'bin';self.bin.mkdir()
        colcon=self.bin/'colcon'
        colcon.write_text(f'#!{sys.executable}\n'+'''import json,os,sys
print(json.dumps({'args':sys.argv[1:],'cwd':os.getcwd(),'make':os.getenv('MAKEFLAGS'),'cmake':os.getenv('CMAKE_BUILD_PARALLEL_LEVEL')}))
''');colcon.chmod(0o755)
        uname=self.bin/'uname'
        uname.write_text('#!/bin/sh\ncase "$1" in -s) echo "${BUILD_TEST_SYSTEM:-Linux}";; -m) echo "${BUILD_TEST_ARCH:-x86_64}";; -r) echo "${BUILD_TEST_KERNEL:-generic}";; esac\n')
        uname.chmod(0o755)
        self.env=dict(os.environ,PATH=str(self.bin)+os.pathsep+os.environ['PATH'])
        for key in ('MAKEFLAGS','CMAKE_BUILD_PARALLEL_LEVEL','BUILD_TEST_SYSTEM','BUILD_TEST_ARCH','BUILD_TEST_KERNEL'):
            self.env.pop(key,None)

    def run_build(self,*args):
        return subprocess.run(['bash',str(self.root/'scripts/build.sh'),*args],env=self.env,text=True,capture_output=True)

    def result(self,*args):
        result=self.run_build(*args)
        self.assertEqual(result.returncode,0,result.stderr)
        return json.loads(result.stdout.splitlines()[-1])

    def test_desktop_keeps_passthrough_and_workspace(self):
        result=self.result('--packages-up-to','kart_bringup','--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DSOME_PATH=/a b')
        self.assertEqual(result['args'],['build','--symlink-install','--packages-up-to','kart_bringup','--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DSOME_PATH=/a b'])
        self.assertEqual(result['cwd'],str((self.root/'ros2_ws').resolve()))
        self.assertIsNone(result['make']);self.assertIsNone(result['cmake'])

    def test_auto_detects_tegra_and_limits_both_levels(self):
        self.env.update(BUILD_TEST_ARCH='aarch64',BUILD_TEST_KERNEL='6.8.12-tegra')
        result=self.result('--packages-up-to','kart_bringup')
        self.assertEqual(result['args'],['build','--symlink-install','--executor','sequential','--packages-ignore','kart_sim','--packages-up-to','kart_bringup'])
        self.assertEqual(result['make'],'-j2 -l2');self.assertEqual(result['cmake'],'2')

    def test_force_profile_inside_container_and_override_detection(self):
        self.assertIn('kart_sim',self.result('--profile','jetson')['args'])
        self.env.update(BUILD_TEST_ARCH='aarch64',BUILD_TEST_KERNEL='tegra')
        self.assertNotIn('--packages-ignore',self.result('--profile','desktop')['args'])

    def test_explicit_sim_and_with_sim_are_supported(self):
        for selection in ('--packages-select','--packages-up-to','--packages-above-and-dependencies'):
            result=self.result('--profile','jetson',selection,'kart_sim','kart_bringup')
            self.assertNotIn('--packages-ignore',result['args'])
        self.assertNotIn('--packages-ignore',self.result('--profile','jetson','--with-sim')['args'])
        self.assertNotIn('--packages-ignore',self.result('--profile','jetson','--packages-select=kart_sim')['args'])

    def test_without_sim_rejects_contradictory_selection(self):
        result=self.run_build('--without-sim','--packages-select','kart_sim')
        self.assertEqual(result.returncode,2);self.assertIn('conflicts',result.stderr)
        self.assertIn('--packages-ignore',self.result('--without-sim')['args'])

    def test_existing_ignore_list_merges_without_losing_defaults(self):
        for option in (['--packages-ignore','other'],['--packages-ignore=other']):
            args=self.result('--profile','jetson',*option,'--packages-up-to','kart_bringup')['args']
            self.assertEqual(args.count('--packages-ignore'),1)
            at=args.index('--packages-ignore')
            self.assertEqual(args[at+1:at+3],['kart_sim','other'])
        args=self.result('--profile','jetson','--packages-ignore','one','--packages-ignore','two')['args']
        self.assertEqual(args.count('kart_sim'),2) # Last argparse occurrence still contains default.

    def test_explicit_parallelism_and_environment_win(self):
        self.env.update(MAKEFLAGS='-j4 -l4',CMAKE_BUILD_PARALLEL_LEVEL='4')
        result=self.result('--profile','jetson','--executor','parallel','--parallel-workers','2')
        self.assertNotIn('sequential',result['args'])
        self.assertEqual(result['make'],'-j4 -l4');self.assertEqual(result['cmake'],'4')
        self.assertNotIn('--executor',self.result('--profile','jetson','--parallel-workers=2')['args'])
        result=self.result('--profile','jetson','--jobs','1')
        self.assertEqual(result['cmake'],'1');self.assertTrue(result['make'].endswith('-j1 -l1'))

    def test_cmake_payload_is_not_reinterpreted(self):
        args=self.result('--profile','jetson','--cmake-args','-DNAME=kart_sim',' --packages-ignore',' --help')['args']
        self.assertEqual(args[-4:],['--cmake-args','-DNAME=kart_sim',' --packages-ignore',' --help'])
        self.assertIn('--packages-ignore',args[:-4])

    def test_help_and_invalid_options(self):
        self.assertEqual(self.run_build('--help').returncode,0)
        for args in (('--profile','wrong'),('--jobs','0'),('--jobs','-2'),('--jobs','abc'),('--jobs',)):
            self.assertNotEqual(self.run_build(*args).returncode,0)



@unittest.skipUnless(shutil.which('colcon') and shutil.which('cmake'), 'colcon/CMake unavailable')
class BuildDependencyTests(unittest.TestCase):
    def test_clean_workspace_with_optional_unbuilt_sim(self):
        """Real colcon graph + CMake install, using tiny non-ROS fixture packages."""
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'scripts').mkdir();(root/'ros2_ws/src').mkdir(parents=True)
            shutil.copyfile(SCRIPT,root/'scripts/build.sh')
            for name in ('kart_sim','kart_bringup'):
                package=root/'ros2_ws/src'/name;package.mkdir()
                dependency='<exec_depend>kart_sim</exec_depend>' if name=='kart_bringup' else ''
                (package/'package.xml').write_text(f'''<package format="3"><name>{name}</name><version>0.0.0</version>
<description>Isolated build wrapper fixture</description><maintainer email="test@example.com">Test</maintainer>
<license>MIT</license>{dependency}<export><build_type>cmake</build_type></export></package>''')
                (package/'fixture.txt').write_text('fixture')
                (package/'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.20)
project({name} NONE)
install(FILES fixture.txt DESTINATION share/{name})
''')
            env=dict(os.environ)
            env['COLCON_HOME']=str(root/'colcon-home')
            for key in ('MAKEFLAGS','CMAKE_BUILD_PARALLEL_LEVEL','COLCON_PREFIX_PATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH'):
                env.pop(key,None)
            for flags,expected in [([],False),(['--with-sim'],True)]:
                result=subprocess.run(['bash',str(root/'scripts/build.sh'),'--profile','jetson',*flags,
                    '--packages-up-to','kart_bringup'],env=env,text=True,capture_output=True,timeout=60)
                self.assertEqual(result.returncode,0,result.stdout+'\n'+result.stderr)
                self.assertTrue((root/'ros2_ws/install/kart_bringup/share/kart_bringup/fixture.txt').is_file())
                self.assertEqual((root/'ros2_ws/install/kart_sim/share/kart_sim/fixture.txt').is_file(),expected)


if __name__=='__main__':unittest.main()
