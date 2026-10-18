import importlib.util
import subprocess
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location('repos', Path(__file__).resolve().parents[1] / 'lib/repos.py')
repos = importlib.util.module_from_spec(spec)
spec.loader.exec_module(repos)


class ReposTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.origin, self.work = root / 'origin', root / 'work'
        subprocess.run(['git', 'init', '-b', 'kart', str(self.origin)], check=True, capture_output=True)
        repos.git(self.origin, 'config', 'user.email', 'test@example.com')
        repos.git(self.origin, 'config', 'user.name', 'Test')
        repos.git(self.origin, 'commit', '--allow-empty', '-m', 'first')
        self.first = repos.git(self.origin, 'rev-parse', 'HEAD').stdout.strip()
        subprocess.run(['git', 'clone', str(self.origin), str(self.work)], check=True, capture_output=True)
        repos.git(self.work, 'checkout', '--detach')
        repos.git(self.origin, 'commit', '--allow-empty', '-m', 'second')

    def test_detached_branch_is_restored_and_fast_forwarded(self):
        repos.update(self.work, 'kart')
        self.assertEqual(repos.git(self.work, 'branch', '--show-current').stdout.strip(), 'kart')
        self.assertEqual(repos.git(self.work, 'rev-parse', 'HEAD').stdout, repos.git(self.origin, 'rev-parse', 'HEAD').stdout)

    def test_pin_is_left_untouched(self):
        repos.update(self.work, self.first)
        self.assertEqual(repos.git(self.work, 'rev-parse', 'HEAD').stdout.strip(), self.first)
        self.assertEqual(repos.git(self.work, 'branch', '--show-current').stdout.strip(), '')

    def test_dirty_worktree_is_preserved(self):
        (self.work / 'local.txt').write_text('keep')
        with self.assertRaises(ValueError):
            repos.update(self.work, 'kart')
        self.assertEqual((self.work / 'local.txt').read_text(), 'keep')

    def test_unique_detached_commit_is_preserved(self):
        repos.git(self.work, 'config', 'user.email', 'test@example.com')
        repos.git(self.work, 'config', 'user.name', 'Test')
        repos.git(self.work, 'commit', '--allow-empty', '-m', 'local')
        head = repos.git(self.work, 'rev-parse', 'HEAD').stdout
        with self.assertRaises(ValueError):
            repos.update(self.work, 'kart')
        self.assertEqual(repos.git(self.work, 'rev-parse', 'HEAD').stdout, head)


if __name__ == '__main__':
    unittest.main()
