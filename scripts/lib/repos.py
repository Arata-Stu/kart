"""Update manifest branches without trying to pull detached, pinned revisions."""
import re
import subprocess
import sys
from pathlib import Path



def git(folder, *args, check=True):
    return subprocess.run(["git", "-C", str(folder), *args], check=check,
                          text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def update(folder, version):
    if not (folder / ".git").exists():
        raise ValueError("未取得です。先に repos.sh import を実行してください")
    if re.fullmatch(r"[0-9a-fA-F]{40}", version):
        current = git(folder, "rev-parse", "HEAD").stdout.strip()
        print(f"SKIP commit固定: {version} (HEAD={current})", flush=True)
        if current != version:
            print("注意: HEADはpackages.reposの固定commitと異なります。自動checkoutはしません。", flush=True)
        return
    if git(folder, "status", "--porcelain").stdout:
        raise ValueError("未コミット変更があります。変更を保存してから再実行してください")
    branch = git(folder, "symbolic-ref", "--quiet", "--short", "HEAD", check=False).stdout.strip()
    if branch and branch != version:
        raise ValueError(f"現在のbranch={branch}、指定branch={version}。自動切替はしません")
    git(folder, "check-ref-format", "--branch", version)
    git(folder, "fetch", "origin", f"refs/heads/{version}:refs/remotes/origin/{version}")
    remote = "refs/remotes/origin/" + version
    if not branch:
        if git(folder, "merge-base", "--is-ancestor", "HEAD", remote, check=False).returncode:
            raise ValueError("detached HEADに追従先にないcommitがあります。自動切替はしません")
        exists = git(folder, "show-ref", "--verify", "--quiet", "refs/heads/" + version, check=False).returncode == 0
        if exists:
            git(folder, "merge-base", "--is-ancestor", version, remote)
            git(folder, "switch", version)
        else:
            git(folder, "switch", "--track", "-c", version, "origin/" + version)
        print(f"detached HEAD → branch {version}", flush=True)
    result = git(folder, "merge", "--ff-only", remote)
    print(result.stdout.strip(), flush=True)


def main():
    import yaml

    root = Path(__file__).resolve().parents[2]
    repos = yaml.safe_load((root / "packages.repos").read_text())["repositories"]
    failed = False
    for relative, entry in repos.items():
        print(f"=== {relative} ===", flush=True)
        try:
            folder = (root / relative).resolve()
            if not folder.is_relative_to(root) or entry["type"] != "git":
                raise ValueError("不正なrepository設定")
            update(folder, str(entry["version"]))
        except (ValueError, subprocess.CalledProcessError) as error:
            failed = True
            print(getattr(error, "stderr", "") or str(error), file=sys.stderr)
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
