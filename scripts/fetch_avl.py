"""Fetch pinned official binaries/source; no alternate solver fallback."""

from pathlib import Path
import hashlib
import json
import tarfile
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
BASE = 'https://web.mit.edu/drela/Public/web/avl/'


def main():
    out = ROOT / 'vendor/avl'
    out.mkdir(parents=True, exist_ok=True)
    files = [
        ('avl352.exe', 'avl.exe', '443520d255408491222a8df9060bd000f78da95a68845a2f6efdbc67e203f07a'),
        ('avl3.52.tgz', 'source.tgz', None),
    ]
    manifest = []
    for remote, name, expected in files:
        data = urlopen(BASE + remote, timeout=60).read()
        digest = hashlib.sha256(data).hexdigest()
        if expected is not None and digest != expected:
            raise ValueError('Official download changed: review its version before replacing the verified executable')
        (out / name).write_bytes(data)
        manifest.append({'file': name, 'url': BASE + remote, 'sha256': digest})
    with tarfile.open(out / 'source.tgz') as archive:
        for member in archive.getmembers():
            if not member.isfile():
                continue
            target = (out / member.name).resolve()
            if not target.is_relative_to(out.resolve()):
                raise ValueError('Source archive contains an unsafe path')
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.extractfile(member).read())
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf8')
    print('Official AVL files fetched and recorded. GPL terms apply to AVL.')


if __name__ == '__main__':
    main()
