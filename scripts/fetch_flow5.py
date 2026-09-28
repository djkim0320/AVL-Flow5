"""Install the pinned official portable flow5 release inside this project."""
from pathlib import Path
import hashlib
import json
import urllib.request
import zipfile

ROOT=Path(__file__).resolve().parents[1]
URL='https://github.com/techwinder/flow5/releases/download/v7.57/flow5_v7.57_win64.zip'
SHA256='dd60fafacaf5ba521dc9fbb6d9737f7eaa8dcfc875b1f333b66772487bd5a465'
EXE_SHA256='7cdf0702bb5cda76a5cc96f0b3e48aac8704e9f9b78f7acb35a87113e4cdada2'


def main():
    directory=ROOT/'vendor/flow5';directory.mkdir(parents=True,exist_ok=True)
    archive=directory/'flow5_v7.57_win64.zip'
    if not archive.exists():urllib.request.urlretrieve(URL,archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest()!=SHA256:raise ValueError('Official archive differs from pinned checksum')
    executable=directory/'bin/flow5_v7.57_win64/flow5.exe'
    if not executable.exists():
        with zipfile.ZipFile(archive) as z:
            target=(directory/'bin').resolve()
            if any(not (target/n).resolve().is_relative_to(target) for n in z.namelist()):raise ValueError('Unsafe archive path')
            z.extractall(target)
    if hashlib.sha256(executable.read_bytes()).hexdigest()!=EXE_SHA256:raise ValueError('Installed executable checksum differs')
    source_url='https://codeload.github.com/techwinder/flow5/zip/refs/tags/v7.57'
    source=directory/'flow5_v7.57_source.zip'
    if not source.exists():urllib.request.urlretrieve(source_url,source)
    license_file=directory/'LICENSE.txt'
    if not license_file.exists():urllib.request.urlretrieve('https://raw.githubusercontent.com/techwinder/flow5/v7.57/LICENSE',license_file)
    record=dict(version='7.57',binary_url=URL,archive_sha256=SHA256,executable_sha256=EXE_SHA256,
                source_url=source_url,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),license='GPL-3.0-or-later')
    (directory/'installation.json').write_text(json.dumps(record,indent=2))
    print('Verified flow5 7.57 portable installation')


if __name__=='__main__':main()
