"""Fingerprint the actual geometry and its referenced aerodynamic input files."""
from pathlib import Path
import hashlib


def input_fingerprint(geometry):
    geometry=Path(geometry).resolve()
    records={'geometry':hashlib.sha256(geometry.read_bytes()).hexdigest()}
    lines=geometry.read_text(encoding='ascii').splitlines()
    for i,line in enumerate(lines):
        token=line.strip().split()[:1]
        if token not in (['AFILE'],['BFILE']):continue
        if i+1>=len(lines):raise ValueError('Missing aerodynamic input reference')
        name=lines[i+1].strip()
        path=(geometry.parent/name).resolve()
        if not name or not path.is_relative_to(geometry.parent):
            raise ValueError('Geometry reference must remain in its input directory')
        records[token[0]+':'+path.relative_to(geometry.parent).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    return records
