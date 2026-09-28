"""Real-process timeout diagnostics; no fabricated aerodynamic outputs."""

import json
import subprocess
from pathlib import Path
import pytest
from dbf_stability.avl import run_avl


def test_invalid_avl_timeout_is_rejected_before_launch(tmp_path):
    with pytest.raises(ValueError, match='timeout_s'):
        run_avl('missing.exe', 'missing.avl', tmp_path, timeout_s=0.0)


@pytest.mark.avl
def test_real_avl_timeout_retains_input_and_diagnostics(cfg, tmp_path):
    root = Path(cfg['_root'])
    out = tmp_path / 'timed_out'
    with pytest.raises(subprocess.TimeoutExpired):
        run_avl(root / cfg['aero']['executable'], root / cfg['aero']['geometry'], out, timeout_s=1e-6)
    assert (out / 'commands.txt').is_file()
    assert (out / 'plane.avl').is_file()
    assert (out / 'stdout.txt').is_file() and (out / 'stderr.txt').is_file()
    assert json.loads((out / 'failure.json').read_text('utf8')) == {'status': 'timeout', 'timeout_s': 1e-6}
