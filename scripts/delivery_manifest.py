"""Create a compact integrity/execution record for the delivered local prototype."""

from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import xml.etree.ElementTree as ET
import nbformat

root = Path(__file__).resolve().parents[1]
paths = [root / 'README.md', root / 'pyproject.toml', root / 'requirements-lock.txt']
for folder in ['src', 'tests', 'scripts', 'examples', 'docs', 'notebooks']:
    paths.extend(p for p in (root / folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
files = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}
notebooks = []
for p in sorted((root / 'notebooks').glob('*.ipynb')):
    n = nbformat.read(p, as_version=4)
    code = [c for c in n.cells if c.cell_type == 'code']
    errors = [o for c in code for o in c.get('outputs', []) if o.output_type == 'error']
    notebooks.append(
        {
            'file': str(p.relative_to(root)),
            'all_cells_executed': all(c.execution_count is not None for c in code),
            'error_outputs': len(errors),
        }
    )
test = ET.parse(root / 'outputs/validation/tests.xml').getroot().find('testsuite')
report = {
    'created_utc': datetime.now(timezone.utc).isoformat(),
    'project_version': '0.1.0',
    'full_plan_completion': False,
    'tests': test.attrib,
    'notebooks': notebooks,
    'full_cycle_mesh_converged': False,
    'actual_aircraft_validated': False,
    'raw_aero_database': 'outputs/aero_database.npz',
    'files_sha256': files,
    'remaining': [
        'conservative variable-length cable energy boundary',
        'full-cycle mesh/contact convergence',
        'actual team CAD/properties/test validation',
    ],
}
(root / 'outputs/delivery_manifest.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf8')
print(json.dumps({'tests': test.attrib, 'notebooks': notebooks, 'files': len(files)}, indent=2))
