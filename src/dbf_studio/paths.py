"""Repository locations shared by the studio server, worker and registry."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
UI = ROOT / 'ui'
DATA = UI / 'data'
TEMPLATES = UI / 'templates'
