"""Isolated solver copy: leave ongoing mesh-reference runs unchanged."""
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
variant=HERE/'solver_variants/analytic_stop'
sys.path.insert(0,str(variant))
import dbf_stability
assert Path(dbf_stability.__file__).resolve().is_relative_to(variant.resolve())
from run_capture_stop import main

if __name__=='__main__':main()
