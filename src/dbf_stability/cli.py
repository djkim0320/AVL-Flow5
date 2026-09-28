import argparse
import json
from pathlib import Path
import numpy as np
from . import load_case, AeroDatabase, build_aero_database, solve_trim, analyze_stability, simulate, resume_simulation
from .plots import export_plots


def main():
    parser = argparse.ArgumentParser(description='DBF coupled sensor research prototype')
    parser.add_argument('command', choices=['build-aero', 'trim', 'stability', 'simulate', 'resume'])
    parser.add_argument('--case', default='examples/reference.yaml')
    parser.add_argument('--aero', default='outputs/aero_database.npz')
    parser.add_argument('--output', default='outputs/run')
    parser.add_argument('--phase', default='mission', choices=['mission', 'deployed', 'stowed', 'aircraft_only'])
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--checkpoint')
    args = parser.parse_args()
    c = load_case(args.case)
    if args.command == 'build-aero':
        db = build_aero_database(c, args.aero, args.workers)
        print(db.path)
        return
    db = AeroDatabase(args.aero)
    if args.command == 'resume':
        if not args.checkpoint or args.duration is None:
            parser.error('resume requires --checkpoint and --duration')
        result = resume_simulation(c, db, args.checkpoint, duration=args.duration, output=args.output)
        export_plots(result, args.output)
        print(json.dumps(result.summary, ensure_ascii=False, indent=2))
        return
    if args.command == 'trim':
        r = solve_trim(c, db, mode='stowed' if args.phase == 'mission' else args.phase)
        print(json.dumps({k: v for k, v in r.items() if k != 'state'}, indent=2))
        return
    if args.command == 'stability':
        r = analyze_stability(c, db)
        out = Path(args.output)
        out.mkdir(parents=True, exist_ok=True)
        np.save(out / 'state_matrix.npy', r['matrix'])
        r['modes'].to_csv(out / 'modes.csv', index=False)
        print(r['modes'].head(12).to_string(index=False))
        return
    result = simulate(c, db, phase=args.phase, duration=args.duration, output=args.output)
    export_plots(result, args.output)
    print(json.dumps(result.summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
