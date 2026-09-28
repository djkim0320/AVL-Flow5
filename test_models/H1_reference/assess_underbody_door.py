"""Static STEP feasibility study; does not alter a case or mission result."""
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import numpy as np
import audit_door_swing as audit
from dbf_stability import load_case
from dbf_stability.door import door_box
from geometry_paths import cad_directory


def initialize(case):
    import cadquery as cq
    c = load_case(case)
    audit.init_worker(c['bay']['door_hinge_offset_m'], case)
    # Opening and closing are assessed with the sensor fully stowed.
    for path in cad_directory(c).glob('sensor*.step'):
        if path.stem != 'sensor_tow_point':
            audit.FIXED[path.stem] = cq.importers.importStep(str(path)).val()


def inspect(angle):
    row = audit.inspect(angle)
    if angle == 270:
        plate = audit.DOOR.rotate(tuple(audit.HINGE), tuple(audit.HINGE+[0, 1, 0]), 170)
        bb = plate.BoundingBox()
        distances = {}
        for name, shape in audit.FIXED.items():
            ob = shape.BoundingBox()
            if any(getattr(bb, a+'max') < getattr(ob, a+'min')-20 or
                   getattr(bb, a+'min') > getattr(ob, a+'max')+20 for a in 'xyz'):
                continue
            try:
                distances[name] = float(plate.distance(shape))
            except (ValueError, RuntimeError) as exc:
                row['errors'].append({'part': name, 'distance_error': str(exc)})
        row['nearby_part_distances_mm'] = distances
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--case', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--workers', type=int, default=2)
    p.add_argument('--step', type=int, default=5)
    p.add_argument('--render-only', action='store_true')
    args = p.parse_args()
    c = load_case(args.case)
    if args.render_only:
        result = json.loads((args.output/'audit.json').read_text(encoding='utf8'))
        assert result['source_case_sha256'] == hashlib.sha256(args.case.read_bytes()).hexdigest()
        assert result['cad_sha256'] == {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in cad_directory(c).glob('*.step')}
        rows = result['rows']
    else:
        args.output.mkdir(parents=True, exist_ok=False)
        if args.step <= 0 or 270 % args.step: raise ValueError('Step must divide 270 degrees')
        angles = list(range(0, 271, args.step))
        with ProcessPoolExecutor(max_workers=args.workers, initializer=initialize,
                                 initargs=(str(args.case.resolve()),)) as pool:
            rows = list(pool.map(inspect, angles))
        result = {
        'scope': 'Door leaf vs every fixed STEP part and stowed sensor at sampled angles. Exported hinge hardware included if present. No continuous sweep, actuator or flight analysis.',
        'continuous_sweep_proof': False,
        'source_case': str(args.case.resolve()),
        'source_case_sha256': hashlib.sha256(args.case.read_bytes()).hexdigest(),
        'cad_sha256': {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in cad_directory(c).glob('*.step')},
        'door_hinge_offset_m': c['bay']['door_hinge_offset_m'],
        'door_closed_offset_m': c['bay']['door_closed_offset_m'],
        'angle_step_deg': args.step, 'samples': len(rows),
        'intersecting_samples': sum(bool(r['hits']) for r in rows),
        'query_error_samples': sum(bool(r['errors']) for r in rows), 'rows': rows,
        }
        (args.output/'audit.json').write_text(json.dumps(result, indent=2), encoding='utf8')
    # Plane through the actual STL airframe, drawn at identical scale for both doors.
    from dbf_stability.collision import read_binary_stl
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font_manager.fontManager.addfont('C:/Windows/Fonts/malgun.ttf')
    plt.rcParams['font.family'] = 'Malgun Gothic'
    plt.rcParams['axes.unicode_minus'] = False
    fig, ax = plt.subplots(figsize=(10, 4.8))
    directory = cad_directory(c).parent/'meshes'
    for name, color in [('fuselage_H1_approx_with_rear_cutout', '#83919b'),
                        ('guide_floor', '#566570'), ('guide_ceiling', '#566570'),
                        ('rear_exit_frame', '#566570'), ('sensor_body', '#d7a338')]:
        if not (directory/(name+'_FRD_m.stl')).exists():continue
        vertices, faces = read_binary_stl(directory/(name+'_FRD_m.stl'))
        for triangle in vertices[faces]:
            intersections = []
            for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
                if abs(a[1]) < 1e-10:
                    intersections.append(a)
                if a[1]*b[1] < 0:
                    intersections.append(a-a[1]/(b[1]-a[1])*(b-a))
            if len(intersections) >= 2:
                line = np.unique(np.round(intersections, 10), axis=0)
                if len(line) >= 2:
                    ax.plot(line[:, 0]*1000, -line[:, 2]*1000, color=color, linewidth=1.4)
    centers = {}
    for angle, color in [(140, '#bb632b'), (270, '#006c9b')]:
        center, axes, half = door_box(c['bay'], angle)
        local = np.array([[x, 0, z] for x, z in [(-half[0], -half[2]),
                         (half[0], -half[2]), (half[0], half[2]),
                         (-half[0], half[2]), (-half[0], -half[2])]])
        polygon = center+local@axes.T
        ax.fill(polygon[:, 0]*1000, -polygon[:, 2]*1000, color=color, alpha=.8,
                label=f'{angle}° 문 위치')
        centers[str(angle)] = center.tolist()
    hinge = audit.door_hinge(c['bay'])*1000
    ax.scatter(hinge[0], -hinge[2], color='#202830', s=35, zorder=5, label='힌지축')
    ax.set_xlim(hinge[0]-95, hinge[0]+165)
    ax.set_ylim(-hinge[2]-85, -hinge[2]+105)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('기체 x [mm]   ← 후방 · 전방 →')
    ax.set_ylabel('높이 -z [mm]')
    ax.set_title('문을 동체 아래로 접는 안 · 현재 CAD의 중앙 단면')
    label = '270°: 동체 아래 접힘\n검사한 각도에서 겹침 없음' if not result['intersecting_samples'] and not result['query_error_samples'] else '270°: 현재 형상에 간섭 있음\n상세 각도는 검사 기록 참조'
    ax.annotate(label,
                xy=(hinge[0]+18, -hinge[2]), xytext=(hinge[0]+70, -hinge[2]-48),
                arrowprops=dict(arrowstyle='->', color='#006c9b'), color='#006c9b', fontsize=10)
    ax.grid(alpha=.2)
    ax.legend(loc='upper left', fontsize=9)
    fig.text(.5, .012, '실제 형상 비교 · 각도별 CAD 검사 · 구동기 응답과 비행 성능을 보여주는 그림은 아님', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .035, 1, 1))
    fig.savefig(args.output/'comparison.png', dpi=170)
    plt.close(fig)
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'cad_sha256')}, indent=2))
    print(json.dumps({'intersecting_angles_deg': [r['angle_deg'] for r in rows if r['hits']],
                      'folded': rows[-1], 'door_centers_frd_m': centers}, indent=2))


if __name__ == '__main__':
    main()
