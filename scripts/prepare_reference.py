"""Create a metrically scaled official AVL example; preserve original attribution."""
from pathlib import Path
import re
import shutil

root=Path(__file__).resolve().parents[1]
source=root/'vendor/avl/AVL3.52rel09032025/runs'
lines=(source/'vanilla.avl').read_text().splitlines()
out=[];scale=.17;pending=None
for i,line in enumerate(lines):
    stripped=line.strip()
    if i==0:
        out.append('MIT Plane Vanilla scaled 0.17 - research reference');continue
    if stripped.startswith('#Sref'):
        pending='refs';out.append(line);continue
    if stripped.startswith('#Xref'):
        pending='xyz';out.append(line);continue
    if stripped.startswith('TRANSLATE'):
        pending='xyz';out.append(line);continue
    if stripped=='SECTION':
        pending='section';out.append(line);continue
    if pending and stripped and not stripped.startswith('#'):
        nums=[float(x) for x in stripped.split()]
        if pending=='refs':
            nums=[nums[0]*scale**2,nums[1]*scale,nums[2]*scale]
        elif pending=='xyz':
            nums=[x*scale for x in nums]
        else:
            nums=[x*scale if j<4 else x for j,x in enumerate(nums)]
        line=' '.join(f'{x:.8g}' for x in nums);pending=None
    out.append(line)
(root/'examples/aircraft.avl').write_text('\n'.join(out)+'\n',encoding='ascii')
shutil.copy2(source/'sd7037.dat',root/'examples/sd7037.dat')
print('Created scaled MIT geometry, not team aircraft')
from dbf_stability import load_case
from dbf_stability.avl import write_mass_file
case=load_case(root/'examples/reference.yaml')
write_mass_file(case,root/'examples/aircraft.mass')
write_mass_file(case,root/'examples/stowed.mass',stowed=True)
