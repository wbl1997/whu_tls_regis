import os

# found 1105/data6/back, and 1105_aft/data4/back but not complete
liodir = f'H:\\jhuai\\results\\front_back_snapshots'
backlio = []
for root, dirs, files in os.walk(liodir):
    for file in files:
        if file.endswith('scan_states.txt') and root.endswith('back'):
            backlio.append(os.path.join(root, file))

for i, f in enumerate(backlio):
    with open(f, 'r') as file:
        lines = []
        for line in file:
            if line.strip() == '' or line.startswith('#'):
                continue
            lines.append(line)
        
        t0 = float(lines[0].split()[0])
        t1 = float(lines[-1].split()[0])
        print(f'{i}: {f} {t0:.2f} {t1:.2f} {t1-t0:.2f}')