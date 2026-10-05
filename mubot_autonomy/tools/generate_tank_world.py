#!/usr/bin/env python3
"""Write a tank world with ArUco markers on the walls (docs/tank_setup.md).

Markers are thin boxes named aruco_<id>. TODO: attach the marker PNG textures
(generated with cv2.aruco.generateImageMarker) as PBR albedo maps.
"""
import argparse
import math


def marker(i, x, y, yaw, size):
    return f'''    <model name="aruco_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose>
      <link name="board"><visual name="v"><geometry><box><size>0.002 {size} {size}</size></box></geometry>
        <material><diffuse>1 1 1 1</diffuse></material></visual></link>
    </model>
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--length', type=float, default=2.0)
    ap.add_argument('--width', type=float, default=1.0)
    ap.add_argument('--spacing', type=float, default=0.2)
    ap.add_argument('--size', type=float, default=0.04)
    ap.add_argument('-o', '--out', default='markers.sdf.part')
    a = ap.parse_args()
    out, i = [], 0
    L, W = a.length / 2 - 0.005, a.width / 2 - 0.005
    n_long, n_short = int(a.length / a.spacing), int(a.width / a.spacing)
    for k in range(1, n_long):
        x = -a.length / 2 + k * a.spacing
        out.append(marker(i, x, W, -math.pi / 2, a.size)); i += 1
        out.append(marker(i, x, -W, math.pi / 2, a.size)); i += 1
    for k in range(1, n_short):
        y = -a.width / 2 + k * a.spacing
        out.append(marker(i, L, y, math.pi, a.size)); i += 1
        out.append(marker(i, -L, y, 0.0, a.size)); i += 1
    with open(a.out, 'w') as f:
        f.write(''.join(out))
    print(f'wrote {i} markers to {a.out}; paste inside <world> of mubot_tank.sdf')


if __name__ == '__main__':
    main()
