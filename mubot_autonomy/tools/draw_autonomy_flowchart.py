#!/usr/bin/env python3
"""Draws docs/architecture/figures/autonomy_flowchart.svg (run from that folder).

PNG: open the SVG in a browser and screenshot, or use headless Chromium.
"""
W, H = 1900, 1270
C = dict(bg='#000000', lane='#0c0c0c', edge='#2c2c2c', text='#f2f2f2', muted='#a3a3a3',
         sens='#d9d9d9', perc='#4fd1e8', slam='#b49cff', miss='#ffc857', beh='#5be38a',
         dec='#ff9f5b', act='#ff6b8a')
out = []
def add(s): out.append(s)
def esc(t): return t.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
def text(x, y, t, size=13, color=None, weight='normal', anchor='start'):
    add(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color or C["text"]}" font-weight="{weight}" text-anchor="{anchor}">{esc(t)}</text>')
def lane(x, y, w, h, label, col):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="14" fill="{C["lane"]}" stroke="{C["edge"]}" stroke-width="1.5"/>')
    add(f'<rect x="{x}" y="{y}" width="6" height="{h}" rx="3" fill="{col}"/>')
    text(x + 20, y + 28, label, 15, col, 'bold')
def box(x, y, w, h, title, lines, col, fill=None):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="9" fill="{fill or "#151515"}" stroke="{col}" stroke-width="1.8"/>')
    cx = x + w / 2
    n = len(lines)
    top = y + h / 2 - (n * 17 + 18) / 2 + 14
    text(cx, top, title, 14.5, C['text'], 'bold', 'middle')
    for i, l in enumerate(lines):
        text(cx, top + 20 + i * 17, l, 12.5, C['muted'], anchor='middle')
def pill(cx, y, w, h, t, col):
    add(f'<rect x="{cx - w/2}" y="{y}" width="{w}" height="{h}" rx="{h/2}" fill="#151515" stroke="{col}" stroke-width="1.8"/>')
    text(cx, y + h / 2 + 5, t, 13.5, C['text'], 'bold', 'middle')
def diamond(cx, cy, w, h, lines, col, src=None):
    add(f'<polygon points="{cx},{cy-h/2} {cx+w/2},{cy} {cx},{cy+h/2} {cx-w/2},{cy}" fill="#1a120b" stroke="{col}" stroke-width="2"/>')
    n = len(lines)
    for i, l in enumerate(lines):
        text(cx, cy - (n - 1) * 8 + i * 16 + 5, l, 13, C['text'], 'bold' if i == 0 else 'normal', 'middle')
def arrow(pts, col, label=None, lx=None, ly=None, anchor='middle', dash=False, width=2):
    d = ' '.join(f'{x},{y}' for x, y in pts)
    mk = {C[k]: k for k in ('sens','perc','slam','miss','beh','dec','act','muted')}[col]
    da = ' stroke-dasharray="7 5"' if dash else ''
    add(f'<polyline points="{d}" fill="none" stroke="{col}" stroke-width="{width}"{da} marker-end="url(#m_{mk})"/>')
    if label:
        text(lx, ly, label, 12, col, anchor=anchor)

add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="DejaVu Sans, Liberation Sans, Arial, sans-serif">')
add('<defs>')
for k in ('sens','perc','slam','miss','beh','dec','act','muted'):
    add(f'<marker id="m_{k}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{C[k]}"/></marker>')
add('</defs>')
add(f'<rect width="{W}" height="{H}" fill="{C["bg"]}"/>')
text(30, 46, 'μBot Autonomy Flowchart', 28, C['text'], 'bold')
text(30, 72, 'NoA = 4 · medium stiffness · HM-3 · camera + range fan + IMU · ROS 2 Jazzy + Gazebo Harmonic', 14, C['muted'])
add('<g transform="translate(0,30)">')

# ---- lanes
lane(30, 100, 250, 800, 'SENSORS', C['sens'])
lane(310, 100, 330, 800, 'PERCEPTION', C['perc'])
lane(670, 100, 420, 520, 'LOCALIZATION + SLAM', C['slam'])
lane(670, 640, 420, 260, 'MISSION', C['miss'])
lane(1110, 100, 760, 800, 'BEHAVIOR DECISION', C['beh'])
lane(30, 930, 1840, 180, 'ACTUATION + WORLD', C['act'])

# ---- sensors
box(50, 160, 210, 90, 'Camera', ['15 Hz · 320 × 240', 'camera_info: fₓ, cₓ'], C['sens'])
box(50, 395, 210, 90, 'Range fan', ['50 Hz · 5 beams', '±30°, 2 cm – 1 m'], C['sens'])
box(50, 570, 210, 90, 'IMU', ['200 Hz', 'yaw, yaw rate'], C['sens'])

# ---- perception
box(330, 150, 290, 100, 'Vision channel', ['HSV blob → θ = atan((u − cₓ)/fₓ)', 'd = fₓ·D / w · Kalman (θ, d)', 'looming TTC = w / ẇ'], C['perc'])
box(330, 272, 290, 76, 'ArUco detector', ['marker id, bearing, range', '→ SLAM landmarks'], C['perc'])
box(330, 385, 290, 110, 'Range channel', ['3-sample median per beam', 'Kalman per sector (d, ḋ)', 'TTC = d / (−ḋ)', 'polar histogram for VFH+'], C['perc'])
box(330, 565, 290, 100, 'Heading filter', ['complementary filter + gyro bias', '1/f stroke average', '→ heading ψ, yaw rate r'], C['perc'])
arrow([(260, 195), (327, 195)], C['sens'])
arrow([(260, 225), (295, 225), (295, 310), (327, 310)], C['sens'])
arrow([(260, 440), (327, 440)], C['sens'])
arrow([(260, 615), (327, 615)], C['sens'])

# ---- localization + slam
box(690, 145, 380, 62, 'Gait odometry', ['speed model v = g(k, b, mode) from gait state'], C['slam'])
box(690, 232, 380, 62, 'EKF (robot_localization)', ['IMU ψ, r + gait speed → odom → base_link'], C['slam'])
box(690, 319, 380, 62, 'SLAM front end', ['keyframe every 5 cm or 10° · match by marker id'], C['slam'])
box(690, 406, 380, 76, 'Pose graph · GTSAM iSAM2', ['odometry + bearing-range factors', 're-seen marker closes the loop → map → odom'], C['slam'])
box(690, 507, 380, 62, 'Occupancy mapper', ['range beams at optimized poses → /mubot/map'], C['slam'])
for y0, y1 in ((207, 232), (294, 319), (381, 406), (482, 507)):
    arrow([(880, y0), (880, y1 - 3)], C['slam'])
arrow([(620, 640), (645, 640), (645, 263), (687, 263)], C['perc'])
arrow([(620, 310), (660, 310), (660, 350), (687, 350)], C['perc'])
arrow([(620, 470), (660, 470), (660, 538), (687, 538)], C['perc'])

# ---- mission
box(690, 680, 380, 96, 'Mission manager', ['SeekTarget · Explore (map frontiers)', 'ReturnHome (first SLAM pose)', '→ goal bearing for behavior'], C['miss'])
arrow([(880, 569), (880, 677)], C['slam'], 'pose, map', 888, 620, 'start')
text(880, 812, 'actions: /mubot/seek_target · /explore · /return_home', 12, C['muted'], anchor='middle')

# ---- perception data bus into behavior
arrow([(475, 100), (475, 84), (1780, 84), (1780, 112)], C['perc'], None, dash=True)
text(1130, 78, 'obstacles (d, ḋ, TTC, histogram) · target (θ, d, looming TTC) · ψ, r', 12.5, C['perc'], anchor='middle')

# ---- behavior decision flow
X = 1460
pill(X, 120, 230, 34, 'every 50 ms (20 Hz)', C['beh'])
diamond(X, 220, 250, 76, ['Teleop override?'], C['dec'])
diamond(X, 330, 270, 96, ['Collision risk?', 'TTC < T_crit (range or looming)', 'or front range < d_min'], C['dec'])
diamond(X, 450, 250, 84, ['Blocked toward goal?', 'VFH+ picks a free gap'], C['dec'])
diamond(X, 558, 250, 76, ['Target visible?'], C['dec'])
box(1360, 612, 200, 46, 'ψ_ref = mission goal', ['or hold last heading'], C['beh'])
box(1310, 684, 300, 60, 'Heading cascade', ['P: ψ_ref − ψ → r_ref   ·   PI: r_ref − r → b'], C['beh'])
diamond(X, 810, 230, 64, ['|ψ_ref − ψ| > 5° ?'], C['dec'])
arrow([(X, 154), (X, 179)], C['beh'])
arrow([(X, 258), (X, 279)], C['dec'], 'no', X + 8, 272, 'start')
arrow([(X, 378), (X, 405)], C['dec'], 'no', X + 8, 396, 'start')
arrow([(X, 492), (X, 517)], C['dec'], 'no', X + 8, 509, 'start')
arrow([(X, 596), (X, 609)], C['dec'], 'no', X + 8, 607, 'start')
arrow([(X, 658), (X, 681)], C['beh'])
arrow([(X, 744), (X, 775)], C['beh'])

# left: overrides that bypass the cascade
box(1135, 196, 160, 48, 'Use teleop', ['primitive as given'], C['beh'])
box(1135, 288, 150, 84, 'BACKWARD', ['phases Ψ → −Ψ', 'then ESCAPE TURN', '±b_max to ≥ 60°'], C['beh'])
arrow([(X - 125, 220), (1298, 220)], C['dec'], 'yes', 1316, 212, 'middle')
arrow([(X - 135, 330), (1288, 330)], C['dec'], 'yes', 1306, 322, 'middle')

# right: heading references into the cascade
box(1640, 428, 205, 46, 'ψ_ref = gap heading', ['avoid obstacle'], C['beh'])
box(1640, 535, 205, 46, 'ψ_ref = ψ + θ', ['slow down with k(d)'], C['beh'])
arrow([(X + 125, 450), (1637, 450)], C['dec'], 'yes', 1600, 442, 'middle')
arrow([(X + 125, 558), (1637, 558)], C['dec'], 'yes', 1600, 550, 'middle')
add(f'<polyline points="1845,451 1858,451 1858,714" fill="none" stroke="{C["beh"]}" stroke-width="2"/>')
arrow([(1845, 558), (1858, 558)], C['beh'], width=2)
arrow([(1858, 714), (1613, 714)], C['beh'])

# mission goal into fallback
arrow([(1070, 700), (1098, 700), (1098, 635), (1357, 635)], C['miss'], 'goal bearing', 1250, 627, 'middle')

# outputs to the primitive bus
box(1630, 788, 160, 44, 'TURN', ['bias b'], C['beh'])
arrow([(X + 115, 810), (1627, 810)], C['dec'], 'yes', 1598, 802, 'middle')
add(f'<polyline points="1135,220 1128,220 1128,880 1710,880 1710,832" fill="none" stroke="{C["beh"]}" stroke-width="2"/>')
add(f'<polyline points="1135,330 1128,330" fill="none" stroke="{C["beh"]}" stroke-width="2"/>')
add(f'<polyline points="{X},842 {X},880" fill="none" stroke="{C["beh"]}" stroke-width="2"/>')
text(X + 8, 866, 'no → FORWARD (exit below 2°)', 12, C['dec'])
arrow([(X, 880), (X, 952)], C['beh'], 'primitive (mode, b, k)', X + 10, 918, 'start')

# ---- actuation row (flows right → left, then back up to sensors)
box(1300, 955, 320, 110, 'Gait generator', ['γ* + primitive → GaitCmd (50 Hz)', 'turn: + bias · backward: −Ψ', 'bias ramp · switch at phase zero'], C['act'])
box(820, 955, 320, 110, 'Actuator system', ['E = e·sin(2π(f·t + Ψ)) + b', 'clip ±15 V · motor + spring', 'watchdog 200 ms · e-stop'], C['act'])
box(330, 955, 320, 110, 'μBot in water (Gazebo)', ['6 segments · 4 active + caudal joint', 'hydro model HM-3 (Ca 1, Cp 0.5)', '4 kHz physics'], C['act'])
arrow([(1300, 1010), (1143, 1010)], C['act'], 'GaitCmd', 1221, 1002, 'middle')
arrow([(820, 1010), (653, 1010)], C['act'], 'joint torques', 736, 1002, 'middle')
arrow([(330, 1010), (298, 1010), (298, 915), (155, 915), (155, 903)], C['act'], 'sensing', 290, 960, 'end')

# ---- legend
items = [('Sensors', C['sens']), ('Perception', C['perc']), ('Localization + SLAM', C['slam']), ('Mission', C['miss']),
         ('Behavior', C['beh']), ('Decision', C['dec']), ('Actuation', C['act'])]
x = 30
for name, col in items:
    add(f'<rect x="{x}" y="1150" width="18" height="18" rx="4" fill="{col}"/>')
    text(x + 26, 1164, name, 13.5, C['text'])
    x += 40 + len(name) * 8.3
text(30, 1200, 'Decisions are checked top to bottom in priority order: teleop › collision › avoidance › target › mission goal. Only the heading cascade sets the bias b.', 13, C['muted'])
text(30, 1222, 'Dashed: perception data feeding every decision. Mode switches take effect at the next gait-phase zero crossing.', 13, C['muted'])
add('</g>')
add('</svg>')
open('autonomy_flowchart.svg', 'w').write('\n'.join(out))
print('ok')
