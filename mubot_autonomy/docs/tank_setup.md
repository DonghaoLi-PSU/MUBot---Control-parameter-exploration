# Tank setup (simulation and real)

- ArUco dictionary 4×4_50, markers about 20 cm apart on the walls, so at least
  one is usually in view.
- At 320 × 240, a 4 cm marker is readable to about 0.8 m; at 160 × 120 the
  range halves.
- Put markers on fixed obstacles too; they become landmarks.
- Texture walls and obstacles so camera looming works between markers.
- Target: red sphere, 5 cm diameter (matches `target_diameter` in
  `perception.yaml`).
- `tools/generate_tank_world.py` writes a world with this layout.
