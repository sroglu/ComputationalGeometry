# Sample meshes (CENG789)

Triangle meshes in Object File Format (`.off`), loaded at runtime by `OffReader` and driven by the
`MeshCatalog` / `Workbench` samples.

- `faces/` — `face`, `facem` (and `-low` decimated variants): open, disk-topology face scans. Suitable
  for the **Tutte unfold** (HW2) since they have a single boundary loop.
- `geodesics/` — `man0` (502 verts), `horse0`: closed surfaces used for the **geodesic** demos (HW1
  Dijkstra / A*). They have no boundary, so the unfold mode does not apply to them.

These originate from the METU CENG789 (Digital Geometry Processing) course (instructor
Yusuf Sahillioğlu) and were part of the author's own coursework, carried over here for the samples.
The much larger `man.off` / `centaur.off` timing meshes and the `M for man0` ground-truth distance
matrix are not included to keep the repo small.
