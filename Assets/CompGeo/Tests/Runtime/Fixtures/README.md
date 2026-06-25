# Geodesic regression fixtures (local-only)

`Man0GroundTruthTests` validates the Burst Dijkstra against the instructor-provided
**all-pairs geodesic distance matrix** for `man0.off` — the CENG789 ground truth (per
`docs/MIGRATION.md §1`, the `meshes1/fprint matrix/` "M for man0" = all-pairs distances of
the 502-vertex `man0` mesh).

That data is **all-rights-reserved course material and is not committed** (see `.gitignore`).
To enable the regression test, drop the two files here locally:

| File | What it is | Where from |
|---|---|---|
| `man0.off` | the 502-vertex mesh | old repo `…/StreamingAssets/meshes1/faces/` (or `timing/`) |
| `M_for_man0.txt` | 502×502 whitespace-separated distance matrix (row-major) | old repo `…/StreamingAssets/meshes1/fprint matrix/` |

Source repo (read on demand, do not clone — ~311 MB of build artifacts):
`https://github.com/mehmetsrl/ComputationalGeometry` → `CENG789_GeometryProcessing`.

When both files are present the test runs and compares each Dijkstra row against the matrix
within tolerance; when absent it is skipped (`Assert.Ignore`), so CI without the data stays green.
