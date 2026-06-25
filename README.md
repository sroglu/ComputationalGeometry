# ComputationalGeometry

Computational-geometry toolkit + experimental workspace (Unity). Project name `ComputationalGeometry`; code namespace abbreviated `CompGeo`.

A personal collection of computational-geometry algorithms, data structures, and
explorations — kept as reusable, performant Unity code and grown over time. The scope is
broad and ongoing: independent/personal studies and new CG topics are added as they come.
One early seed is the mesh-processing work migrated from an earlier METU CENG789 project
(`github.com/mehmetsrl/ComputationalGeometry`) — it's a starting point, not the boundary.
Reusable core is render-independent (pure C# + Burst); visualization and the `Sandbox/`
playground sit on top.

**Migration plan / technical evaluation:** see `docs/MIGRATION.md`.

## Planned structure (capability-based)

```
Runtime/
  Core/             # primitives, geometric predicates, mesh (HalfEdge/SoA/CSR), I/O
  Collections/      # indexed heap, KD-tree, BVH, spatial hash
  Numerics/         # sparse matrix (CSR), sparse SPD / CG solvers
  MeshProcessing/
    Geodesics/      # Dijkstra, FastMarching, HeatMethod
    Parameterization/ # Tutte, LSCM, ARAP
  <future>/         # ConvexHull / Delaunay / Voronoi / Intersection / RangeSearch ...
Visualization/      # GPU instancing, line-topology meshes, heatmap shaders
Sandbox/            # experimental prototypes (not packaged)
Samples~/           # polished demos
Tests/              # incl. geodesic ground-truth regression
```

## Project setup

Unity project — open the repo root in **Unity 6000.3.12f1** (matching the sibling repos
ProjectFoundation / Playnest / Infrastructural). Code lives under `Assets/CompGeo/` (namespace
`CompGeo`), organized by capability. Key package deps: `com.unity.mathematics` 1.3.3,
`com.unity.collections` 2.6.6 (pulls in Burst), `com.unity.test-framework` 1.6.0.

Coding conventions: see `Documentation/CODING-STYLE.md` (adopted from ProjectFoundation).

## Status

Migration progresses in the order set out in `docs/MIGRATION.md §6`.

| Step | Scope | State |
|---|---|---|
| 1 | **Core** — SoA/CSR mesh (`MeshData`), OFF I/O (`OffReader`), adjacency builder, geometric predicates & primitives | ✅ implemented (`Assets/CompGeo/Runtime/Core/`) |
| 2 | **HW1 Burst geodesics** — decrease-key `NativeIndexedMinHeap` (`CompGeo.Collections`); shared `GeodesicSearch` kernel exposed as `DijkstraGeodesics` (one-to-all) and `AStarGeodesics` (point-to-point, admissible Euclidean heuristic); `[BurstCompile]` jobs; `M for man0` regression scaffold | ✅ implemented (`Runtime/Collections/`, `Runtime/MeshProcessing/Geodesics/`) |
| 3 | **GPU rendering layer (URP)** — `KdTree3` (`CompGeo.Collections`, point + ray nearest); `MeshGpuView` (single Points mesh + single Lines edge mesh + path, per-vertex heatmap, one draw/layer); `MeshPicker` (KD-tree ray/point pick); `Heatmap` ramp; URP vertex-colour shader; `GeodesicDemo` driver (click-pick → Dijkstra heatmap + A* path) | ✅ verified in-editor (URP asset `Assets/Settings/CompGeo_URP.asset` assigned; `Samples/GeodesicDemo.unity` renders the Dijkstra heatmap) |
| 4 | HW2 sparse Laplacian solve (CSR assembly + sparse SPD solver) | ⬜ planned |
| 5 | Polish — heatmap viz, unfolded-result RT preview, samples | ⬜ planned |

The Dijkstra/A* path-finding is a **clean-room reimplementation** from the textbook spec on a
data-oriented structure (CSR + `dist`/`pred` + indexed heap); it shares no code lineage with the
original course project's unlicensed-gist `PriorityQueue<double,Path>`/`HashSet` version. Same
kernel is intended for ProjectFoundation's `DataStructures` (see its FOUNDATION-MIGRATION-REPORT).

The remaining `Runtime/Numerics`, `MeshProcessing/Parameterization` and `Sandbox/`
are scaffolded placeholders for the steps above. The URP layer is wired up: `Assets/Settings/CompGeo_URP.asset`
is assigned in Project Settings → Graphics so the `CompGeo/VertexColorUnlit` shader renders; open
`Assets/CompGeo/Samples/GeodesicDemo.unity` and press Play to see the live Dijkstra heatmap / A* path demo.
