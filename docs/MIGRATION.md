# CENG789 Geometry Processing — Technical Evaluation & Performance Migration Plan (LOCAL — do not push)

> **Source repo (read on demand from GitHub — NOT cloned locally):**
> `https://github.com/mehmetsrl/ComputationalGeometry` → subfolder `CENG789_GeometryProcessing`.
> When code detail is needed, read it from GitHub (raw URLs / `gh`); don't keep a local clone — the repo
> carries ~311 MB of committed build artifacts (`Build/`, `obj/`, `Logs/`).
> User's own METU CENG789 (Digital Geometry Processing) course project, written before learning Unity.
> Goal: keep the (solid) algorithms, re-implement performantly, cite the proper papers, publish under the
> sroglu GitHub. This doc is a standalone LOCAL plan (not committed/pushed).

---

## 1. What the project actually does (the work underneath — it's solid)

| Part | Algorithm | Files |
|---|---|---|
| **Mesh data structure** | Vertex/Edge/Triangle incidence-adjacency (per-vertex `VertexList`/`TriangleList`/`EdgeList`, edge endpoints, triangle vertices + normal). Textbook adjacency mesh. | `Model.cs` |
| **Mesh I/O** | OFF/mesh reader → builds the adjacency structure | `ModelReader.cs`, `ModelManager.cs` |
| **HW1 — discrete geodesics** | Shortest path on the vertex/edge graph via **Dijkstra** + a generic **A\*** (with admissible Euclidean estimate), over an `IHasNeighbours<T>` graph abstraction, custom binary-heap `PriorityQueue<TKey,TValue>`, immutable `Path<TNode>`. | `HW1/ShotestPathSearch.cs`, `PriorityQueue.cs` |
| **HW2 — mesh unfolding / parameterization** | **Tutte/Floater barycentric embedding** with the **uniform graph Laplacian**: boundary vertices pinned to a convex loop, interior solved by `L·X=bX`, `L·Y=bY`. | `Model.cs` (`UnfoldModel`/`UniformUnfold`) |
| Support libs (vendored) | KD-tree NN (`KdTreeLib`), ear-clipping triangulation (`Triangulator`), dense linear algebra (`Accord.Math`/`Accord.Statistics`) | — |

**This is real, correct geometry processing.** The algorithms are textbook-faithful; the adjacency model and the Dijkstra/A* + Laplacian-unfold pipeline are sound. The weakness is **100% in the Unity-side data layout, linear algebra choices, and rendering** — not in the math.

> Side note: this repo is the **origin of the framework `DataStructures` A\*/`Path`/`IHasNeighbours`/`PriorityQueue` pattern** (the "THeK3nger gist" lineage flagged in the foundation audit). Same code family.

### Validation data (confirmed 2026-06-25)
The `StreamingAssets/meshes1/fprint matrix/` folder is NOT an implemented feature (no `fprint`/`fingerprint` code anywhere — only HW1 and HW2 exist). **"fprint" = "footprint"**: `M for man0.off` is the **all-pairs geodesic distance matrix** for `man0.off` — a 502×502 symmetric, zero-diagonal matrix (man0 has 502 vertices). Per `meshes1/readme.txt` (by "ysf" = Yusuf Sahillioğlu, the course instructor) it is the **instructor-provided ground-truth answer** for validating the HW1 Dijkstra geodesics output. `timing/` (man, centaur, dragon) are benchmark meshes for Dijkstra runtime; `faces/` are meshes for the A* + mesh-generation parts.
**Useful for the rewrite:** keep `M for man0.off` as a **regression-test fixture** — validate the new Burst Dijkstra (and any Heat Method / FMM upgrade) against this ground-truth distance matrix.

---

## 2. Performance & architecture problems

### A. Rendering — GameObject-per-element (the headline)
- **Every vertex** = `GameObject.Instantiate(prefab)` with a `MeshRenderer` + **its own material instance** (`rend.materials[0]`). N vertices → N GameObjects + N renderers + N materials. A 10k-vertex mesh = 10k GameObjects. (`Model.cs` `Vertex` ctor)
- **Every edge** and **every triangle normal** = a `LineRenderer` GameObject (`Model.cs` `Edge`/`Triangle`). LineRenderers do not batch; this is extremely heavy.
- **Per-vertex material instantiation** breaks batching and explodes memory; selection sets `materials[0].color` → another material instance.
- **Path/highlight** rebuilt as new `GameObject`s + LineRenderers each query, destroyed/recreated.

### B. Data layout — managed AoS, GC-heavy, cache-hostile
- `Vertex`/`Edge`/`Triangle` are **classes** (reference types) with `List<int>` adjacency → pointer-chasing, GC pressure, no SoA, no `NativeArray`.
- `Path<TNode>` is an **immutable linked list allocating a node per step**, and `PriorityQueue<double,…>` **boxes** the double keys → heavy per-iteration GC inside the Dijkstra/A* inner loop.
- Statics for shared state (`Vertex.Center`, `count`) → not multi-mesh / re-entrancy safe.

### C. HW2 linear algebra — dense and O(N³) (algorithmic, not just rendering)
- `laplacianMatrix = new float[N, N]` — **dense** N×N for a matrix that is **sparse** (≈ avg-valence non-zeros per row).
- Solved by **`Accord.Math.Matrix.Inverse(L)`** — explicit dense inverse, O(N³) time + O(N²) memory. Should never invert; solve the sparse SPD system.
- Matrix assembly uses `Vertexs.Find(x => x.idx == i)` **inside nested loops** → O(N²) linear scans just to build the matrix (and the vertices are already index-addressable).

### D. Selection / picking
- `Physics.Raycast` against a **per-vertex collider**, then **`Int32.TryParse(hit.transform.name)`** to recover the vertex index. Name-as-identity + physics colliders per vertex — fragile and heavy. A KD-tree (already present!) or GPU pick is the right tool.

### E. No Burst / Jobs / GPU
- Everything is managed, main-thread. No `Unity.Mathematics`, no Jobs, no Burst, no compute/instancing.

---

## 3. Migration plan — re-implement performantly

### Principle: split **Algorithm** (pure C#/Burst) from **Presentation** (GPU)
Today they're fused — a `Vertex` holds both graph adjacency *and* a `GameObject`. Separate them:
- **Core (Burst-friendly POCO/struct):** SoA `NativeArray<float3> positions`, `NativeArray<int3> triangles`, **CSR adjacency** (`NativeArray<int> adjOffsets` + `NativeArray<int> adjNeighbours`) instead of `List<int>` per vertex. No GameObjects, no Unity refs.
- **Presentation:** draws the core arrays on the GPU; never one GameObject per element.

### HW1 — geodesics
- Burst-compiled **Dijkstra over CSR** with an **index-based native binary heap**; output `NativeArray<float> dist` + `NativeArray<int> pred` (reconstruct path from `pred`) — zero `Path` allocation, no boxing.
- Optional upgrade to **true geodesics** (the graph distance is only an approximation): **Fast Marching** (Kimmel–Sethian) or the **Heat Method** (Crane et al. — two sparse solves, GPU-friendly, the modern performant choice), or exact **MMP** (Surazhsky et al.).

### HW2 — unfolding / parameterization
- Assemble the Laplacian as a **sparse** matrix in **O(E)** via CSR (drop the `Vertexs.Find` O(N²) scans — index directly).
- Replace `Matrix.Inverse` with a **sparse SPD solve** (sparse Cholesky, or CG/BiCG) — Math.NET Numerics, or a Burst CG over CSR. Orders of magnitude faster + far less memory.
- Optional quality upgrade: **cotangent weights** (Pinkall–Polthier) instead of uniform, or **LSCM/ARAP** for low-distortion conformal maps.

### Rendering — GPU instead of GameObjects
- **Vertices** → one draw via `Graphics.RenderMeshInstanced` / `RenderPrimitives` (or a procedural point mesh + shader). **Per-vertex color** (geodesic heatmap, selection) via a `StructuredBuffer`/instance color buffer — not a material per vertex.
- **Edges** → a single mesh with `MeshTopology.Lines` (one mesh, one draw) instead of N LineRenderers.
- **Path/highlight** → one dynamic line mesh updated in place.
- **Unfolded 2D result** (`UnfoldingResult` image) → render-to-texture preview.

### Picking
- Ray vs positions using the **existing KD-tree** (or a BVH), or GPU ID-buffer picking. Drop per-vertex physics colliders and name parsing.

### Hygiene
- Remove statics (`Vertex.Center`/`count`) → instance state, multi-mesh safe. `Unity.Mathematics` types throughout.

---

## 4. Reuse / inspiration from the existing Render module

(The framework `Render` module — documented in `ProjectFoundation/docs/FOUNDATION-MIGRATION-REPORT.md` Part 2 — already solves most of the GPU side.)

| Need here | Render-module piece to reuse/adapt |
|---|---|
| Draw N vertices as instanced point cloud + Burst culling | **`BatchRendering`** (`IBatchRenderingService.RegisterBatch`, Classic/Indirect/Procedural backends, Burst frustum/distance culling, pure C#) |
| Per-vertex compute (color by geodesic dist, animated viz) | **`Particles.Image`** Burst sim patterns (`FunctionPointer<ApplyDelegate>` dispatch) |
| Custom edge/line pass, GPU picking pass | **`Render.Core`** `RenderFeatureBase` / `RenderPassBase<TPassData>` (URP RenderGraph scaffold) |
| Push heatmap ramp / global params to shaders | **`GlobalShaderParameterManager`** (`Register(provider, priority)`) |
| 2D unfolded-result preview surface | **`RenderContext`** (pooled RT + Camera + content root, RawImage anchor) |
| Pooled scratch render targets | **`RenderTexturePool`** |

Stay open-minded: these are starting points, not mandates — if a plain `Graphics.RenderMeshInstanced` + a CG solver is simpler for a standalone research repo, prefer simplicity.

---

## 5. Papers to cite (verify final list against what each HW implements)

**Geodesics (HW1):**
- E. W. Dijkstra, *A note on two problems in connexion with graphs*, 1959 — the graph shortest path actually used.
- Hart, Nilsson, Raphael, *A formal basis for the heuristic determination of minimum cost paths*, 1968 — A*.
- Mitchell, Mount, Papadimitriou, *The discrete geodesic problem*, 1987 — exact polyhedral geodesics (MMP).
- Surazhsky, Surazhsky, Kirsanov, Gortler, Hoppe, *Fast exact and approximate geodesics on meshes*, SIGGRAPH 2005.
- Kimmel, Sethian, *Computing geodesic paths on manifolds*, PNAS 1998 — Fast Marching.
- Crane, Weischedel, Wardetzky, *Geodesics in Heat*, ACM TOG 2013 — the Heat Method.

**Parameterization / unfolding (HW2):**
- W. T. Tutte, *How to draw a graph*, 1963 — barycentric embedding (the uniform-Laplacian unfold used).
- M. Floater, *Parametrization and smooth approximation of surface triangulations*, CAGD 1997; *Mean value coordinates*, 2003.
- Floater & Hormann, *Surface parameterization: a tutorial and survey*, 2005.
- Pinkall & Polthier, *Computing discrete minimal surfaces and their conjugates*, 1993 — cotangent Laplacian (quality upgrade).
- Lévy, Petitjean, Ray, Maillot, *Least Squares Conformal Maps*, SIGGRAPH 2002 (conformal alternative).
- Sorkine & Alexa, *As-Rigid-As-Possible surface modeling*, 2007 (ARAP alternative).

**Support:**
- J. L. Bentley, *Multidimensional binary search trees (k-d trees)*, 1975 — the KD-tree NN.

---

## 5b. Compute shaders — where they fit (and where they don't)

This is a desktop/research tool (not the mobile Playnest), so compute shaders are fully available — but use them selectively, by data-parallelism, not by default.

**Good fit (data-parallel over vertices/faces/edges):**
- Per-vertex/face fields: normals, curvature, geodesic-field gradient/divergence, heatmap color mapping — embarrassingly parallel.
- **Heat Method** geodesics (Crane 2013): the diffusion / gradient-normalize / divergence steps; iterative (Jacobi/CG) solves.
- **HW2 unfolding via an iterative solver**: replace the dense inverse with **Jacobi/CG**, where each iteration is a CSR sparse matrix-vector product parallel over rows — a real GPU win on large meshes.
- All-pairs / many-source geodesic matrix (the "fprint matrix"): parallel over source vertices.
- GPU picking (ID buffer).

**Wrong tool (sequential / irregular):**
- Single-source **Dijkstra / A\*** — inherently sequential frontier; Burst CPU is better for interactive single queries. (Parallel only via wasteful Bellman-Ford-style relaxation.)
- **Sparse direct solve (Cholesky)** — irregular/sequential factorization; prefer CPU sparse Cholesky (Math.NET) or a GPU *iterative* solver.
- Mesh I/O and topology construction — sequential, CPU.

**Pragmatic order:** do everything in **Burst/Jobs (CPU) first** — simpler, debuggable, fast enough for most meshes (the Render module already handles the *drawing* via GPU instancing). Add compute shaders **only where profiling shows a real win and the data already lives on the GPU** (heat-method fields, large-mesh iterative Laplacian solve, per-vertex heatmaps). Note: the documented Render module pieces don't use compute (BatchRendering = instancing, Blur = fragment, ColorGrading = LUT), so compute is new ground here — no ready-made reuse.

## 5c. Project identity, structure & sandbox (decided 2026-06-25)

This is NOT a mesh-geometry-processing-only project — it's a **computational-geometry umbrella** that will grow (new CG topics added over time) and double as an experimental workspace.

- **Umbrella repo:** `ComputationalGeometry` (keep as-is).
- **Brand / namespace:** **`CompGeo`** (`CompGeo.*`) — the field-standard abbreviation (SoCG community); short, scope-clear, room to grow.
- **Organize by capability, not by homework.** The old `HW1`/`HW2` split is dropped; algorithms live under what they *do*. Old HW1/HW2 become demo scenes under `Samples~`.
- **Reusable core is render-independent** (pure C# + Burst, its own asmdef → UPM-packageable, reusable in other projects). Visualization depends on core, never the reverse.
- **A dedicated `Sandbox/`** holds experimental/prototype work, separate from the clean core, relaxed quality rules, not packaged. (The existing `Practices/` folder can be formalized into this.)

```
ComputationalGeometry/        (repo; namespace CompGeo)
  Runtime/
    Core/             # primitives: point/segment, geometric predicates (orientation, in-circle),
                      #   mesh (HalfEdge/SoA/CSR), I/O — render-independent, Burst
    Collections/      # reusable DS: indexed heap, KD-tree, BVH, spatial hash
    Numerics/         # sparse matrix (CSR), sparse SPD / CG solvers
    MeshProcessing/   # the CENG789 work, capability-based:
      Geodesics/      #   Dijkstra, FastMarching, HeatMethod   (was HW1)
      Parameterization/ # Tutte, LSCM, ARAP                    (was HW2)
    <future>/         # ConvexHull / Delaunay / Voronoi / Intersection / RangeSearch ...
  Visualization/      # Unity GPU viz (instancing, line mesh, heatmap shader) — depends on core
  Sandbox/            # experimental playground: scratch scenes, prototypes (not packaged)
  Samples~/           # polished demos (old HW1/HW2 as showcases)
  Tests/              # + "M for man0" geodesic ground-truth regression fixture
```

**Note:** `Collections` (heap/KD-tree) overlaps the framework `DataStructures` module — share or keep local, but avoid a duplicate implementation.

## 6. Suggested rewrite order

1. **Core extraction** — SoA mesh (positions/triangles/CSR adjacency) as Burst-friendly structs; mesh I/O into it. No rendering.
2. **HW1 Burst Dijkstra** (dist+pred arrays, native heap) → validate against the old output. Optional: Heat Method.
3. **GPU rendering layer** — instanced points + line-topology edges + per-instance color (reuse `BatchRendering`). Replace GameObject-per-element. KD-tree picking.
4. **HW2 sparse Laplacian solve** (CSR assembly O(E) + sparse SPD solver) replacing the dense inverse. Optional: cotangent weights / LSCM.
5. **Polish** — heatmap visualization of geodesic fields, unfolded-result RT preview, screenshots, README with paper citations → publish under sroglu.
