# CompGeo — Coding Style

Conventions for CompGeo code (a render-independent computational-geometry core in pure
C# + Burst, with a thin Unity visualization layer on top).

Adopted directly from the sibling **ProjectFoundation** framework — the rules are generic and
shared across the codebase family. Concrete examples below cite PFound (the shared foundation)
and CompGeo interchangeably; the principle is the same.

Tracked and shipped with the code — unlike `docs/`, which is private planning.

Read §1 before touching any `.cs` file; the rest is reference.

---

## 1. Null discipline — fail fast, never defend

**Nothing in our own code should be null.** Do not anticipate a value being null and
write defensive code around it. If something that shouldn't be null *is* null, let the
`NullReferenceException` throw at the access site — that throw is the bug signal we want,
and we actively support it. Defensive guards hide the real lifecycle/wiring bug; the loud
crash points straight at it.

### Forbidden — defensive handling of our own state

- ❌ `_field ??= new Foo();` — lazy init via null-coalescing. Initialize the field eagerly
  (field initializer or ctor) so it is never null.
- ❌ `_field?.Method();` on our own state — if it can be null mid-call, the lifecycle is wrong.
- ❌ `if (_field != null)` as an "is it initialized?" check — initialize deterministically instead.
- ❌ defensive skips on our own data (`if (x == null) return;`) — let it throw. The data is valid
  by construction; if it isn't, fix the lifecycle, don't paper over it.
- ❌ `null` as a sentinel for a valid state. Use an explicit representation
  (e.g. `MeshData.HasAdjacency` / `NativeArray.IsCreated`, never a null-meaning-empty mesh).

### Allowed — these throw, or read a genuine *external* optional

- ✅ **Actively throwing** on bad input: `ArgumentNullException`/`FormatException` on public-API
  args (e.g. `OffReader` throwing `FormatException` on a malformed header). That *is* "let it
  throw," with a clearer message. Encouraged at the library's public boundary.
- ✅ **Fail-fast access**: an accessor that throws when the thing is absent — no silent default.
- ✅ A genuine optional from an **external contract**: a Unity API that returns null, an
  async/asset load that can fail. That null is expected and external — not our state.
- ✅ **Unity-Object liveness** (visualization layer / samples only): a `UnityEngine.Object` ref
  can become "fake-null" when `Destroy`ed. Prefer coupling lifetimes over per-frame null guards.

### Rule of thumb

"Did I forget to initialize / wire this?" → lifecycle bug, let it throw.
"Did this *external* call find what I asked for?" → boundary check is the right answer.

---

## 2. Prefer plain C# over Unity/MonoBehaviour

It's a Unity project — Unity is there and fine to use. The guideline is simply: **reach for
plain C# first; use Unity types only where you actually need them.** Plain classes/structs are
testable without the editor, cheaper, and predictable, and most logic (geometry, topology,
solvers, scheduling) needs nothing from Unity.

This is structural here, not just a default: the **`CompGeo.Core` assembly is render-independent**
(`Unity.Mathematics` + `Unity.Collections`, no `UnityEngine` GameObject/MonoBehaviour). Algorithms
operate on `MeshData` (SoA + CSR), never on scene objects. `MonoBehaviour`/`UnityEngine` live only
in the **Visualization** layer — and that shell stays thin: forward `Awake`/`Update`/`OnDestroy`
into plain methods that drive the core.

---

## 3. try/catch only at external boundaries

`try/catch` is not control flow. The throwable thing must be **external** to our process —
file/IO, an OS call, async cancellation, or a reflection/loader failure. If the only thing that
can throw is our own NRE / IndexOutOfRange / InvalidCast, don't wrap it — fix the bug or let it
crash.

- ✅ `catch (IOException …)` around mesh file reads — external IO failure.
- ❌ `try { ... } catch { /* ignore */ }` — silent swallow turns bugs into mysteries.
- ❌ `try/catch` around our own code "to be safe."

Test harness code (asserting that something throws) is exempt — that's the test's purpose.

---

## 4. Fail-fast at boundaries

- Don't pre-validate with `Debug.Assert(x != null)` at every method entry — access it; the NRE
  is the report.
- Validate **public-API arguments** explicitly (`ArgumentNullException`, `ArgumentException`,
  `FormatException`) with a clear message — that is the contract boundary (e.g. `OffReader`).
- Surface invariant violations loudly; never silent-skip.

---

## 5. Zero-alloc hot paths

This is a perf-sensitive library — the whole point of the migration (see `docs/MIGRATION.md §2`).
Per-element inner loops (Dijkstra relaxation over CSR, Heat-Method/Laplacian iterations,
per-vertex field passes) must allocate **0 managed bytes** after warmup.

- Operate on `NativeArray`/SoA (`MeshData.Positions`/`Triangles`/`AdjOffsets`/`AdjNeighbours`),
  never per-element classes. No `List<int>`-per-vertex, no `Path`-node-per-step, no boxed keys —
  these were the original project's hot-loop GC sins (`docs/MIGRATION.md §2.B`).
- Cache callback delegates in a field (init once); never allocate a closure per frame/query.
- Structs for results/masks; no LINQ, no boxing, no per-call collections on the hot path.
- Topology/I-O construction is allowed to allocate (it's sequential, one-shot) — keep it out of
  the per-query path. Build CSR once, reuse.

---

## 6. Centralize cross-cutting behavior

When the same behavior appears at a second call site, extract one shared method and route both
through it — don't copy-paste-and-tweak. A fix then lands in one place for every caller
(e.g. `MeshBuilder.BuildAdjacency` is the single source of truth for CSR construction, used by
both `OffReader` and any future mesh source).

---

## 7. Verify before "done"

- Core (engine-free check where possible): compile + run the Core/Tests assemblies and confirm
  `failed=0` — e.g.
  `csc -nologo -warn:0 -out:/tmp/compgeo.exe Assets/CompGeo/Runtime/Core/**/*.cs Assets/CompGeo/Tests/Runtime/*.cs`
  (stub `Unity.Mathematics`/`Unity.Collections` as needed), or run the Unity Test Runner.
- Unity: console clean (errors + warnings), and a visual/behavior check for anything that runs.
- Hot-path changes: re-confirm GC = 0 (`GC.GetAllocatedBytesForCurrentThread()` delta over N
  iterations) and validate geodesic output against the `M for man0` ground-truth fixture.
