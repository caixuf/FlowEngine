# Debug 3D Scene

Use this skill when debugging or modifying FlowBoard's Three.js 3D visualization.

**Golden rule:** the road, car, obstacles, lane markings, and camera form a single
integrated system.  Change one, verify all.  Never tune parts in isolation.

## Architecture — think in systems

```text
┌─────────────────────────────────────────────────────┐
│  Scene View (the whole thing)                       │
│  ┌─ Road system ─────────────────────────────────┐ │
│  │  Asphalt + Edge lines + Shoulders + Dashes    │ │
│  │  → Must move as ONE unit during curves        │ │
│  └───────────────────────────────────────────────┘ │
│  ┌─ Ego vehicle ─────────────────────────────────┐ │
│  │  Position from deadreckon.js (smoothX/Z/Hdg)  │ │
│  │  deadreckon.js is the SINGLE engine:           │ │
│  │    updateDeadReckon() ← SSE data arrives       │ │
│  │    tickDeadReckon()   ← every rAF frame        │ │
│  │  scene3d.js is a PURE consumer — never writes  │ │
│  └───────────────────────────────────────────────┘ │
│  ┌─ Obstacles ───────────────────────────────────┐ │
│  │  World-space positions from update3D()        │ │
│  │  Extrapolated by velocity per frame            │ │
│  └───────────────────────────────────────────────┘ │
│  ┌─ Camera ──────────────────────────────────────┐ │
│  │  Follows ego, lerp-smooth                      │ │
│  │  Overridable via window._debugCam              │ │
│  └───────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
```

**Coordinate mapping:**

```text
sim_world  →  Three.js
────────────────────────
x          →  X (forward)
y          →  Z (lateral, right = positive)
z (up)     →  Y (up)

monitor_node sends:
  "lane": {"width": 3.5, "count": 2, "center": 0.0}
  "ego":  {"x": ..., "y": ...}   // y=-1.75 = left lane, y=+1.75 = right lane
```

## Road curve — the two approaches

### Approach A: Segment groups (USE THIS)

Road split into N × SEG_LEN groups.  Each group = `THREE.Group` containing
asphalt slice + 2 edge lines + 2 shoulders.  Curve shifts `group.position.z`
as one unit.  No vertex deformation.

**Pros:** Edge lines CANNOT separate from asphalt — they are children of the
same Group, which gets one atomic Z shift.
**Cons:** Steps between segments.  Fix: make SEG_LEN small enough.

**SEG_LEN formula:** step_size ≈ curve_offset / (curve_length / SEG_LEN).
For a 260m/9m curve: SEG_LEN=2m → step < 7cm → invisible.

### Approach B: Vertex deformation (fragile — avoid)

Single long BoxGeometry(width, h, d, XSEG, 1, 1).  Curve adds
`_curveShiftAt(vertexX)` to each vertex Z.

**Cons that make it unreliable:**

- Asphalt, edge lines, shoulders are **separate meshes**.  Vertex positions
  come from their **local** geometry buffers.  Even with identical XSEG,
  `computeVertexNormals()` after deformation can produce different normals
  for meshes of different sizes, causing lighting divergence at seams.
- Calling `_applyRoadCurve` a second time **adds** to already-shifted Z
  (the key guard prevents this for identical params, but any param change
  or scene rebuild causes accumulation).
- Three.js `BoxGeometry` vertex layout differs subtly by dimensions.
- Bottom line: vertex deformation across **separate meshes** is NOT
  guaranteed to produce identical world-space results.

**Use Approach A.  Pick SEG_LEN small enough that steps are invisible.**

## Common issues — diagnosis table

| Symptom | Root cause | Fix |
|---------|-----------|-----|
| Top half of 3D view is black/empty | 2D canvas not hidden, stacked above 3D div | Add `display:none` to `#scene2d` in HTML |
| Car not centred in lane | `ROAD_HALF` doesn't match `lane_width × lane_count / 2` | Read `lane` from topology metrics; match road builder |
| Lane lines invisible | Material too dark/thin | emissive ≥ `0x333333`, roughness ≤ 0.3, width ≥ 0.22m, thickness ≥ 0.02m |
| Lines separate during curve | Vertex deformation across separate meshes | Switch to segment groups (Approach A) |
| Jagged road edges during curve | SEG_LEN too large | Reduce SEG_LEN: 2m for demo-scale curves |
| Double-speed animation | Two rAF loops calling `_renderFrame()` | Only `anim3D()` in scene3d.js drives render; remove `renderLoop` from app.js |
| `_animT` shared across modules | Imported from deadreckon.js | Make it `let _animT = 0` local to scene3d.js |
| Camera shows too much sky | look-ahead too far (shallow pitch) | lookX=8m, camera height=5m, back=15m |

## Layout — make sure only ONE view is visible

```html
<!-- In index.html: 2D starts hidden, 3D starts visible -->
<canvas id="scene2d" style="display:none;..."></canvas>
<div id="scene3d" style="...background:#5588aa;"></div>
```

`switchSceneView('2d')` hides 3D and shows 2D.  `switchSceneView('3d')` does the reverse.
Both must never be visible simultaneously.

## Debugging with debug3d.html

```text
http://localhost:8800/debug3d.html
```

This standalone page loads the SAME `scene3d.js`.  Use it to:

- Tweak camera params via sliders → `window._debugCam`
- Set ego position manually (X, Y/Lane, speed)
- Apply test curves (start, length, offset)
- Toggle wireframe to see mesh boundaries
- Toggle fog to check visibility
- Export state as JSON snapshot

**Workflow when you can't see the screen:**

1. Ask user to open `debug3d.html`
2. Ask them to move ONE slider and describe what changes
3. Use wireframe mode to verify mesh alignment
4. Test the curve in isolation before running the full demo

## Checklist before committing 3D changes

- [ ] Road width matches `lane_width × lane_count` from sim data
- [ ] Edge lines and asphalt use segment groups (Approach A), not vertex deformation
- [ ] SEG_LEN ≤ 2m for demo-scale curves
- [ ] Lane markings have emissive + low roughness (visible in all lighting)
- [ ] `#scene2d` has `display:none` by default
- [ ] Only ONE rAF loop exists (check `grep -r 'requestAnimationFrame\|_renderFrame'`)
- [ ] `_animT` is local to scene3d.js
- [ ] `deadreckon.js` is the ONLY module with lerp logic — no local lerp in scene3d.js or scene2d.js
- [ ] `scene3d.js` NEVER calls `updateDeadReckon()` or writes `_dr.last*` — only reads `getDeadReckonState()`
- [ ] `scene2d.js` has NO monkey-patch on `updateAll()`
- [ ] Heading lerp uses angle wrapping (`while (dh > Math.PI) dh -= 2*Math.PI`)
- [ ] dt is clamped (`dt > 0.1 ? 0.1 : dt`) in tickDeadReckon()
- [ ] Camera defaults tested with debug3d.html sliders
- [ ] Tested: straight road → curve → post-curve (all three phases)
