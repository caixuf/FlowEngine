# Debug 3D Scene

Use this skill when debugging FlowBoard's Three.js 3D visualization — camera issues, road/lane alignment, curve deformation, layout problems, or rendering glitches. Always prefer visual verification over static code analysis.

## Workflow

### 1. Open the debugger page FIRST

```
http://localhost:8800/debug3d.html
```

This standalone page loads the SAME `scene3d.js` module with:
- **Camera sliders** (back, height, side, lerp) → writes `window._debugCam`
- **Ego position controls** (X, Y/Lane, speed, heading) → drives `update3D()` at 10Hz
- **Curve test** (start X, length, offset) → calls `_applyRoadCurve()` directly
- **Obstacle / LiDAR test** (add mock objects)
- **Render options** (wireframe, axes, fog toggle)
- **FPS + debug info** overlay

### 2. Screenshot or inspect visually

Use browser MCP (`@hisma/server-puppeteer`) if available:
```
navigate to debug3d.html → tweak sliders → screenshot → compare
```

Without MCP: ask the user what they see and which numbers reproduce the issue.

### 3. Isolate the problem

| Symptom | Likely cause | Check |
|---------|-------------|-------|
| Black top/bottom bars | Canvas size ≠ container size | `resize3D()` timing; 2D canvas not hidden with `display:none` |
| Lane lines invisible | Material too dark/thin | emissive, roughness, line width, Y position vs asphalt |
| Lines zigzag during curve | Segmented meshes with per-segment Z shift | Use single long mesh + XSEG vertex deformation instead |
| Lines separate from road during curve | Different XSEG or different deformation path | Ensure edge lines & asphalt use SAME BoxGeometry(width, h, d, **XSEG**, 1, 1) |
| Road becomes diagonal | BoxGeometry with XSEG=1 (default) | Set XSEG ≥ 200 for smooth vertex deformation |
| Car not on lane | sim world `lane_count` ≠ 3D road lanes | Match ROAD_HALF = lane_width × lane_count / 2 |
| Double rendering / fast animation | Two rAF loops calling `_renderFrame()` | Only `anim3D()` in scene3d.js should drive the render loop |
| Whole scene missing | WebGL context lost | Check `_glLost` flag; `debug3d.html` info panel shows GL status |

### 4. Key coordinate conventions

```
sim_world  →  Three.js
────────────────────────
x          →  X (forward)
y          →  Z (lateral, right = positive)
z (up)     →  Y (up)

monitor_node sends:
  "lane": {"width": 3.5, "count": 2, "center": 0.0}
  "ego":  {"x": ..., "y": ...}   // y=-1.75 left lane, y=+1.75 right lane
```

### 5. Common fixes checklist

- [ ] `#scene2d` canvas has `display:none` by default in HTML (not both visible)
- [ ] `scene3d` div background matches Three.js `scene.background` color
- [ ] Road uses `BoxGeometry(L, h, w, XSEG≥200, 1, 1)` for smooth curves
- [ ] Edge lines use SAME XSEG as asphalt, tagged `isEdge` (not `isLaneMark`)
- [ ] Center dashes use individual 4.5m segments tagged `isLaneMark`
- [ ] `isLaneMark` dashes get `position.z = baseZ + curveShift` (good for <5m segments)
- [ ] Long meshes get per-vertex Z deformation (identical for same XSEG)
- [ ] Lane markings at y=0.048 (on asphalt surface at y=0.05)
- [ ] Marking material: emissive ≥ `0x222222`, roughness ≤ 0.4
- [ ] Only ONE `requestAnimationFrame` loop calls `_renderFrame()` (in `anim3D()`)
- [ ] `app.js` does NOT import or call `_renderFrame` directly
- [ ] `window._debugCam` respected in chase camera code
- [ ] `_animT` is a `let` in `scene3d.js` module scope (not imported from deadreckon)

### 6. The `_debugCam` protocol

`scene3d.js` chase camera reads `window._debugCam` on every frame:
```javascript
var dc = window._debugCam;
var back   = dc ? dc.back   : defaultBack;
var height = dc ? dc.height : defaultHeight;
var side   = dc ? (dc.side || 0) : 0;
var camLerp= dc ? dc.lerp   : 0.08;
```

`debug3d.html` writes to `window._debugCam` from its sliders. Any other page can do the same.

### 7. When you can't see the screen

1. Ask the user to open `debug3d.html`
2. Ask them to adjust specific sliders and describe what changes
3. Use the `debug3d.html` "Export State" button to get a JSON snapshot
4. Narrow down: toggle wireframe (`opt-wire` checkbox) to see mesh boundaries
5. Toggle fog off to check if fog is hiding content

### 8. Curve debugging

The demo scenario applies a curve at ~16s:
```
curve_start_x: 150, curve_length_m: 260, curve_offset_m: 9
```

In `debug3d.html`, set Curve sliders to these values, click "Apply Curve", then drive ego past X=150 to see the effect.

To verify vertex deformation correctness:
1. Enable Wireframe mode
2. Check that edge line vertices and asphalt vertices at the same X have the same Z shift
3. If they don't, the XSEG values differ or the meshes use different deformation paths
