# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Pantograph Studio is an in-browser linkage simulator. Users put rigid bars with evenly spaced holes on a table, join holes into pivots, pin a hole, choose a tracer and up to 5 pens, then trace an image so the pens draw scaled, rotated or flipped copies.

It is plain static HTML/CSS/JS: `index.html`, `styles.css`, `app.js`. There is no build step, package manager, linter or test suite, and the directory is not a git repo.

## Running

```sh
python3 -m http.server 5178   # then open http://localhost:5178
```

`.claude/launch.json` sets up the same server. For debugging, the console exposes `window.pantograph = { state, solve, buildPreset }`.

## Architecture (`app.js`)

All the code is in one IIFE, split into `// ---------- Section ----------` blocks. Put new code in the block it belongs to.

**Model.** All state lives in the single `state` object. Bars are `{id, n, p:[p0,p1], holes, arms, color}`: `n` is the length in hole spaces (`UNIT` = 30px) and `p` holds the two endpoint positions. Links, pins, the tracer and pens all point to holes as `{bar, h}` refs and never store positions. A hole is stored in the bar's own frame as `{t, s}`, where `t` runs along the bar and `s` is the sideways offset. Its world position is `pointAt(bar, t, s) = p0 + t·d + s·perp(d)`. Angled rigid arms (`addArm`) just add more holes with `s ≠ 0` to an existing bar, so a bar can be L- or V-shaped and the solver needs no extra case for it.

**Joints.** `computeGroups()` runs union-find over `state.links`, so a joint is any set of holes joined by links. Code that asks whether two holes are the same joint (`sameJoint`, `penAtJoint`, removing or separating parts) calls it each time instead of caching the result. Each joint gets at most one pen, and pens can't share a joint with the tracer or a pin.

**Solver.** This is position-based constraint projection. `P` is linear in `p0` and `p1`, so the hole-to-target (`projPoint`) and hole-to-hole (`projLink`) projections have closed-form solutions, spread across both endpoints by `weight(t,s)`. `projLen` keeps each bar's length fixed. `solve(iters, drag)` applies the drag target, then pins, then links, then lengths, and finishes with a few extra "settle" iterations that leave out the drag. While dragging, `stepDrag()` moves the dragged point toward the mouse in steps of at most `STEP` px (up to `MAX_SUB` substeps per frame). It calls `solve` and `recordPen()` after each step, which keeps pen lines smooth and lets it detect a linkage that has stopped moving (`d.stuck`).

**Rendering.** There are two canvases. The visible `#board` is redrawn every frame by `render()` from the `requestAnimationFrame` loop in `frame()`. A separate offscreen `ink` canvas builds up pen strokes as they are drawn (`inkSegment`). It is fully redrawn from `state.strokes` (`rerenderInk`) only after a resize, undo, clear, or a change to the guides setting. Each stroke stores `pts` (the pen path), and the first stroke in a group also stores `gpts` (the tracer guide path). One tracing gesture makes one stroke per pen, all with the same `group`, so a single undo removes all of them.

**UI sync.** Call `changed()` after any change to the mechanism. It rebuilds the build checklist, including the Grübler mobility count, where the target is 2, and rebuilds the pen list. The HUD shows the scale and angle range for each pen during a trace, so you can see if a copy is distorted. Tool modes are listed in `HINTS` and `KEYS` (V/P/T/N/X/I/D), and `setMode()` switches between them.

**Presets.** Quick assemble options are spec strings such as `scale:0.5`, `flip:2` and `tilt:30`, and `buildPreset` reads them. `scale` and `flip` build the classic 4-bar pantograph. Pinning it at the end gives a copy the same way up (×k), and pinning it at the middle joint gives an upside-down copy (×−(1−k)/k). `tilt` builds Sylvester's skew pantograph (plagiograph), a hinged rhombus with welded arms that makes a same-size copy turned by θ. A flat linkage cannot mirror or shear an image. The UI says so, so don't add presets that promise either.

## Conventions

- Orange (`TRACER_COLOR`) is only for the tracer. Pens choose from `PEN_COLORS`, and two pens never share a colour.
- Styling uses the CSS custom properties defined on `:root` in `styles.css`.
