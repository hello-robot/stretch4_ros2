# Line-sensor filtering

This package turns raw line-sensor ranges into hazard points the base can act
on: **obstacles**, **small drops**, **deep drops / cliffs**, and **degraded
sectors**. 

```python
from stretch_core.line_sensor_filter import LineSensorSource, LineSensorConfig

source = LineSensorSource(geometry, sensor_names, LineSensorConfig())
hits = source.process(status)     # status: {sensor_name: {"ranges": [...]}}
hits.obstacle_xy, hits.deep_drop_xy, hits.degraded_xy   # (N, 2) arrays
```

---

## The shape of the package

| File | What lives there | Front door |
|---|---|---|
| `source.py` | The orchestrator — owns state, runs the frame | `LineSensorSource.process()` |
| `config.py` | Every tunable, grouped by stage | `LineSensorConfig` |
| `hits.py` | The vocabulary and the output object | `BinClass`, `LineSensorHits` |
| `geometry.py` | Range → point math, shared by every stage | `Projector` |
| `classify.py` | Label one returning bin | `classify_bin()` |
| `gloss.py` | Reject glossy-floor phantoms | `FlipTracker`, `quarantine_spray_candidates()` |
| `shape.py` | Reject spray / streaks / noise by shape | `ShapeGate` |
| `confirm.py` | Require a hazard to persist | `bin_confirmed()` |
| `nulls.py` | Read the *silences* → cliffs & degraded | `NullEvidenceDetector` |
| `arrays.py` | Tiny shared numpy helpers | `as_range_array()`, `runs()` |

Three layers:

- **Foundation** — `hits`, `config`, `geometry`, `arrays`. The words, the knobs,
  and the math everything else stands on. `Projector` is the important one: it
  is the *only* place range becomes a point.
- **Stages** — `classify` → `gloss` → `shape` → `confirm`, and `nulls` running
  alongside. Each is one step in a returning bin's life (or, for `nulls`, the
  bins that never returned).
- **Orchestrator** — `source`. Holds the per-frame memory and calls the stages
  in order. If you only read one file, read this one.

---

## The one idea underneath everything: range → z

The sensor measures **range**, not height. Each bin is compared to its own
clear-floor reference (from the tare); the difference, expressed as a height, is
**z**.

```
z ≈ (floor_reference_range − measured_range) × sin(down_pitch)
```

- Beam reaches the floor where expected → `z ≈ 0` (**free**).
- Something blocks it early → range short → `z > 0` (**obstacle height**).
- Floor is lower than expected → range long → `z < 0` (**a drop**).

Everything downstream reasons about `z`. → `geometry.py::Projector.project`.

---

## From range to a point — `geometry.py`

`Projector` is built once and wraps the hardware `geometry`. It answers the
geometry questions every stage asks, and caches the parts that don't depend on
the live ranges:

- `project(sensor_idx, ranges)` — the (x, y, z) of every bin.
- `local_contrast(z, ranges)` — z minus the rolling median of its neighbours,
  so a whole-array floor shift can't masquerade as an object.
- `floor_intersections`, `bin_bearings`, `sensor_origin`, `radial_metrics` —
  where a ray meets the floor, its world bearing, the sensor mount point, and
  the shape descriptors of a run of bins.

Nothing here decides "hazard" — it only measures.

---

## The frame, end to end — `source.py`

`process()` reads as the pipeline. Each stage is a single call:

```
Stage 1  classify        for every returning bin → BinClass        classify.py
Stage 2  gloss quarantine drop phantom near-field arcs (context)    gloss.py
Stage 3  shape gate       group into runs, drop spray/streaks/noise  shape.py
Stage 4  confirm          require persistence across frames          confirm.py
Stage 5  null evidence    read no-return bins → cliffs / degraded    nulls.py   (parallel)
Stage 6  package          fill in LineSensorHits                     hits.py
```

`LineSensorSource` holds the only mutable per-frame state — two frame-history
deques and a frame counter. Everything stateful *within* a stage (flip counters,
degraded latches) lives inside that stage's own helper, built once in
`__init__`, so the hot loop allocates nothing extra.

---

## Classifying one bin — `classify.py`

`classify_bin(cfg, z, contrast, bin_reliable)` is a pure decision: given a bin's
height and local contrast, return its `BinClass`. Reliable (tared) bins use the
sensitive deviation bands; untared bins fall back to coarse absolute-height
bands. Obstacle-family and drop-family bins become **candidates**; the rest are
dropped.

---

## Rejecting glossy-floor phantoms — `gloss.py`

Shiny floors throw back compact near-field arcs that *look* exactly like a real
object. Shape can't tell them apart, so this stage uses context:

- `FlipTracker` — a **time** signature. A gloss bin flickers on/off constantly;
  a real object flips once per approach. A decayed flip count flags the
  flickerers.
- `quarantine_spray_candidates` — a **space** signature: if several sensors see
  near-field hazards at once, no single object could cause it, so those
  candidates are rerouted to SPRAY (a debug-only class), except runs that look
  like a genuine pressed-against-the-base object.

---

## Filtering by shape — `shape.py`

`ShapeGate.gate()` groups the survivors into contiguous per-sensor runs and
tests each: a thin monotonic streak is **spray**; a tiny isolated blob is
**point noise**; a run that is too long or too spread out is rejected. Runs
separated by a small gap are merged and re-tested so a broken-up streak can't
slip through in pieces.

---

## Making it wait a beat — `confirm.py`

A hazard must appear on enough consecutive frames before it publishes.
`confirm_frames_for_bin` says how many (strong obstacles clear fast; marginal
ones and anything seen during active gloss wait longer); `bin_confirmed` checks
the frame history for that streak. Survivors are the **promoted** hazards.

---

## Reading the silences — `nulls.py`

Everything above is about bins that *return*. `NullEvidenceDetector.detect()`
handles the bins that don't. A null run counts as evidence only where the bins
are expected to return on clear floor, only if it is long enough, and only if
the same region was mostly null in the previous frame. The surviving runs are
then classified in order.

A run containing the far-return sentinel (`5.09`) is classified as a
**probable cliff**. Any run on another sensor whose bearings fall within that
void's angular span is also classified as a **probable cliff**, since a ledge
is continuous across the floor plane and does not stop at a sensor boundary.

A run adjacent to an obstacle is classified as an **occlusion shadow**, while a
run on a sensor reporting a strong nearby return is classified as a
**suppressed exposure**. Both are **benign**.

A run adjacent to a drop bin, or aligned with one on another sensor, is also
classified as a **probable cliff**.

Anything left is treated as **dark floor**: benign but unexplained. If the
smoothed, hysteretic fraction of unexplained nulls exceeds the configured
threshold, that sensor is marked as **degraded**. A degraded sensor causes the
robot to slow down as a precaution but does not stop it.

---

## What a frame produces — `hits.py`

`process()` returns a `LineSensorHits`. The fields the hazard layer consumes:

| Field | Meaning | Severity |
|---|---|---|
| `obstacle_xy` | confirmed obstacles | stop |
| `small_drop_xy` | confirmed 2–10 cm drops | soft hazard |
| `deep_drop_xy` | confirmed returning drops deeper than `cliff_max_drop_m` | stop (lethal) |
| `probable_cliff_xy` | cliff-typed null runs, at their nearest possible floor intersection | stop (lethal) |
| `degraded_xy` | sectors that lost floor coverage | slow down |

The remaining `raw_*`, `spatial_*`, `benign_null_xy` fields are debug views of
the intermediate stages.

---

## Where each knob lives

Every tunable is a field on `LineSensorConfig` (`config.py`), grouped so the
knobs for a stage sit together. Change a default there, or override it at
runtime: `line_sensor_publisher` declares one ROS parameter per config field,
named after the field, so a knob is reachable from `config/line_sensors.yaml`
the moment it exists here.

---

## The public API

Import only these from `stretch_core.line_sensor_filter`:

- `LineSensorSource` — build once, `.process(status)` per frame.
- `LineSensorConfig` — the tunables.
- `LineSensorHits` — the result.
- `BinClass` — the per-bin labels, if you need them.
- `as_range_array` — coerce a raw ranges payload to float64.

`Projector`, `ShapeGate`, `FlipTracker`, `NullEvidenceDetector` and the
per-stage functions are internal — reach for them only when working inside the
package.
