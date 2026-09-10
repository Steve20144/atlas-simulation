# Coanda foil evaluation (2026-09-10)

Scenario `atlas_phase01_cad`: CAD positions, reference point at the dashboard volume centroid,
placeholder mass 12 kg and the two-point XFly curve (33.3 N per motor). Stock PX4 concept, hover
collective. All numbers are relative until the weights export and a measured fan curve exist.

## The Coanda model used

The eight wing motors blow aft into curved foils. The jet follows the convex surface by the
Coanda effect and detaches when the wrap exceeds a separation angle that depends on the jet
thickness h over the surface radius R:

    theta_sep = 245 deg * exp(-1.64 * h / R)

For the PHASE_0.1 foils the channel walls curve down about 75 mm over the first 160 mm, which is a
surface radius of about 0.25 m; with the 80 mm duct exit as jet thickness, h / R = 0.32 and the
jet stays attached up to about 145 degrees. Thrust retained while attached is 1 minus 0.10 per
90 degrees of turning; a separated jet leaves at the separation angle with a further 30 percent
loss. All four constants are engineering estimates exposed in the UI (Coanda surface block) and in
the scenario JSON (`foils[].coanda`) and must be calibrated on the rig: measure the deflected jet
angle and the thrust at 45, 90 and 120 degrees of wrap.

Consequences for the design: any wrap up to about 145 degrees is available on the current surface;
a thinner exit slot or a larger radius extends that; thicker jets or tighter radii shorten it.

## Why 0 to 90 degrees never trims

Every wrap below 90 degrees leaves all eight jets with a forward component and nothing on the
aircraft cancels it, so a level hover trim does not exist. This is independent of the Coanda
model. At exactly 90 degrees the aircraft hovers (9436 W, headroom 0.59, roll 26.6 N m, pitch
18.0 N m) but has no yaw authority at all. Above 145 degrees the jet separates.

## What the Coanda-limited sweep finds

Same wrap angle on both foils, 0 to 180 degrees in 15 degree steps: 90 is the only controllable
geometry (no yaw). 150, 165 and 180 are flagged "jet separates".

Segmented foil, one wrap per motor pair, 45/90/135 degrees (81 combinations, 42 controllable,
minimum headroom 0.10), ranked by hover power:

| wrap outer to inner | hover W | headroom | roll N m | pitch N m | yaw N m |
|---|---|---|---|---|---|
| 90 / 90 / 135 / 45 | 10429 | 0.47 | 23.7 | 18.0 | 6.4 |
| 90 / 90 / 45 / 135 | 10443 | 0.46 | 24.0 | 18.0 | 6.6 |
| 90 / 135 / 90 / 45 | 10456 | 0.48 | 21.1 | 19.0 | 8.1 |
| 90 / 135 / 45 / 90 | 10476 | 0.46 | 22.5 | 18.0 | 9.9 |
| 135 / 90 / 90 / 45 | 10486 | 0.49 | 18.7 | 20.3 | 9.7 |

Segmented foil, 30 to 150 in 30 degree steps (625 combinations, 120 controllable with at least
5 N m yaw), ranked by yaw per kilowatt:

| wrap outer to inner | hover W | headroom | roll N m | pitch N m | yaw N m | yaw per kW |
|---|---|---|---|---|---|---|
| 30 / 120 / 60 / 120 | 11379 | 0.37 | 15.4 | 12.5 | 16.4 | 1.44 |
| 30 / 120 / 120 / 60 | 11312 | 0.40 | 16.5 | 13.7 | 16.2 | 1.43 |
| 30 / 120 / 90 / 120 | 11335 | 0.36 | 19.2 | 11.9 | 16.1 | 1.42 |
| 120 / 30 / 60 / 120 | 11271 | 0.39 | 18.6 | 16.4 | 15.1 | 1.34 |

The most yaw in that grid is 16.5 N m (30 / 120 / 30 / 120) at 12414 W and headroom 0.21.

## Gain and cost relative to the pure-lift baseline

Baseline: all foils at 90 degrees, 9436 W, no yaw.

- Cheapest yaw (90 / 90 / 135 / 45): +10.5 percent hover power buys 6.4 N m yaw; roll drops from
  26.6 to 23.7 N m, pitch unchanged, headroom 0.59 to 0.47.
- Best yaw per kilowatt (30 / 120 / 60 / 120): +20.6 percent hover power buys 16.4 N m yaw; roll
  15.4 N m, pitch 12.5 N m, headroom 0.37.
- The Coanda thrust loss at 120 degrees of wrap is 13 percent per motor, at 135 degrees 15
  percent; that is most of the power penalty of the yaw-capable geometries.

Compared with the earlier lossless vane model, the Coanda losses raise every hover figure by
roughly 8 to 12 percent and remove the 150 degree options; the ranking of geometries is otherwise
unchanged.

## Reproduce

    uv run --project backend python scripts/sweep_tilt.py scenarios/atlas_phase01_cad.json --tilts 0:180:15 --min-headroom 0
    uv run --project backend python scripts/sweep_tilt.py scenarios/atlas_phase01_cad.json --tilts 45,90,135 --grouping per_pair --min-headroom 0.1
    uv run --project backend python scripts/sweep_tilt.py scenarios/atlas_phase01_cad.json --tilts 30:150:30 --grouping per_pair --min-headroom 0.1 --min-yaw 5 --rank-by yaw_per_kW
