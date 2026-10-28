# Phase 2.1 — AMR Physical Geometry and TF Convention

## Status

Phase 2.1 geometry baseline — provisionally frozen.

This document defines the mechanical envelope and coordinate convention
for Phase 2.2 robot-description modeling.

Do not use the obsolete 900 x 600 x 350 mm geometry.

---

## 1. Coordinate Convention

REP-103 convention:

+X = forward
+Y = left
-Y = right
+Z = upward

The robot longitudinal centerline is Y = 0.

The preliminary `base_link` reference is located at the
drive-wheel axle centerline and ground-contact reference.

---

## 2. Chassis

Overall chassis envelope:

Length = 1.800 m
Width  = 1.000 m
Height = 0.300 m

Chassis longitudinal envelope relative to chassis center:

X = -0.900 m to +0.900 m

Ground clearance:

0.025 m

Therefore chassis body occupies:

Z = 0.025 m to 0.325 m

The chassis visual/collision box center is:

Z = 0.175 m

---

## 3. Pallet / Payload Envelope

Pallet footprint:

Length = 1.200 m
Width  = 0.800 m

Payload stack height above deck:

Preliminary maximum = 1.200 m

For the Phase 2.1 stability model:

Payload mass = 400 kg
Payload CoG height = 0.600 m above ground

Payload CoG is modeled conservatively as a design parameter,
not as a finalized mechanical measurement.

---

## 4. Drive Wheels

Wheel diameter = 0.250 m
Wheel radius   = 0.125 m
Wheel width    = 0.080 m

Drive-wheel center separation:

0.800 m

Therefore:

Left wheel Y  = +0.400 m
Right wheel Y = -0.400 m

Drive axle X relative to chassis center:

+0.150 m

Drive-wheel center Z:

0.125 m

---

## 5. Caster

Caster diameter = 0.150 m
Caster radius   = 0.075 m

Caster longitudinal position:

X = -0.450 m relative to chassis center

Caster lateral position:

Y = 0

Caster center height:

Z = 0.075 m

This corresponds to a caster located 0.600 m behind
the drive axle.

---

## 6. Mass Model

Unloaded robot mass:

150 kg

Payload mass:

400 kg

Loaded robot mass:

550 kg

Preliminary unloaded robot CoG:

X = -0.012 m relative to chassis center
Z = 0.153 m

Relative to the drive axle:

X = -0.162 m

The inertial model remains a preliminary lumped model.
Final component inertias require finalized mechanical design.

---

## 7. Stability Design Assumption

Normal planned deceleration target:

1.0 m/s^2

Emergency/safety-stop target:

2.0 m/s^2

Preliminary static tipping design factor:

2.5x emergency deceleration

Required static tipping threshold:

5.0 m/s^2

### Current Loaded CoG Model

The unloaded 150 kg mass model was revised to distribute
the previous 8 kg caster mass across four 2 kg caster
assemblies at the frozen front/rear support positions.

Revised unloaded CoG:

X = -0.001333 m
Y = 0.000000 m
Z = 0.153167 m

Payload model:

Mass = 400 kg
CoG X = -0.125 m
CoG Z = 0.900 m

Loaded robot:

Mass = 550 kg
CoG X ≈ -0.091273 m
CoG Y = 0.000000 m
CoG Z ≈ 0.696318 m

### Longitudinal Braking Stability

Drive axle X:

0.000 m

Front caster support line:

X = +0.300 m

Loaded CoG distance behind front support:

≈ 0.391273 m

Using the preliminary static tipping model:

Critical forward-tip acceleration:

≈ 5.512 m/s^2

Required static tipping threshold:

5.000 m/s^2

Margin over emergency/safety-stop acceleration:

≈ 2.76x

Margin over the preliminary 2.5x design threshold:

≈ 1.10x

RESULT:

PASS — the current six-contact support geometry
exceeds the preliminary 5.0 m/s^2 tipping threshold.

### Lateral Stability — Preliminary Model

Required lateral support half-width:

≈ 0.355 m

This value uses the same 5.0 m/s^2 design threshold
as a preliminary common-axis simplification.

It is NOT an independently derived cornering/rollover
limit. Actual lateral stability must later be evaluated
using vehicle speed, turn radius, tire/caster contact,
and dynamic load transfer.

Actual caster half-width:

0.450 m

Preliminary lateral geometric margin:

≈ 0.095 m

RESULT:

PASS — preliminary support width exceeds the simplified
lateral requirement.

### Static Contact-Load Model

The six-contact system is statically indeterminate.
The exact load distribution between the front casters,
drive wheels, and rear casters cannot be derived from
geometry alone.

It requires caster/suspension stiffness and contact
compliance data.

Bounding case with front caster reaction treated as zero:

Rear support reaction ≈ 847 N
Drive-line reaction ≈ 4548 N

These values are a bounding calculation, not a prediction
of the actual six-contact load distribution.

### Stability Status

The longitudinal and preliminary lateral geometry now
passes the current design assumptions.

The model remains preliminary and must be revisited when
actual payload mass distribution, braking capability,
wheel/caster friction, contact compliance, and mechanical
design are finalized.


---

## 8. Important Design Status

The following values are preliminary engineering assumptions,
not finalized mechanical specifications:

- chassis dimensions
- pallet dimensions
- payload CoG
- unloaded component mass distribution
- emergency deceleration target
- static stability factor

These values must be revisited when the actual mechanical design,
payload characteristics, wheel selection, braking capability,
and facility constraints are known.

---

## 9. Obsolete Geometry

The previous baseline:

900 x 600 x 350 mm chassis
500 mm wheel separation
drive axle at chassis center
caster X = -350 mm

is obsolete and must not be used for subsequent modeling.

