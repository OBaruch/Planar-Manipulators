# Kinematics Models

[← Back to README](../README.md)

This page is reference material taken **directly from the DH definitions and formulas in the original scripts**. It restates what the code implements in mathematical notation. It does not add new models.

DH parameter order follows the Robotics Toolbox `Revolute`/`Prismatic` constructors: `a` (link length), `alpha` (twist), `d` (offset), `theta` (joint angle), plus `offset` (joint-coordinate offset). Lengths are in meters (**Inferred** from the values used, e.g. 0.35).

---

## 1. Planar manipulator

### 2-DOF (R-R), in `PlanarManipulator2DOF.m`

| Link | Joint | a | α | d | θ |
|---|---|---|---|---|---|
| 1 | R | a1 = 0.35 | 0 | 0 | θ1 |
| 2 | R | a2 = 0.25 | 0 | 0 | θ2 |

Target: `(tx, ty) = (0.4, 0.4)`.

Closed-form inverse kinematics as coded:

```
θ2 = ± acos( (tx² + ty² − a1² − a2²) / (2·a1·a2) )      (configuration 1: −, configuration 2: +)
θ1 = atan2(ty, tx) − asin( a2·sin θ2 / √(tx² + ty²) )
```

Reachability check used: `tx² + ty² − a1² − a2² > 2·a1·a2` → not reachable.

### 3-DOF (R-R-R), in `act8.m` and `Act9.m`

| Link | Joint | a | α | d | θ |
|---|---|---|---|---|---|
| 1 | R | a1 | 0 | 0 | θ1 |
| 2 | R | a2 | 0 | 0 | θ2 |
| 3 | R | a3 | 0 | 0 | θ3 |

Numeric values in `Act9.m`: `a1 = 0.35, a2 = 0.35, a3 = 0.25`.

End-effector position (hand-written in `Act9.m`):

```
x = a1·cos θ1 + a2·cos(θ1+θ2) + a3·cos(θ1+θ2+θ3)
y = a1·sin θ1 + a2·sin(θ1+θ2) + a3·sin(θ1+θ2+θ3)
z = 0
```

---

## 2. Anthropomorphic manipulator (R-R-R)

| Link | Joint | a | α | d | θ |
|---|---|---|---|---|---|
| 1 | R | 0 | π/2 | d1 | θ1 |
| 2 | R | a2 | 0 | 0 | θ2 |
| 3 | R | a3 | 0 | 0 | θ3 |

Numeric values: `d1 = 0.35, a2 = 0.3, a3 = 0.25` (in both `PlanarManipulator2DOF.m` and `Act9.m`).

Closed-form IK in `PlanarManipulator2DOF.m`, target `(0.3, 0.2, 0.35)`:

```
θ1 = atan2(ty, tx)
θ3 = ± acos( (tx² + ty² + (tz−d1)² − a2² − a3²) / (2·a2·a3) )      (configuration 1: +, configuration 2: −)
θ2 = atan2(tz−d1, √(tx²+ty²)) − asin( a3·sin θ3 / √(tx²+ty²+(tz−d1)²) )
```

End-effector position (hand-written in `Act9.m`):

```
x = cos θ1 · (a2·cos θ2 + a3·cos(θ2+θ3))
y = sin θ1 · (a2·cos θ2 + a3·cos(θ2+θ3))
z = d1 + a2·sin θ2 + a3·sin(θ2+θ3)
```

---

## 3. Cylindrical manipulator (R-P-P)

`act8.m` / `Act9.m` model:

| Link | Joint | a | α | d | θ |
|---|---|---|---|---|---|
| 1 | R | 0 | 0 | d1 | θ1 |
| 2 | P | 0 | π/2 | d2 | 0 |
| 3 | P | 0 | 0 | d3 | 0 |

`PlanarManipulator2DOF.m` uses the same chain with joint offsets: link 1 `offset = π/2`, link 2 `offset = d2off = 0.15`, link 3 `offset = d3off = 0.15`, and `d1 = 0.35`.

Closed-form IK in `PlanarManipulator2DOF.m`, target `(0.5, 0.25, 0.8)`:

```
θ1 = atan2(ty, tx)
d2 = tz − d1 − d2off
d3 = sqrt(tx² + ty² − d3off)          (as written in the code)
```

End-effector position (hand-written in `Act9.m`, `d1 = 0.35`):

```
x =  d3 · sin θ1
y = −d3 · cos θ1
z =  d1 + d2
```

---

## 4. Spherical manipulator (R-R-P)

`act8.m` / `Act9.m` model:

| Link | Joint | a | α | d | θ |
|---|---|---|---|---|---|
| 1 | R | 0 | π/2 | d1 | θ1 |
| 2 | R | 0 | −π/2 | 0 | θ2 |
| 3 | P | 0 | 0 | d3 | 0 |

`PlanarManipulator2DOF.m` uses the same chain with link 2 `offset = −π/2` and link 3 `offset = d3off = 0.35`, plus `d1 = 0.35`.

Closed-form IK in `PlanarManipulator2DOF.m`, target `(0.5, 0.25, 0.8)`:

```
θ1 = atan2(ty, tx)
θ2 = atan2(tz − d1, √(tx² + ty²))
d3 = √(tx² + ty² + (tz−d1)²) − d3off
```

End-effector position (hand-written in `Act9.m`, `d1 = 0.35`):

```
x = −d3 · cos θ1 · sin θ2
y = −d3 · sin θ1 · sin θ2
z =  d1 + d3 · cos θ2
```

---

## Jacobian formulation (`act8.m`)

For a 3-joint arm the scripts build the geometric Jacobian:

```
J = [ Jv ]      Jv(:, i) = ∂p / ∂q_i              (p = end-effector position from fkine)
    [ Jw ]      Jw(:, i) = z_{i-1}  for a revolute joint
                         = 0        for a prismatic joint
```

where `z0 = [0 0 1]ᵀ` and `z_{i-1}` is the third column of the rotation part of `T0,i-1`.

## Numerical inverse kinematics (`Act9.m`)

Discrete closed-loop inverse kinematics with the Jacobian pseudo-inverse:

```
e_k     = p_d − p(q_k)
q̇_k     = J⁺ · K · e_k             K = I₃,   J⁺ = pinv(J)
q_{k+1} = q_k + Δt · q̇_k           Δt = 0.1,  k = 1 … 100
```

As implemented, `J` is evaluated once at the initial configuration `q_0` (see [code-overview.md](code-overview.md#srcact9m-jacobian-based-numerical-inverse-kinematics-actividad-9-inferred)).
