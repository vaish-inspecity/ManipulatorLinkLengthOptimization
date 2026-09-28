# Robot Kinematics Notes: Angular Velocity, Jacobian & Singularities

*Craig's notation: `^A_B R` = orientation of {B} in {A}; `^A P` = vector P written in {A}; `s1 = sin θ1`, `c12 = cos(θ1+θ2)`.*

**How these notes flow**

```
pose R(Θ) → its rate Ṙ → angular velocity ω (world | body) → change of frame
→ matrix form S(ω) and its properties → angular velocities add → propagation along links
→ Jacobian (base | tool frame) → singularities → forces (τ = Jᵀ F)
```

---

## 1. Pose of a frame

The orientation of {B} in {A} is a rotation matrix whose columns are the axes of {B} written in {A}:

```
^A_B R = [ ^A X̂_B   ^A Ŷ_B   ^A Ẑ_B ]
```

It is orthonormal: `R^T R = I`, so `R⁻¹ = R^T` and `det R = +1`. Rotations compose by multiplication and **do not commute**: `^A_C R = ^A_B R · ^B_C R`. Adding the origin offset gives the point mapping `^A P = ^A_B R ^B P + ^A P_BORG`, packaged as the 4×4 homogeneous transform `T = [R  P; 0  1]`, with `T⁻¹ = [R^T  −R^T P; 0  1]`.

On an arm, every link carries a frame, and the transform from one joint frame to the next uses four parameters (modified DH: `a_{i-1}, α_{i-1}, d_i, θ_i`):

```
^{i-1}_i T = Rot_x(α_{i-1}) · Trans_x(a_{i-1}) · Rot_z(θ_i) · Trans_z(d_i)
^0_N T(Θ)  = ^0_1 T(θ1) · ^1_2 T(θ2) ··· ^{N-1}_N T(θN)        (forward kinematics)
```

The reverse problem, inverse kinematics, solves `^0_N T(Θ) = T_desired` for Θ. A solution exists only inside the workspace, is generally not unique (elbow-up / elbow-down), and has a closed form when three consecutive axes intersect (Pieper's criterion, e.g. a spherical wrist).

*→ Since Θ = Θ(t), both R and P change in time. How do we describe the rate of change of R?*

---

## 2. Angular velocity from the rotation matrix

`R R^T = I` holds at every instant, so differentiate it:

```
Ṙ R^T + R Ṙ^T = 0     ⇒     (Ṙ R^T)^T = −(Ṙ R^T)
```

So `Ṙ R^T` (and likewise `R^T Ṙ`) is **skew-symmetric**. A 3×3 skew-symmetric matrix has only three free entries, so it is a 3-vector ω in matrix form:

```
S(ω) = [  0   −ωz   ωy ]
       [  ωz   0   −ωx ]          S(ω) p = ω × p
       [ −ωy   ωx   0  ]
```

The direction of ω is the instantaneous axis of rotation and ‖ω‖ is the rotation rate (rad/s). Let ω be the angular velocity of {B} relative to {A}; it can be written in either frame (`^Aω ≡ ^AΩ_B`):

**World (spatial) frame**, components in {A}:

```
S(^Aω) = ^A_B Ṙ · ^A_B R^T          ⇔          ^A_B Ṙ = S(^Aω) · ^A_B R
```

Column by column, each axis of {B} simply swings about ω: `d/dt(^A X̂_B) = ^Aω × ^A X̂_B`.

**Body frame**, components in {B} (what a gyroscope fixed to the body reads):

```
S(^Bω) = ^A_B R^T · ^A_B Ṙ          ⇔          ^A_B Ṙ = ^A_B R · S(^Bω)
```

For a point fixed in {B}, `d/dt(R ^BQ) = Ṙ ^BQ = ^Aω × (R ^BQ)`, which is the familiar `v = ω × r`.

*→ Both definitions describe one physical vector. Equating them gives the frame transformation.*

---

## 3. Transforming ω between frames

Equate the two expressions for Ṙ: `S(^Aω) R = R S(^Bω)`, so `S(^Aω) = R S(^Bω) R^T = S(R ^Bω)`, hence

```
^Aω = ^A_B R · ^Bω          ^Bω = ^A_B R^T · ^Aω
```

ω transforms like any free vector: a pure rotation, no translation term. It is the same for every point of a rigid body, unlike linear velocity. At the matrix level the two definitions are related by a similarity transform, `S(^Aω) = R S(^Bω) R^T`.

*→ That similarity is one of several useful properties of S(·).*

---

## 4. Properties of S(ω) (matrix form)

| Property | Equation | Where it matters |
|---|---|---|
| Skew-symmetric | `S^T = −S`, zero diagonal | why `Ṙ R^T` encodes a 3-vector |
| Linear | `S(aω₁ + bω₂) = a S(ω₁) + b S(ω₂)` | angular velocities add (§5) |
| Cross product | `S(a) b = a × b = −S(b) a` | `v = ω × r` |
| Axis is the null vector | `S(ω) ω = 0`, so `det S = 0` (rank 2), eigenvalues `0, ±j‖ω‖` | S is not invertible |
| Square | `S(ω)² = ω ω^T − ‖ω‖² I` | Rodrigues formula |
| Similarity | `R S(ω) R^T = S(Rω)` | frame change (§3) |
| Rate of R | `Ṙ = S(ω) R = R S(R^T ω)` | integrating ω to get orientation |
| Exponential | constant ω = K̂θ̇: `R = e^{S(K̂)θ} = I + sinθ S(K̂) + (1 − cosθ) S(K̂)²` | angle-axis form of R |

*→ Linearity plus the vector nature of ω says something important about chains of rotating links.*

---

## 5. Combining rotations: angular velocities add

Take `^A_C R = ^A_B R · ^B_C R` and use the product rule:

```
^A_C Ṙ = ^A_B Ṙ · ^B_C R + ^A_B R · ^B_C Ṙ
       = S(^AΩ_B) ^A_C R + S(^A_B R ^BΩ_C) ^A_C R          (similarity property)
       = S( ^AΩ_B + ^A_B R ^BΩ_C ) ^A_C R
```

so, with `^AΩ_C` = angular velocity of {C} relative to {A}, written in {A}:

```
^AΩ_C = ^AΩ_B + ^A_B R · ^BΩ_C
```

Rotation *matrices* do not commute, yet angular velocities *add* as vectors once written in a common frame. Combined with `ω × r`, this gives the velocity of a point Q in a moving frame {B}:

```
^A V_Q = ^A V_BORG + ^A_B R ^B V_Q + ^AΩ_B × ( ^A_B R ^B Q )
```

(origin motion + motion inside {B} + the rotation term).

*→ An arm is exactly a chain of frames, each rotating about its own Z axis at rate θ̇, so we can add link by link.*

---

## 6. Propagating velocity along the links

Start at the base (`^0ω_0 = 0`, `^0v_0 = 0`). Each revolute joint adds `θ̇_{i+1}` about `^{i+1}Ẑ_{i+1}` (§5), re-expressed in the next frame (§3):

```
Revolute:   ^{i+1}ω_{i+1} = ^{i+1}_i R ^iω_i + θ̇_{i+1} ^{i+1}Ẑ_{i+1}
            ^{i+1}v_{i+1} = ^{i+1}_i R ( ^iv_i + ^iω_i × ^iP_{i+1} )

Prismatic:  ^{i+1}ω_{i+1} = ^{i+1}_i R ^iω_i
            ^{i+1}v_{i+1} = ^{i+1}_i R ( ^iv_i + ^iω_i × ^iP_{i+1} ) + ḋ_{i+1} ^{i+1}Ẑ_{i+1}
```

The end result `^Nv_N, ^Nω_N` is the tool velocity written **in the tool (body) frame**, and it is linear in Θ̇. Multiplying by `^0_N R` moves it to the base (world) frame (§3).

*→ Collecting the coefficients of Θ̇ is precisely the Jacobian.*

---

## 7. The Jacobian

The tool velocity vector `ν = [v; ω]` (6×1) is linear in the joint rates, with coefficients that depend on configuration:

```
ν = J(Θ) Θ̇          J is 6 × n
```

**Body (tool-frame) Jacobian `^N J`:** read off directly from the propagation in §6, so `^Nν = ^N J Θ̇`. Its angular rows are the body-frame ω of §2.

**World (base-frame) Jacobian `^0 J`:** unroll the addition rule (§5) and `v = ω × r` (§2) in the base frame. With `Z_i` = axis of joint i, `P_i` = a point on it (frame origin), `P_e` = tool origin, all in {0}:

```
ω = Σ θ̇_i Z_i          v = Σ θ̇_i Z_i × (P_e − P_i)          (revolute joints)
```

| Joint i | Column i, linear part | Column i, angular part |
|---|---|---|
| Revolute | `Z_i × (P_e − P_i)` | `Z_i` |
| Prismatic | `Z_i` | `0` |

**Transformation between the two:** v and ω both rotate exactly as in §3.

```
^0 J = [ ^0_N R     0     ] ^N J          ^N J = [ ^N_0 R     0     ] ^0 J
       [    0     ^0_N R  ]                      [    0     ^N_0 R  ]
```

**Matrix properties**
- `J = [J_v; J_ω]`. `J_v = ∂P_e/∂Θ`, but in general `J_ω` is *not* the derivative of any set of orientation angles (ω is not integrable to angles).
- The frame-change matrix `𝓡 = diag(R, R)` is orthogonal with `det 𝓡 = 1`. So the rank, determinant and singular values of J are the same in every frame. Only directions rotate: `J J^T → 𝓡 (J J^T) 𝓡^T`.

**Example, planar 2R arm** (link lengths l1, l2). Here ω has only a z-component, `ω_z = θ̇1 + θ̇2`, by §5:

```
^3 J = [ l1 s2          0  ]          ^0 J = ^0_3 R · ^3 J = [ −l1 s1 − l2 s12    −l2 s12 ]
       [ l1 c2 + l2     l2 ]          (^0_3 R = Rz(θ1+θ2))   [  l1 c1 + l2 c12     l2 c12 ]
```

**Angular part vs Euler-angle rates.** Because ω is not a derivative of angles, angle rates map to ω through `ω = E(φ) φ̇`. For Z-Y-X Euler angles (α, β, γ), in the world frame:

```
ω = [ 0   −sα   cα cβ ] [α̇]
    [ 0    cα   sα cβ ] [β̇]          det E = −cos β
    [ 1    0    −sβ   ] [γ̇]
```

So E is singular at β = ±90°, and the analytic Jacobian `J_a = diag(I, E⁻¹) J` blows up there. This is a *representation* singularity, distinct from the *mechanism* singularities below.

*→ Everything we do with J (inverting it, transposing it) depends on its rank.*

---

## 8. Singularities: where J loses rank

Motion control needs the velocity-level counterpart of inverse kinematics, `Θ̇ = J⁻¹ ν`, which exists only if `det J(Θ) ≠ 0`. Configurations with `det J = 0` are **singularities**:

- The columns of J (each joint's contribution, §7) become linearly dependent, so some direction of v or ω cannot be produced by any joint motion (a lost DOF), while some non-zero Θ̇ produces no tool motion at all.
- Near a singularity `‖Θ̇‖ ≈ ‖ν‖ / σ_min(J)`, so a finite tool velocity demands huge joint speeds. Manipulability `w = √det(J J^T) = σ₁σ₂⋯` measures the distance to a singularity. In practice, damped least squares is used: `Θ̇ = J^T (J J^T + λ² I)⁻¹ ν`.
- Singularity belongs to the configuration, not the frame: `det ^BJ = det ^AJ` (§7).

**Boundary singularity (2R arm).** `det J = l1 l2 sin θ2 = 0` at θ2 = 0° (stretched) or 180° (folded): the tool cannot move along the arm's own direction. In inverse kinematics the same configurations appear as the elbow-up and elbow-down solutions merging.

**Interior singularity (wrist).** The angular block `[Z4 Z5 Z6]` must span 3-space to produce any ω. When θ5 makes axes 4 and 6 collinear (θ5 = 0 in the usual convention), `ω = (θ̇4 + θ̇6) Z4 + θ̇5 Z5`: only the sum θ̇4 + θ̇6 matters. This is the addition rule of §5 applied to parallel axes, and inverse kinematics then has infinitely many (θ4, θ6) solutions.

*→ Velocity and force are power-conjugates, so the same J (and the same singularities) govern statics.*

---

## 9. Static forces: the same Jacobian, transposed

Power must agree in joint space and tool space, `τ^T Θ̇ = F^T ν = F^T J Θ̇` for every Θ̇, so

```
τ = J^T F          (F = [force; moment] at the tool, written in the same frame as J)
```

The same result comes link by link, from the tool inward:

```
^i f_i = ^i_{i+1} R ^{i+1}f_{i+1}
^i n_i = ^i_{i+1} R ^{i+1}n_{i+1} + ^iP_{i+1} × ^i f_i
τ_i = ^i n_i^T ^iẐ_i   (revolute)          τ_i = ^i f_i^T ^iẐ_i   (prismatic)
```

At a singularity `J^T` also loses rank: a tool load in the direction that cannot move is carried by the links themselves and needs no joint torque, the dual of the lost-velocity picture in §8.

---

## Quick recap

| Idea | Key equation |
|---|---|
| ω, world frame | `S(^Aω) = Ṙ R^T` |
| ω, body frame | `S(^Bω) = R^T Ṙ` |
| Frame change of ω | `^Aω = R ^Bω`, and `R S(ω) R^T = S(Rω)` |
| Adding rotations | `^AΩ_C = ^AΩ_B + ^A_B R ^BΩ_C` |
| Link propagation | `ω_{i+1} = R ω_i + θ̇_{i+1} Ẑ_{i+1}` |
| Jacobian | `ν = J Θ̇`, revolute column `[Z × (P_e − P_i); Z]` |
| Frame change of J | `^B J = diag(R, R) ^A J` |
| Singularity | `det J = 0` (rank drop) |
| Statics | `τ = J^T F` |
