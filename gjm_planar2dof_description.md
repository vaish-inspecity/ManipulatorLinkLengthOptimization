# Generalized Jacobian of a Planar 2-DOF Arm on a Free-Floating Base

Companion document to `gjm_planar2dof_defs.m`

Section numbers in this document (§x) are referenced in the MATLAB comments as "Doc §x".

---

## 1. Overview

A robotic arm mounted on a spacecraft that is **not** actively controlled (free-floating) does not behave like a ground robot. Every joint motion pushes back on the base, so the base translates and rotates, and the end effector ends up somewhere different from where the fixed-base kinematics predicts.

Because no external force or torque acts on the system, the total linear and angular momentum are conserved. This conservation law is a constraint that ties the base velocity to the joint velocities. Eliminating the base velocity through this constraint gives the **Generalized Jacobian Matrix (GJM)** $J^*$, which maps joint rates directly to end-effector velocity while accounting for base reaction:

$$
\begin{bmatrix} v_e \\ \omega_e \end{bmatrix} = J^*(\phi_0, \theta)\,\dot\theta \;+\; \dot x_{e,\text{drift}}
$$

The drift term is non-zero only when the system carries momentum from its initial state (the base was already moving at $t_0$).

The script derives, symbolically:

| Output | Size | Meaning |
|---|---|---|
| `J_fixed` | 3×2 | Fixed-base Jacobian (Craig Ch. 5) |
| `J_b`, `J_m` | 3×3, 3×2 | Floating-base kinematic Jacobians (base part, arm part) |
| `H` | 5×5 | Generalized inertia matrix of base + arm |
| `A` | 3×5 | Momentum map, $\Pi = A(q)\dot q$ |
| `J_bs` | 3×2 | Base-velocity response to joint rates, $-H_b^{-1}H_{bm}$ |
| `Jstar` | 3×2 | Generalized Jacobian $J^*$ |
| `xe_drift` | 3×1 | End-effector drift due to initial momentum |

---

## 2. System description and assumptions

The system consists of three rigid bodies moving in a plane:

- **Body 0 – base (satellite).** Three planar DOF: position $(x_0, y_0)$ of its CoM and yaw $\phi_0$.
- **Body 1 – link 1.** Joint 1 is revolute, mounted at a fixed point on the base.
- **Body 2 – link 2.** Joint 2 is revolute, and the end effector sits at its tip.

Total DOF: 5, with $q = [x_0,\ y_0,\ \phi_0,\ \theta_1,\ \theta_2]^T$.

Assumptions:

1. All bodies are rigid.
2. All joint axes are parallel to $z_{\mathcal F}$, so motion is confined to the $x$–$y$ plane. Offsets along $z$ are allowed and are carried through the frame chain, but they do not affect planar kinematics or dynamics.
3. There is no gravity and no external force or torque, and the base has no actuation. The system is **free-floating**, so momentum is conserved.
4. The base may have a non-zero initial velocity, so the conserved momentum $\Pi_0$ is not necessarily zero.
5. Link centres of mass may be offset from the link axis in both $x_i$ and $y_i$ (and $z_i$).

Relation to the reference paper (Antonello et al., 2019). The paper treats a **free-flying** base, where thrusters and VSCMGs apply $\mathcal F^{\mathcal F}_{\mathcal B}$ in Eq. (27). This work is the special case $\mathcal F^{\mathcal F}_{\mathcal B} = 0$ and $F_{ext} = 0$, in which Eq. (27) integrates once to the momentum conservation law used here.

---

## 3. Frames and conventions

### 3.1 Frames

| Frame | Origin | Axes | Motion relative to parent |
|---|---|---|---|
| $\mathcal F$ | Fixed inertial point $O$ | Inertial; $z_F$ normal to plane | — |
| $\mathcal B$ | Base CoM (paper's $\{C,\mathcal B\}$ coincides) | Body-fixed; $z_B \parallel z_F$ | Translation $(x_0,y_0)$, yaw $\phi_0$ relative to $\mathcal F$ |
| $\{0\}$ | Arm mount point | Yawed by constant $\theta_0$ from $\mathcal B$ | None (rigid in $\mathcal B$) |
| $\{1\}$ | On joint-1 axis, $d_1$ along $z_0$ | $z_1$ = joint-1 axis, $x_1$ along link 1 | Rotation $\theta_1$ about $z_1$ relative to $\{0\}$ |
| $\{2\}$ | On joint-2 axis, $l_1$ along $x_1$, $d_2$ along $z_2$ | $z_2$ = joint-2 axis, $x_2$ along link 2 | Rotation $\theta_2$ about $z_2$ relative to $\{1\}$ |
| $\{T\}$ | End effector, $l_2$ along $x_2$, $d_T$ along $z_2$ | Parallel to $\{2\}$ | None (rigid in $\{2\}$) |
| $\{C,i\}$ | CoM of link $i$ | Parallel to $\{i\}$ | None (rigid in $\{i\}$) |

### 3.2 Modified DH table (Craig Ch. 3)

The link transform is

$$
{}^{i-1}_{\ \ i}T = \text{Rot}_X(\alpha_{i-1})\,\text{Trans}_X(a_{i-1})\,\text{Rot}_Z(\theta_i)\,\text{Trans}_Z(d_i)
$$

With $\alpha = 0$ this reduces to

$$
{}^{i-1}_{\ \ i}T = \begin{bmatrix} c\theta_i & -s\theta_i & 0 & a_{i-1} \\ s\theta_i & c\theta_i & 0 & 0 \\ 0 & 0 & 1 & d_i \\ 0&0&0&1\end{bmatrix}
$$

| $i$ | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$ |
|---|---|---|---|---|
| 1 | 0 | 0 | $d_1$ | $\theta_1$ |
| 2 | 0 | $l_1$ | $d_2$ | $\theta_2$ |
| T | 0 | $l_2$ | $d_T$ | 0 |

The full chain is $\mathcal F \rightarrow \mathcal B \rightarrow \{0\} \rightarrow \{1\} \rightarrow \{2\} \rightarrow \{T\}$, with $\{C,1\}$ hanging off $\{1\}$ and $\{C,2\}$ off $\{2\}$.

### 3.3 Notation and conventions

- $P^{z}_{x/y}$ is the position of the origin of $x$ relative to the origin of $y$, expressed in $z$.
- $v^{z}_{x/y}$ and $\omega^{z}_{x/y}$ are the velocity and angular velocity of $x$ relative to $y$, expressed in $z$.
- Code `T_y_x` $= {}^{y}_{x}T$ maps coordinates from $x$ to $y$: $p^y = {}^y_xT\,p^x$. Code `R_F_x` $= {}^{\mathcal F}_xR$; its columns are the axes of $x$ written in $\mathcal F$.

Three questions must be answered for every vector:

1. **Relative to what** – two points for a position; plus the observer frame for a velocity.
2. **Derivative in which frame** – the frame in which the rate of change is observed.
3. **Expressed in which frame** – the basis of the components (only a multiplication by $R$).

**Global convention of the script:** every vector is expressed in $\mathcal F$, and every velocity is the time derivative observed in $\mathcal F$:

$$
v^{\mathcal F}_{x/\mathcal F} = \frac{{}^{\mathcal F}d}{dt}\,r_x = \frac{\partial r_x}{\partial q}\,\dot q
$$

The reference paper's Eq. (22) uses body-frame components of the base velocity. The two are related by $v^{\mathcal F}_{\mathcal B/\mathcal F} = R_{F\_B}\,v_{\mathcal B/\mathcal F}$; the paper notes the choice is arbitrary.

**Transport theorem.** For any vector $\rho$ fixed in a body rotating with $\omega_{\mathcal B/\mathcal F}$:

$$
\frac{{}^{\mathcal F}d}{dt}\rho = \frac{{}^{\mathcal B}d}{dt}\rho + \omega_{\mathcal B/\mathcal F}\times\rho
$$

This is the origin of every $\hat z\times(\cdot)$ column in the Jacobians below.

**Planar operators.** For planar vectors:

- $\hat z\times v = [-v_y,\ v_x]^T$
- $a\times v = a_xv_y - a_yv_x$ (a scalar, the $z$-component)
- $a\times(\hat z\times v) = a\cdot v$

---

## 4. Variables (code Part 1)

### 4.1 Generalized coordinates and rates (code §1)

| Code | Symbol | Meaning | Relative to | Derivative in | Expressed in | Unit |
|---|---|---|---|---|---|---|
| `x0, y0` | $P^{\mathcal F}_{\mathcal B/\mathcal F}$ | Base CoM position | Origin of $\mathcal F$ | — | $\mathcal F$ | m |
| `phi0` | $\phi_0$ | Base yaw | $\mathcal F$ | — | about $z_F$ | rad |
| `th1` | $\theta_1$ | Joint-1 angle, $x_0\to x_1$ | $\{0\}$ | — | about $z_1$ | rad |
| `th2` | $\theta_2$ | Joint-2 angle, $x_1\to x_2$ | $\{1\}$ | — | about $z_2$ | rad |
| `dx0, dy0` | $v^{\mathcal F}_{\mathcal B/\mathcal F}$ | Base CoM velocity | $\mathcal F$ | $\mathcal F$ | $\mathcal F$ | m/s |
| `dphi0` | $\omega_{\mathcal B/\mathcal F}$ | Base yaw rate | $\mathcal F$ | $\mathcal F$ | $z_F = z_B$ | rad/s |
| `dth1` | $\omega_{1/0}$ | Joint-1 rate | $\{0\}$ | $\{0\}$ | $z_1$ | rad/s |
| `dth2` | $\omega_{2/1}$ | Joint-2 rate | $\{1\}$ | $\{1\}$ | $z_2$ | rad/s |

Stacked vectors:

- `xb` $= [x_0, y_0, \phi_0]^T$ and `dxb` $= [\dot x_0, \dot y_0, \dot\phi_0]^T$. This is a mixed twist: linear velocity in $\mathcal F$ components plus a scalar yaw rate, not a body-frame twist.
- `theta`, `dth` are the joint coordinates and rates.
- `q` $= [x_b;\theta]$ and `dq` $= [\dot x_b;\dot\theta]$.

Joint angles are **relative**. The absolute angle of link 1 is $\phi_0+\theta_0+\theta_1$, and of link 2 is $\phi_0+\theta_0+\theta_1+\theta_2$.

### 4.2 Initial conditions and conserved momentum (code §2)

| Code | Meaning | Relative to | Derivative in | Expressed in | Unit |
|---|---|---|---|---|---|
| `q_i`, `dq_i` | State and rates at $t_0$ (same definitions as §4.1) | as §4.1 | as §4.1 | as §4.1 | — |
| `Px0, Py0` | Total linear momentum $\sum m_i v^{\mathcal F}_{C_i/\mathcal F}$ | $\mathcal F$ | $\mathcal F$ | $\mathcal F$ | kg·m/s |
| `Lz0` | Total angular momentum about **fixed** origin $O$ | Point $O$ | $\mathcal F$ | $z_F$ | kg·m²/s |
| `Pi0` | $[P_{x0};P_{y0};L_{z0}]$ | — | — | — | — |

`Pi0` is kept symbolic; `Pi0_eval` (code §11) gives its value for a specific initial state.

### 4.3 Points in $\mathcal F$ (code §5)

All points are positions relative to $O$, expressed in $\mathcal F$, taken from column 4 of the corresponding `T_F_x`.

| Code | Symbol | Point |
|---|---|---|
| `r0` | $P^{\mathcal F}_{\mathcal B/\mathcal F}$ | Base CoM |
| `rJ1` | $P^{\mathcal F}_{1/\mathcal F}$ | Origin of $\{1\}$ (on joint-1 axis) |
| `rJ2` | $P^{\mathcal F}_{2/\mathcal F}$ | Origin of $\{2\}$ (on joint-2 axis) |
| `re` | $P^{\mathcal F}_{T/\mathcal F}$ | End effector |
| `r1`, `r2` | $P^{\mathcal F}_{C_i/\mathcal F}$ | Link CoMs |

### 4.4 Relative vectors (code §6)

These are geometric vectors between two points, expressed in $\mathcal F$. They rotate with the bodies, so their $\mathcal F$-derivative includes $\omega\times(\cdot)$ terms.

| Code | Symbol | From → To | Body-frame form |
|---|---|---|---|
| `a` | $P^{\mathcal F}_{1/\mathcal B}$ | Base CoM → joint 1 | $R_{F\_B}\,(b + R_{B\_0}[0;0;d_1])$ |
| `p1` | $P^{\mathcal F}_{C1/1}$ | Joint 1 → link-1 CoM | $R_{F\_1}[l_{c1};d_{c1};h_{c1}]$ |
| `L1` | $P^{\mathcal F}_{2/1}$ | Joint 1 → joint 2 | $R_{F\_1}[l_1;0;d_2]$ |
| `p2` | $P^{\mathcal F}_{C2/2}$ | Joint 2 → link-2 CoM | $R_{F\_2}[l_{c2};d_{c2};h_{c2}]$ |
| `L2` | $P^{\mathcal F}_{T/2}$ | Joint 2 → end effector | $R_{F\_2}[l_2;0;d_T]$ |
| `rho1` | $P^{\mathcal F}_{C1/\mathcal B}$ | Base CoM → link-1 CoM | — |
| `rho2` | $P^{\mathcal F}_{C2/\mathcal B}$ | Base CoM → link-2 CoM | — |
| `rhoe` | $P^{\mathcal F}_{T/\mathcal B}$ | Base CoM → end effector | — |

### 4.5 Mass-weighted vectors and system CoM (code §7)

| Code | Definition | Meaning |
|---|---|---|
| `w1` | $m_1(r_1-r_{J1}) + m_2(r_2-r_{J1})$ | First mass moment of links outboard of joint 1, about joint 1 |
| `w2` | $m_2(r_2-r_{J2})$ | Same for joint 2 |
| `rhog` | $(m_1\rho_1+m_2\rho_2)/M$ | $P^{\mathcal F}_{G/\mathcal B}$: base CoM → system CoM |
| `rg` | $r_0+\rho_g$ | $P^{\mathcal F}_{G/\mathcal F}$: system CoM |

Column $j$ of the linear coupling block of $H$ equals $\hat z\times w_j$. This is why $w_1$ and $w_2$ appear.

### 4.6 Angular velocities and joint axes (code §8)

| Code | Symbol | Value | Relative to | Derivative in | Expressed in |
|---|---|---|---|---|---|
| `omega_B` | $\omega^{\mathcal F}_{\mathcal B/\mathcal F}$ | $\dot\phi_0$ | $\mathcal F$ | $\mathcal F$ | $\mathcal F$ (= $\mathcal B$) |
| `omega_1` | $\omega^{\mathcal F}_{1/\mathcal F}$ | $\dot\phi_0+\dot\theta_1$ | $\mathcal F$ | $\mathcal F$ | $\mathcal F$ |
| `omega_2` | $\omega^{\mathcal F}_{2/\mathcal F}$ | $\dot\phi_0+\dot\theta_1+\dot\theta_2$ | $\mathcal F$ | $\mathcal F$ | $\mathcal F$ |
| `z1`, `z2` | $\hat z_i$ | Joint axes | — | — | $\mathcal F$ |

Craig's recursion $\omega_{i+1} = R\,\omega_i + \dot\theta_{i+1}\hat z_{i+1}$ (paper Eq. 8) collapses to scalar addition because every axis is parallel to $z_F$. The frames $\{T\}$ and $\{C,2\}$ share $\omega_2$; $\{C,1\}$ shares $\omega_1$.

---

## 5. Parameters (code §3)

| Code | Symbol | Meaning | Defined relative to | Expressed in | Unit | Assumption |
|---|---|---|---|---|---|---|
| `m0` | $m_0$ | Base mass | — | — | kg | positive |
| `m1`, `m2` | $m_i$ | Link masses | — | — | kg | positive |
| `I0` | $I_0$ | Base $I_{zz}$ about base CoM | Base CoM | $\mathcal B$ | kg·m² | positive |
| `I1`, `I2` | $I_i$ | Link $I_{zz}$ about own CoM (zz entry of $I^C_i$) | Link CoM | $\{C,i\}$ | kg·m² | positive |
| `l1` | $a_1$ | Joint-1 axis → joint-2 axis, along $x_1$ | $\{1\}$ | $\{1\}$ | m | positive |
| `l2` | $a_2$ | Joint-2 axis → end effector, along $x_2$ | $\{2\}$ | $\{2\}$ | m | positive |
| `d1`, `d2` | $d_i$ | DH offset along joint axis $z_i$ | $\{i-1\}$ | $z_i$ | m | real |
| `dT` | $d_T$ | Tool offset along $z_2$ | $\{2\}$ | $z_2$ | m | real |
| `lc1`, `dc1`, `hc1` | $P^1_{C1/1}$ | Link-1 CoM: $x_1$, $y_1$, $z_1$ components | Origin of $\{1\}$ | $\{1\}$ | m | real |
| `lc2`, `dc2`, `hc2` | $P^2_{C2/2}$ | Link-2 CoM: $x_2$, $y_2$, $z_2$ components | Origin of $\{2\}$ | $\{2\}$ | m | real |
| `bx`, `by`, `bz` | $P^{\mathcal B}_{0/\mathcal B}$ | Arm mount point | Base CoM | $\mathcal B$ | m | real |
| `th0` | $\theta_0$ | Fixed yaw of $\{0\}$ | $\mathcal B$ | about $z_B$ | rad | real |
| `z0` | $z_0$ | Base CoM height | Origin of $\mathcal F$ | $\mathcal F$ | m | real (constant) |
| `M` | $M$ | Total mass $m_0+m_1+m_2$ | — | — | kg | derived |

The CoM offsets are defined in the link frame because there they are constants; in $\mathcal F$ they change with every joint angle. The $z$-direction offsets ($d_i$, $h_{ci}$, $b_z$, $z_0$) remain in the frame chain for completeness but cancel out of every planar quantity.

---

## 6. Derivation step by step (code Part 2)

### 6.0 Step 1 – Fixed-base Jacobian (code §12, `J_fixed`)

**Concept.** With the base clamped, the end-effector velocity depends on joint rates only. Craig's outward velocity propagation (Eqs. 5.45 and 5.47) gives

$$
{}^{i+1}\omega_{i+1} = {}^{i+1}_{\ \ i}R\,{}^i\omega_i + \dot\theta_{i+1}\hat z_{i+1}, \qquad
{}^{i+1}v_{i+1} = {}^{i+1}_{\ \ i}R\left({}^iv_i + {}^i\omega_i\times{}^iP_{i+1}\right)
$$

Starting from ${}^0\omega_0 = 0$ and ${}^0v_0 = 0$, and expressing the result in $\{0\}$ with $\theta_0=0$:

$$
J_{fixed} = \begin{bmatrix} -l_1s_1 - l_2s_{12} & -l_2s_{12} \\ l_1c_1 + l_2c_{12} & l_2c_{12} \\ 1 & 1\end{bmatrix}
$$

Here $s_{12} = \sin(\theta_1+\theta_2)$ and similarly for $c_{12}$. The kinematic singularity is at $\det = l_1l_2\sin\theta_2 = 0$.

**In code.** `J_fixed = subs(J_m, [phi0, th0], [0, 0])`. The floating-base arm Jacobian with the base frozen and aligned reduces to Craig's result.

### 6.1 Point Jacobians (code §9)

**Concept.** Every point is written as an explicit function $r(q)$ in $\mathcal F$. Its velocity observed in $\mathcal F$ is the chain rule:

$$
v^{\mathcal F}_{x/\mathcal F} = \frac{\partial r_x}{\partial q}\dot q = J_{v,x}(q)\,\dot q
$$

Column $k$ is the velocity the point would have if only $\dot q_k = 1$. By the transport theorem, the columns have a clear physical meaning:

| Column | Coordinate | Value for a point $r$ |
|---|---|---|
| 1, 2 | $x_0, y_0$ | $[1,0]^T$, $[0,1]^T$ – pure translation |
| 3 | $\phi_0$ | $\hat z\times(r - r_0)$ – rotation about base CoM |
| 4 | $\theta_1$ | $\hat z_1\times(r - r_{J1})$ if $r$ is outboard of joint 1, else 0 |
| 5 | $\theta_2$ | $\hat z_2\times(r - r_{J2})$ if $r$ is outboard of joint 2, else 0 |

The angular Jacobians are row vectors of ones and zeros: $J_{\omega,0} = [0\,0\,1\,0\,0]$, $J_{\omega,1} = [0\,0\,1\,1\,0]$, $J_{\omega,2} = [0\,0\,1\,1\,1]$.

**In code.** `jacobian(r(1:2), q)` for positions and `jacobian(omega(3), dq)` for angular velocities. Rows 1:2 keep the in-plane components.

### 6.2 Generalized inertia matrix (code §10)

**Concept.** The kinetic energy of each rigid body splits into translation of its CoM plus rotation about its CoM (Koenig's theorem; Craig Ch. 6):

$$
T = \frac12\sum_{i=0}^{2}\left(m_i\,|v_{C_i}|^2 + I_i\,\omega_i^2\right) = \frac12\,\dot q^T H(q)\,\dot q
$$

Substituting $v_{C_i} = J_{v,i}\dot q$ and $\omega_i = J_{\omega,i}\dot q$ gives

$$
H(q) = \sum_{i=0}^{2}\left(m_i\,J_{v,i}^TJ_{v,i} + I_i\,J_{\omega,i}^TJ_{\omega,i}\right)
$$

**Block partition.** This corresponds to paper Eq. (27): $H_b \leftrightarrow M_b$, $H_{bm}\leftrightarrow M_{mb}^T$, $H_m\leftrightarrow M$.

$$
H = \begin{bmatrix} H_b & H_{bm} \\ H_{bm}^T & H_m \end{bmatrix},\qquad
H_b = \begin{bmatrix} M\,\mathbb 1_2 & c_b \\ c_b^T & I_w\end{bmatrix},\qquad
H_{bm} = \begin{bmatrix} J_{TW} \\ h_3\end{bmatrix}
$$

| Code | Closed form | Meaning |
|---|---|---|
| `c_b` | $M\,(\hat z\times\rho_g)$ | Linear/angular coupling of base: rotating the base about its CoM moves the system CoM |
| `Iw` | $\sum I_i + \sum m_i\lvert\rho_i\rvert^2$ | Inertia of the whole system about the base CoM |
| `J_TW` | $[\hat z\times w_1,\ \hat z\times w_2]$ | Linear momentum generated by unit joint rates |
| `h3` | $h_{3j} = \sum_{i\ge j}\left(I_i + m_i\,\rho_i\cdot(r_i - r_{Jj})\right)$ | Angular momentum about base CoM generated by unit joint rates |
| `H_m` | — | Fixed-base arm inertia matrix $M(\theta)$ |

$H$ is symmetric and positive definite. Its entries depend on $\phi_0$ and $\theta$ only (not on $x_0, y_0$), which reflects translational invariance.

### 6.3 Momentum and the conservation law (code §11)

**Concept.** The generalized momenta of the base coordinates are

$$
\frac{\partial T}{\partial \dot x_b} = H_{(1:3,:)}\,\dot q
$$

- Rows 1–2 are the total linear momentum $P = \sum m_i v_{C_i}$, observed in and expressed in $\mathcal F$.
- Row 3 is the total angular momentum about the **moving** base CoM $r_0$, computed with absolute velocities: $h_{/r_0} = \sum I_i\omega_i + \sum m_i\,\rho_i\times v_{C_i}$.

**Why the reference point matters.** Angular momentum depends on the reference point:

$$
h_{/O} = h_{/r_0} + r_0\times P
$$

With no external wrench, $h$ is conserved only about a fixed point $O$ or about the system CoM $G$. If the base starts moving ($P\neq0$), then $h_{/r_0}$ is **not** constant. The code therefore defines

$$
L_O = h_{/r_0} + r_0\times P, \qquad L_G = L_O - r_g\times P
$$

**Momentum map.** Stacking the conserved quantities:

$$
\Pi(q,\dot q) = \begin{bmatrix}P\\L_O\end{bmatrix} = A(q)\,\dot q,\qquad A = \begin{bmatrix}A_b & A_{bm}\end{bmatrix}
$$

$A = E\,H_{(1:3,:)}$ with $E = \begin{bmatrix}1&0&0\\0&1&0\\-y_0&x_0&1\end{bmatrix}$. Since $E$ is invertible, $A_b^{-1}A_{bm} = H_b^{-1}H_{bm}$, so the GJM does not depend on $x_0$ or $y_0$.

**Conservation law** – the constraint that defines free-floating motion:

$$
\boxed{A_b(q)\,\dot x_b + A_{bm}(q)\,\dot\theta = \Pi_0}
$$

`Pi0_eval` evaluates $\Pi_0 = A(q_i)\dot q_i$ from the initial state. Because momentum is conserved, only the initial state is needed.

### 6.4 End-effector kinematic Jacobian (code §12)

**Concept.** Kinematics alone, valid for any base motion (free-flying or free-floating), splits the end-effector twist into a base part and an arm part:

$$
\begin{bmatrix}v_e\\\omega_e\end{bmatrix} = J_b\,\dot x_b + J_m\,\dot\theta
$$

The base Jacobian is the rigid-body transport of the base twist to the end effector:

$$
J_b = \begin{bmatrix}1&0&-(y_e-y_0)\\0&1&x_e-x_0\\0&0&1\end{bmatrix}
$$

This is the planar form of paper Eq. (22).

The manipulator Jacobian $J_m$ is the fixed-base Jacobian rotated into $\mathcal F$ by the base (and mount) yaw. Its columns are $\hat z_j\times(r_e - r_{Jj})$ with angular row $[1\ 1]$.

**In code.** `Je = [Jv_e; Jw_2]`, `J_b = Je(:,1:3)`, `J_m = Je(:,4:5)`.

### 6.5 Base velocity from momentum – Schur complement (code §13)

**Concept.** The conservation law is solved for $\dot x_b$. Rather than inverting $A_b$ symbolically, the block structure of $H_b$ is used.

**Linear rows:**

$$
M\dot r_0 + c_b\dot\phi_0 + J_{TW}\dot\theta = P_0 \;\Rightarrow\; \dot r_0 = \frac{1}{M}\left(P_0 - c_b\dot\phi_0 - J_{TW}\dot\theta\right)
$$

**Angular row.** Substituting into the angular row, written about $G$, eliminates $\dot r_0$:

$$
\tilde I\,\dot\phi_0 + \hat h\,\dot\theta = L_{G0}
$$

The quantities in this equation are:

| Code | Expression | Meaning |
|---|---|---|
| `Itil` | $\tilde I = I_w - c_b^Tc_b/M = I_w - M\lvert\rho_g\rvert^2$ | Inertia of the whole system about its own CoM (parallel-axis theorem); always > 0 |
| `hhat` | $\hat h = h_3 - c_b^TJ_{TW}/M$ | Angular momentum about $G$ produced by unit joint rates |
| `L_G0` | $L_{G0} = L_{z0} - r_g\times P_0$ | Conserved angular momentum about $G$ |

$L_{G0}$ is constant in time: $\dot r_g = P_0/M$ is parallel to $P_0$, so $\frac{d}{dt}(r_g\times P_0) = 0$. It can therefore be evaluated at the current configuration.

**Result.** The base velocity splits into a joint-driven part and a momentum-driven part:

$$
\dot\phi_0 = \frac{L_{G0} - \hat h\,\dot\theta}{\tilde I},\qquad
\dot x_b = J_{bs}\,\dot\theta + \dot x_{b,\text{drift}}
$$

- `J_bs` $= \partial\dot x_b/\partial\dot\theta = -H_b^{-1}H_{bm}$ is the base reaction to joint motion.
- `dxb_drift` $= \dot x_b|_{\dot\theta=0}$ is the base motion carried by the initial momentum.

**Physical reading.** When $\Pi_0 = 0$, the base yaws by $-\tilde I^{-1}\hat h\,\dot\theta$ and the system CoM stays fixed. When $\Pi_0\neq 0$, the system CoM drifts at $P_0/M$ and the system spins at $L_{G0}/\tilde I$ on top of the joint-induced reaction.

### 6.6 Generalized Jacobian (code §14)

Substituting §6.5 into §6.4:

$$
\boxed{
\begin{bmatrix}v_e\\\omega_e\end{bmatrix} = \underbrace{\left(J_m + J_b\,J_{bs}\right)}_{J^* = J_m - J_bH_b^{-1}H_{bm}}\dot\theta \;+\; \underbrace{J_b\,\dot x_{b,\text{drift}}}_{\dot x_{e,\text{drift}}}}
$$

**Explicit form.** With $\Delta = r_e - r_g$ (system CoM → end effector) and $w_j$ from §4.5:

$$
J^* = \begin{bmatrix}
J_{mv} - \dfrac1M\,[\hat z\times w_1,\ \hat z\times w_2] - (\hat z\times\Delta)\,\tilde I^{-1}\hat h\\[2mm]
[1\ \ 1] - \tilde I^{-1}\hat h
\end{bmatrix}
$$

**Properties:**

1. $J^*$ depends on $\phi_0$, $\theta$ and the mass properties – **not** on $x_0, y_0$. The dependence on $\phi_0$ is a pure rotation of the linear rows, so the angular row depends on $\theta$ only.
2. $J^*$ is a function of **dynamics** (masses, inertias, CoM offsets) as well as geometry – unlike a fixed-base Jacobian.
3. **Dynamic singularities** occur where the 2×2 linear block of $J^*$ loses rank. They are configuration- and mass-dependent and differ from the kinematic singularity $\sin\theta_2 = 0$.
4. **Fixed-base limit.** As $m_0, I_0\to\infty$: $J_{TW}/M\to0$ and $\tilde I^{-1}\hat h\to0$, so $J^*\to J_m$.
5. The angular momentum constraint is non-holonomic: the base attitude after a joint maneuver depends on the path, not only on the final joint angles.

---

## 7. Verification checks (code §15)

Each check should print zeros (symbolic) or values near machine precision (numeric, random parameters).

| Check | Variable | Tests |
|---|---|---|
| (a) | `chk_Jb` | `J_b` from `jacobian` equals the rigid-transport form of §6.4 |
| (b) | `chk_cb` | $c_b = M(\hat z\times\rho_g)$ – closed form of the inertia block |
| (c) | `err_J`, `err_drift` | Schur solution (§6.5) equals direct inversion $J_m - J_bA_b^{-1}A_{bm}$ and $J_bA_b^{-1}\Pi_0$ |
| (d) | `err_Pi` | Substituting `dxb_sol` into $\Pi(q,\dot q)$ returns $\Pi_0$ – momentum is truly conserved |
| (e) | `err_heavy` | With $m_0 = I_0 = 10^9$, $J^*\approx J_m$ – fixed-base limit |

The numeric checks substitute random values in $[0.5, 1.5]$ for every symbol. Run the script several times to test different configurations.

---

## 8. Outputs and usage

### 8.1 Main outputs

| Variable | Size | Depends on | Use |
|---|---|---|---|
| `J_fixed` | 3×2 | $\theta$ | Fixed-base reference |
| `J_b`, `J_m` | 3×3, 3×2 | $q$ | Free-flying (actuated base) kinematics |
| `H` | 5×5 | $\phi_0,\theta$ | Dynamics, $H\ddot q + C = \tau$ |
| `A`, `Pi0_eval` | 3×5, 3×1 | $q$ / $(q_i,\dot q_i)$ | Momentum bookkeeping |
| `J_bs`, `dxb_drift` | 3×2, 3×1 | $\phi_0,\theta,\Pi_0$ | Base motion prediction |
| `Jstar` | 3×2 | $\phi_0,\theta$ | Resolved-rate control of the free-floating arm |
| `xe_drift` | 3×1 | $q,\Pi_0$ | Feed-forward drift compensation |

### 8.2 Numeric evaluation

```matlab
% Substitute parameter values, then generate fast numeric functions
params = [m0 m1 m2 I0 I1 I2 l1 l2 lc1 dc1 lc2 dc2 bx by th0];
vals   = [ ... ];                                       % testbed values
Jstar_n = subs(Jstar, params, vals);
fJstar  = matlabFunction(Jstar_n, 'Vars', {phi0, th1, th2});
fDrift  = matlabFunction(subs(xe_drift, params, vals), ...
                         'Vars', {x0, y0, phi0, th1, th2, Px0, Py0, Lz0});
```

### 8.3 Resolved-rate control with drift compensation

To command a desired end-effector velocity $\dot x_{e,d}$ (planar position rows):

$$
\dot\theta = (J^*_v)^{-1}\left(\dot x_{e,d} - \dot x_{e,\text{drift},v}\right)
$$

Here $J^*_v$ is the 2×2 linear block. Use a damped pseudoinverse near dynamic singularities. The base twist then follows from $\dot x_b = J_{bs}\dot\theta + \dot x_{b,\text{drift}}$ and is integrated alongside $\theta$.

### 8.4 Performance notes

`simplify` on `H`, `Jstar` and `xe_drift` can be slow because of the offset terms. If it is too slow, remove `simplify` from §13–§14 and rely on the numeric checks, or substitute numeric parameters before simplifying.

---

## 9. Limitations and extensions

- **Planar only.** A spatial arm requires 3×3 inertia tensors, the full 6×6 base block and quaternion attitude (paper Eqs. 6–7). The structure $J^* = J_m - J_bH_b^{-1}H_{bm}$ carries over unchanged.
- **Momentum wheels / VSCMGs.** Stored momentum can be added to $\Pi$ as an extra term (the paper's $h_{med}$ in Eq. 5). It enters exactly like $\Pi_0$ but varies with wheel speed.
- **Thrusters (free-flying).** With a base force $f_{\mathcal B}$ and torque $t_{\mathcal B}$ applied, $\Pi$ is no longer conserved: $\dot\Pi = [f_{\mathcal B};\ t_{\mathcal B} + r_0\times f_{\mathcal B}]$. The GJM is then replaced by the full dynamics of paper Eq. (30).
- **Contact.** Any end-effector wrench $F_{ext}$ breaks conservation. That case requires the full dynamic model (paper Eq. 27 with $J^TF_{ext}$).

---

## 10. References

1. A. Antonello, A. Valverde, P. Tsiotras, "Dynamics and Control of Spacecraft Manipulators with Thrusters and Momentum Exchange Devices," *Journal of Guidance, Control, and Dynamics*, 42(1), 2019, pp. 15–29. doi:10.2514/1.G003601
2. J. J. Craig, *Introduction to Robotics: Mechanics and Control*, Ch. 3 (manipulator kinematics), Ch. 5 (Jacobians: velocities and static forces), Ch. 6 (manipulator dynamics).
3. Y. Umetani, K. Yoshida, "Resolved Motion Rate Control of Space Manipulators with Generalized Jacobian Matrix," *IEEE Transactions on Robotics and Automation*, 5(3), 1989, pp. 303–314.
4. S. Dubowsky, E. Papadopoulos, "The Kinematics, Dynamics, and Control of Free-Flying and Free-Floating Space Robotic Systems," *IEEE Transactions on Robotics and Automation*, 9(5), 1993, pp. 531–543.
