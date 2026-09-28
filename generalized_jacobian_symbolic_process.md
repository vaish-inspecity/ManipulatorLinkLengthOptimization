# The Generalized Jacobian of a Free-Floating Space Manipulator

**A theoretical guide to its symbolic formulation through recursive Newton–Euler kinematics and momentum conservation**

**References**

- **[AVT]** A. Antonello, A. Valverde, P. Tsiotras, *Dynamics and Control of Spacecraft Manipulators with Thrusters and Momentum Exchange Devices*, Journal of Guidance, Control, and Dynamics 42(1), 2019. DOI 10.2514/1.G003601.
- **[Craig]** J. J. Craig, *Introduction to Robotics: Mechanics and Control*:
  - Ch. 3 — frames and the Denavit–Hartenberg convention
  - Ch. 5 — velocity propagation, Jacobians, change of Jacobian frame, static forces
  - Ch. 6 — iterative Newton–Euler dynamics and the structure of the closed-form equations

Equations tagged "AVT (n)" refer to the paper's numbering. Craig is referenced by chapter.

---

## 1. The system and its assumptions

A free-floating space manipulator consists of a rigid base $\mathcal B$ with an $N$-link serial arm mounted on it. The base carries no active actuation: thrusters are off and momentum exchange devices are inactive.

This guide assumes the following:

1. **No external wrench.** No thruster force, no gravity gradient, no contact force at the end effector: $\mathcal F^{\mathcal F}_{\mathcal B}=0$ and $F_{ext}=0$ in AVT (27) and (30).
2. **Rigid bodies, revolute joints.**
3. **Zero initial momentum.** $\mathbf P_0=\mathbf 0$ and $\mathbf L_0=\mathbf 0$. The non-zero case is treated in §6.4.

Under these assumptions the base moves only as a *reaction* to the joint motion. That reaction is fixed entirely by momentum conservation. The **generalized Jacobian** $J^*$ maps joint rates to the end-effector twist with the base reaction already embedded:

$$\begin{bmatrix}v_E\\ \omega_E\end{bmatrix}=J^*(\theta;\ \text{mass properties})\ \dot\theta$$

Unlike the fixed-base Jacobian, $J^*$ depends on the masses and inertias of every body, not only on the geometry.

---

## 2. Frames and kinematic parameters

| Symbol | Definition |
|---|---|
| $\mathcal F$ | Inertial frame |
| $\mathcal B$ | Base body frame, origin at the base centre of mass |
| $\{0\}$ | Manipulator mount frame, rigidly attached to $\mathcal B$; position $P_{0/\mathcal B}$, orientation $R^{\mathcal B}_0$ |
| $\{i\}$ | Frame of link $i$ (Denavit–Hartenberg), $i=1..N$ |
| $\{C,i\}$ | Frame at the centre of mass of link $i$, axes parallel to $\{i\}$ |
| $P_{i/i-1}$ | Origin of $\{i\}$ relative to $\{i-1\}$ |
| $P_{\{C,i\}/i}$ | Centre of mass of link $i$ relative to the origin of $\{i\}$ |
| $m_i,\ I^C_i$ | Mass and centroidal inertia tensor of body $i$ ($i=0$ or $\mathcal B$ denotes the base) |

**Link transform.** Each link is described by one DH row:
$${}^{i-1}T_{i}=\begin{bmatrix}R^{i-1}_i&P_{i/i-1}\\0&1\end{bmatrix}=\operatorname{Rot}_z(\theta_i)\operatorname{Trans}_z(d_i)\operatorname{Trans}_x(a_i)\operatorname{Rot}_x(\alpha_i)$$

This is the standard convention: joint $i$ rotates about $\hat z_{i-1}$. [AVT] uses it, with the index shifted so that $\theta_{i+1}$ acts about $\hat z_{i+1}$. [Craig] uses the modified convention, in which $\hat z_i$ is the axis of joint $i$. The two conventions differ only in *where* the rotation matrices and offsets appear in the recursion below; the resulting velocities are identical.

**Mount frame.** Because $\{0\}$ is rigidly fixed to the base, the mount joint has no motion ($\dot\theta_0=\ddot\theta_0=0$). This is the step that turns AVT (20)–(21) into AVT (23)–(24).

---

## 3. Outward velocity propagation

### 3.1 Moving-base initial conditions

For a fixed-base arm, [Craig] Ch. 5 starts the recursion with $\omega_0=v_0=0$. On a floating base, the mount frame inherits the base twist. At velocity level (AVT 20–22):
$$\omega_{0}=R^{0}_{\mathcal B}\,\omega_{\mathcal B},\qquad v_{0}=R^{0}_{\mathcal B}\big(v_{\mathcal B}+\omega_{\mathcal B}\times P^{\mathcal B}_{0/\mathcal B}\big)$$
Here $v_{\mathcal B}$ and $\omega_{\mathcal B}$ are the inertial linear velocity of the base centre of mass and the inertial angular velocity of the base, both expressed in $\mathcal B$.

### 3.2 Link-to-link recursion (local frames)

For $i=0,\dots,N-1$ (AVT 8 and the velocity-level form of AVT 10; [Craig] Ch. 5):
$$\omega_{i+1}=R^{i+1}_{i}\,\omega_i+\dot\theta_{i+1}\,\hat z_{i+1}$$
$$v_{i+1}=R^{i+1}_{i}\big(v_i+\omega_i\times P_{i+1/i}\big)$$
$$v_{\{C,i+1\}}=v_{i+1}+\omega_{i+1}\times P_{\{C,i+1\}/i+1}$$

The end-effector point $E$, fixed in link $N$, has:
$$v_E=v_N+\omega_N\times P_{E/N},\qquad \omega_E=\omega_N$$

### 3.3 Equivalent recursion in the base frame

For symbolic work it is often more convenient to express every vector directly in $\mathcal B$. Let $p_i$ be the origin of $\{i\}$, $r_i$ the centre of mass of body $i$, and $\hat z_{i-1}$ the axis of joint $i$, all in $\mathcal B$ coordinates. Then:
$$\omega_i=\omega_{i-1}+\dot\theta_i\,\hat z_{i-1}$$
$$\dot r_i=\dot p_{i-1}+\omega_i\times(r_i-p_{i-1})$$
$$\dot p_i=\dot p_{i-1}+\omega_i\times(p_i-p_{i-1})$$

The recursion starts from $\omega_0=\omega_{\mathcal B}$ and $\dot p_0=v_{\mathcal B}+\omega_{\mathcal B}\times P_{0/\mathcal B}$.

The two forms are related by the rotations ${}^{\mathcal B}R_i=R^{\mathcal B}_0R^0_1\cdots R^{i-1}_i$. The base-frame form avoids repeated rotation of intermediate results, which keeps symbolic expressions compact.

### 3.4 Acceleration level (for completeness)

Differentiating the velocity recursion gives the outward acceleration sweep, AVT (9)–(10). Combined with the Newton and Euler equations (AVT 11–12) and the inward force sweep (AVT 16–18, 25–26), this is the full recursive Newton–Euler algorithm of [AVT] §IV–V and [Craig] Ch. 6.

The generalized Jacobian needs only the velocity level. The acceleration level enters through the equivalence discussed in §5.3.

---

## 4. Linearity and the Jacobian blocks

Collect all independent rates in the vector
$$\dot{\mathbf x}=\begin{bmatrix}\dot{\mathbf x}_b\\ \dot\theta\end{bmatrix},\qquad \dot{\mathbf x}_b=\begin{bmatrix}v_{\mathcal B}\\ \omega_{\mathcal B}\end{bmatrix}$$

Every quantity produced by §3 is a **linear** function of $\dot{\mathbf x}$, with coefficients that depend on $\theta$ only (when expressed in $\mathcal B$). The Jacobians are therefore exact partial derivatives:
$$J_{v,i}=\frac{\partial\dot r_i}{\partial\dot{\mathbf x}},\qquad J_{\omega,i}=\frac{\partial\omega_i}{\partial\dot{\mathbf x}},\qquad J_E=\frac{\partial}{\partial\dot{\mathbf x}}\begin{bmatrix}v_E\\ \omega_E\end{bmatrix}=\big[\,J_b\ \ \ J_m\,\big]$$

- $J_m$ is the conventional fixed-base Jacobian of [Craig] Ch. 5, written in the base frame.
- $J_b$ maps the base twist to the end-effector twist. For a point rigidly attached to the base it would be $\begin{bmatrix}\mathbb 1&-[\rho_E^{\times}]\\0&\mathbb 1\end{bmatrix}$, with $\rho_E=p_E-r_{\mathcal B}$.

These are the matrices $J$ and $J_b$ appearing in AVT (27):
$$\begin{bmatrix}v_E\\ \omega_E\end{bmatrix}=J_b\,\dot{\mathbf x}_b+J_m\,\dot\theta$$

---

## 5. System momentum

### 5.1 Linear and angular momentum

With every vector in $\mathcal B$, and body $0$ denoting the base:
$$\mathbf P=\sum_{i=0}^{N}m_i\,\dot r_i$$
$$\mathbf L=\sum_{i=0}^{N}\Big[\,{}^{\mathcal B}I_i\,\omega_i+m_i\,\rho_i\times\dot r_i\,\Big],\qquad {}^{\mathcal B}I_i={}^{\mathcal B}R_i\,I^C_i\,{}^{\mathcal B}R_i^{T}$$

Here $\rho_i$ is the position of body $i$'s centre of mass relative to a reference point.

**Choice of reference point.** When $\mathbf P=0$, the angular momentum has the same value about every point. The reference point can therefore be chosen freely, and the base centre of mass ($\rho_i=r_i$) or the system centre of mass are the natural choices.

### 5.2 The momentum map

Substituting the Jacobians of §4:
$$\begin{bmatrix}\mathbf P\\ \mathbf L\end{bmatrix}=H_b\,\dot{\mathbf x}_b+H_{bm}\,\dot\theta$$

$$H_b=\sum_{i=0}^{N}\begin{bmatrix}m_iJ^{b}_{v,i}\\ {}^{\mathcal B}I_iJ^{b}_{\omega,i}+m_i[\rho_i^{\times}]J^{b}_{v,i}\end{bmatrix},\qquad H_{bm}=\sum_{i=1}^{N}\begin{bmatrix}m_iJ^{m}_{v,i}\\ {}^{\mathcal B}I_iJ^{m}_{\omega,i}+m_i[\rho_i^{\times}]J^{m}_{v,i}\end{bmatrix}$$

The superscripts $b$ and $m$ select the base and joint columns of each body Jacobian.

The translational block of $H_b$ is always $M\,\mathbb 1_3$, where $M=\sum_i m_i$. This is the structural fact that makes the elimination in §6 inexpensive.

### 5.3 Relation to the generalized inertia matrix

The kinetic energy of the system is
$$T=\tfrac12\sum_i\big(m_i\dot r_i^T\dot r_i+\omega_i^T\,{}^{\mathcal B}I_i\,\omega_i\big)=\tfrac12\,\dot{\mathbf x}^T\tilde M\,\dot{\mathbf x}$$
where $\tilde M$ is the generalized inertia matrix of AVT (27) and (30). The generalized momenta are $\partial T/\partial\dot{\mathbf x}=\tilde M\dot{\mathbf x}$.

With $\mathcal B$'s origin at the base centre of mass, their base components are exactly the linear momentum and the angular momentum about the base centre of mass. Therefore:
$$\big[\,H_b\ \ H_{bm}\,\big]=\text{base rows of }\tilde M=\big[\,M_b\ \ M_{mb}^T\,\big]$$

This connects the momentum map to the Newton–Euler recursion. [AVT] evaluates $\tilde M$ with the Walker–Orin procedure: the recursive Newton–Euler algorithm is run with zero velocities and a unit generalized acceleration $\ddot{\mathbf x}=\mathbf e_j$.

- The base wrench it returns is column $j$ of $[H_b\ H_{bm}]$.
- The joint torques it returns are column $j$ of $[M_{mb}^T\ M]$.

The momentum map is therefore the part of the Newton–Euler inertia computation that belongs to the base. The direct summation of §5.2 and the unit-acceleration Newton–Euler evaluation are two routes to the same symbolic matrices.

---

## 6. Momentum conservation and elimination of the base

### 6.1 Conservation

Assumption 1 gives $\dot{\mathbf P}=\dot{\mathbf L}=0$ in the inertial frame. With zero initial momentum, the components in $\mathcal B$ also vanish identically:
$$H_b\,\dot{\mathbf x}_b+H_{bm}\,\dot\theta=\mathbf 0$$

The linear-momentum rows integrate to a fixed system centre of mass. The angular-momentum rows are **non-integrable**: they are a nonholonomic constraint, which is why the final base attitude depends on the joint *path* and not only on the final configuration.

### 6.2 Block elimination

Partition the conservation equation into translational (v), rotational (ω) and joint (q) blocks:
$$\begin{bmatrix}M\mathbb 1&H_{v\omega}\\ H_{\omega v}&H_{\omega\omega}\end{bmatrix}\begin{bmatrix}v_{\mathcal B}\\ \omega_{\mathcal B}\end{bmatrix}+\begin{bmatrix}H_{vq}\\ H_{\omega q}\end{bmatrix}\dot\theta=\mathbf 0$$

**Linear rows.** These solve trivially for the base translation:
$$v_{\mathcal B}=-\frac{1}{M}\big(H_{v\omega}\,\omega_{\mathcal B}+H_{vq}\,\dot\theta\big)$$

**Angular rows.** Substituting $v_{\mathcal B}$ leaves a reduced (Schur-complement) system:
$$\hat H_{\omega}\,\omega_{\mathcal B}+\hat H_{q}\,\dot\theta=\mathbf 0$$
$$\hat H_{\omega}=H_{\omega\omega}-\frac{1}{M}H_{\omega v}H_{v\omega},\qquad \hat H_{q}=H_{\omega q}-\frac{1}{M}H_{\omega v}H_{vq}$$

$\hat H_\omega$ is the inertia of the whole system about its centre of mass, with the joints locked. It is symmetric positive definite, so it is always invertible.

**Base reaction map.** The base twist follows linearly from the joint rates:
$$\omega_{\mathcal B}=K_\omega\dot\theta,\quad K_\omega=-\hat H_\omega^{-1}\hat H_q;\qquad v_{\mathcal B}=K_v\dot\theta,\quad K_v=-\frac1M\big(H_{v\omega}K_\omega+H_{vq}\big)$$
$$\dot{\mathbf x}_b=K\,\dot\theta,\qquad K=\begin{bmatrix}K_v\\ K_\omega\end{bmatrix}=-H_b^{-1}H_{bm}$$

Only a $3\times3$ matrix is inverted in the spatial case, and only a scalar in the planar case.

### 6.3 The generalized Jacobian

Substituting the base reaction map into the end-effector twist:
$$\boxed{\ J^*=J_m+J_b\,K=J_m-J_b\,H_b^{-1}H_{bm}\ }$$

This is the fixed-base Jacobian corrected by the kinematic effect of the base reaction.

### 6.4 Non-zero initial momentum

With $\mathbf P_0\neq0$, the system centre of mass drifts at the constant velocity $v_G=\mathbf P_0/M$. The frame translating with it is inertial. Relative to that frame the linear momentum is zero, and the angular momentum about the centre of mass, $\mathbf L_{G0}$, is conserved.

The base reaction then acquires a constant offset:
$$\dot{\mathbf x}_b=K\dot\theta+H_b^{-1}\begin{bmatrix}0\\ R^{\mathcal B}_{\mathcal F}\mathbf L_{G0}\end{bmatrix}$$

The end-effector twist gains an affine drift term:
$$\begin{bmatrix}v_E\\ \omega_E\end{bmatrix}=J^*\dot\theta+J_bH_b^{-1}\begin{bmatrix}0\\ R^{\mathcal B}_{\mathcal F}\mathbf L_{G0}\end{bmatrix}+\begin{bmatrix}R^{\mathcal B}_{\mathcal F}\,v_G\\ 0\end{bmatrix}$$

$J^*$ itself is unchanged.

---

## 7. Frames of expression

**Base-frame form.** Expressed in $\mathcal B$, the matrices $H_b$, $H_{bm}$, $J_b$, $J_m$ and $J^*$ depend on $\theta$ alone. Rigid rotation of the whole system changes neither the internal kinematics nor the mass distribution, so the base attitude cannot appear.

**Inertial form.** Inertial components follow from the Jacobian frame change of [Craig] Ch. 5:
$${}^{\mathcal F}J^*=\begin{bmatrix}R^{\mathcal F}_{\mathcal B}&0\\0&R^{\mathcal F}_{\mathcal B}\end{bmatrix}{}^{\mathcal B}J^*$$
Here $R^{\mathcal F}_{\mathcal B}$ is obtained from the attitude quaternion $q_{\mathcal B/\mathcal F}$ (AVT 6–7).

**Mixing frames.** Deriving in $\mathcal F$ from the start makes every entry depend on the base attitude and inflates the expressions without adding information.

---

## 8. Closed form for a planar chain: the virtual manipulator

For a planar system moving in the plane normal to $\hat z$, every angular velocity is a scalar, and the elimination of §6 can be carried out by hand.

### 8.1 Notation

Index the bodies $j=0..N$, with $0$ the base. Define:

- the absolute angles $\phi_0=\theta_0$ and $\phi_j=\theta_0+q_1+\dots+q_j$;
- the unit vectors $\mathbf e_j=[\cos\phi_j,\ \sin\phi_j]^T$ and $\mathbf e^\perp_j=[-\sin\phi_j,\ \cos\phi_j]^T$;
- $L_j$, the joint-to-joint length, with $L_0=b_0$ the mount offset;
- $a_j$, the joint-to-centre-of-mass distance, with $a_0=0$.

### 8.2 Barycentric lengths

Linear momentum conservation fixes the system centre of mass $\mathbf r_G$. Every position relative to it becomes a chain of link directions with **constant, mass-weighted** coefficients:
$$\mathbf r_E-\mathbf r_G=\sum_{j=0}^{N}c_j\,\mathbf e_j,\qquad c_j=\frac{L_j\,(m_0+\dots+m_j)-m_ja_j}{M}$$

The $c_j$ are the link lengths of the *virtual manipulator*: a fixed-base arm, anchored at the system centre of mass, that has the same end-effector motion as the real one.

The first moments of the chain are:
$$k_j=m_ja_j+L_j\,(m_{j+1}+\dots+m_N)$$

### 8.3 Inertia coupling

Kinetic energy and angular momentum about $\mathbf r_G$ are governed by one symmetric matrix:
$$W_{jl}=c_j\,k_l\ \ (j<l),\qquad W_{jj}=c_jk_j-m_ja_j(L_j-a_j)$$

From $W$, build the configuration-dependent inertia matrix:
$$G_{lk}(q)=I_l\,\delta_{lk}+W_{lk}\cos(\phi_k-\phi_l)$$
Every angle difference $\phi_k-\phi_l$ is a partial sum of the joint angles, so the base attitude drops out.

### 8.4 Angular momentum weights

The angular momentum about the system centre of mass is a weighted sum of the body rates:
$$L=\sum_k h_k\,\omega_k,\qquad h_k=\sum_l G_{lk}$$

Define the total inertia and its tail sums:
$$H_{tot}=\sum_{k=0}^{N}h_k,\qquad H_j=\sum_{k\ge j}h_k$$

$H_{tot}$ is the scalar $\hat H_\omega$ of §6.2. Setting $L=0$ gives the base attitude rate:
$$\dot\theta_0=-\sum_{j=1}^{N}\frac{H_j}{H_{tot}}\,\dot q_j$$

### 8.5 Generalized Jacobian

Column $j$ of $J^*$ is:
$$J^*_{j}=\begin{bmatrix}\displaystyle\sum_{i\ge j}c_i\,\mathbf e^\perp_i-\frac{H_j}{H_{tot}}\sum_{i=0}^{N}c_i\,\mathbf e^\perp_i\\[2mm] 1-\dfrac{H_j}{H_{tot}}\end{bmatrix}$$

**Reading the structure.**

- The first sum is the Jacobian of the virtual manipulator.
- The second term is the base-rotation reaction, weighted by the fraction $H_j/H_{tot}$ of the system's angular momentum that joint $j$ controls.

---

## 9. Properties of the generalized Jacobian

**Fixed-base limit.** As $m_{\mathcal B}\to\infty$ and $I_{\mathcal B}\to\infty$, the virtual lengths tend to the real ones ($c_j\to L_j$), the momentum fractions vanish ($H_j/H_{tot}\to0$), and $J^*\to J_m$.

**Dynamic singularities.** $J^*$ loses rank on configurations that depend on the mass properties, not only on the geometry. These dynamic singularities are path-dependent in workspace terms, because the base attitude is itself path-dependent (§6.1).

**Resolved-rate inversion.** When $J^*$ is square and non-singular, joint rates that produce a desired end-effector twist follow directly:
$$\dot\theta=J^{*-1}\begin{bmatrix}v_E\\ \omega_E\end{bmatrix}_d$$
The redundant case uses a pseudoinverse. The base reaction is then obtained from $\dot{\mathbf x}_b=K\dot\theta$.

**Link to the full dynamics.** In the coupled equations AVT (27) and (30), the base rates are retained as coordinates and $J_b$, $J_m$ appear separately. $J^*$ is the kinematic projection of that model onto the momentum-conserving subspace.

**Workspace notions.** Because of §6.1, the reachable set splits into two concepts:

- the *path-independent workspace*: points reachable regardless of the joint path taken;
- the *path-dependent workspace*: points reachable only along particular joint paths.

---

## 10. MATLAB symbolic implementation

The script below implements §3 (base-frame recursion), §4 (Jacobian blocks by differentiation), §5 (momentum map), §6 (block elimination) and §8 (planar closed form). It then compares the recursive and closed-form $J^*$ numerically and exports evaluation functions.

**Scope.**

- The `planar` flag selects either planar motion ($\dot{\mathbf x}_b=[v_x,v_y,\omega_z]$) or full spatial motion ($\dot{\mathbf x}_b=[v_{\mathcal B};\omega_{\mathcal B}]$).
- Spatial arms require editing the DH table (`alpha`, `d`) and the inertia tensors.

**Requirements.** Symbolic Math Toolbox. `matlabFunction` performs common-subexpression optimization when exporting.

```matlab
%% Symbolic Generalized Jacobian of a free-floating base + N-link manipulator
%  Outward velocity propagation (Craig Ch. 5; Antonello-Valverde-Tsiotras 2019,
%  Eqs. 8, 20-22) -> momentum map (base rows of Eq. 27) -> momentum conservation
%  -> J* = Jm + Jb*K.
%  All vectors in the base body frame B, origin at the base CoM.
%  Assumes zero initial linear and angular momentum.
%  Requires the Symbolic Math Toolbox.

clear; clc;

%% ---------------- user settings ----------------------------------------
N      = 3;          % number of joints (all revolute)
planar = true;       % true: planar motion about z; false: spatial

%% ---------------- symbols ----------------------------------------------
q   = sym('q',   [N 1], 'real');      % joint angles
qd  = sym('qd',  [N 1], 'real');      % joint rates
m   = sym('m',   [N 1], 'positive');  % link masses
L   = sym('L',   [N 1], 'positive');  % DH link lengths a_i
ac  = sym('ac',  [N 1], 'positive');  % CoM distance from joint i along link i
Izz = sym('Izz', [N 1], 'positive');  % link inertias about CoM (planar)
syms mB IBzz b0 positive              % base mass, base inertia, mount offset

% DH parameters (standard DH, frame i at the distal end of link i)
alph  = zeros(N,1);   d = zeros(N,1);  % planar chain: edit for spatial arms
a     = L;

% Mount frame {0} in B (position and orientation)
P0  = [b0; 0; 0];
RB0 = eye(3);

% Link inertia tensors about the CoM, in link frames
Ic = cell(N,1);
for i = 1:N
    Ic{i} = Izz(i) * diag([0 0 1]);    % spatial: diag([Ixx Iyy Izz]) or full tensor
end
IB = IBzz * diag([0 0 1]);          % spatial: full base inertia tensor

%% ---------------- helper maps (anonymous => MATLAB & Octave) -----------
S0  = sym(0);  S1 = sym(1);   % keep every row symbolic (portable concatenation)
Rz  = @(t) [cos(sym(t)), -sin(sym(t)), S0; sin(sym(t)), cos(sym(t)), S0; S0, S0, S1];
Rx  = @(t) [S1, S0, S0; S0, cos(sym(t)), -sin(sym(t)); S0, sin(sym(t)), cos(sym(t))];
zax = [0; 0; 1];

%% ---------------- base twist and stacked rate vector -------------------
if planar
    syms vBx vBy wBz real
    vB = [vBx; vBy; sym(0)];   wB = [sym(0); sym(0); wBz];
    xb = [vBx; vBy; wBz];
    iP = 1:2;  iL = 3;     iE = [1 2 6];   % momentum rows, EE twist rows
else
    vB = sym('vB', [3 1], 'real');  wB = sym('wB', [3 1], 'real');
    xb = [vB; wB];
    iP = 1:3;  iL = 1:3;   iE = 1:6;
end
xd = [xb; qd];
nb = numel(xb);  nP = numel(iP);

%% ---------------- outward propagation in frame B -----------------------
R  = RB0;                         % orientation of the current frame in B
p  = P0;                          % origin of the current frame in B
v  = vB + cross(wB, P0);          % inertial velocity of that origin
w  = wB;                          % inertial angular velocity

P    = mB * vB;                   % linear momentum (base term)
Lmom = IB * wB;                   % angular momentum about base CoM (base term)

for i = 1:N
    zi   = R * zax;                              % joint i axis (z_{i-1})
    w    = w + qd(i) * zi;                       % omega_i
    Ri   = R * Rz(q(i)) * Rx(alph(i));          % frame i orientation
    pn   = p + R * Rz(q(i)) * [a(i); 0; d(i)];   % frame i origin
    rc   = pn + Ri * [ac(i) - a(i); 0; 0];       % CoM of link i
    vc   = v + cross(w, rc - p);                 % CoM velocity
    v    = v + cross(w, pn - p);                 % frame i origin velocity

    P    = P    + m(i) * vc;
    Lmom = Lmom + Ri*Ic{i}*Ri.' * w + m(i) * cross(rc, vc);

    R = Ri;  p = pn;
end
vE = v;  wE = w;                                  % EE at origin of frame N

%% ---------------- Jacobian blocks by differentiation ------------------
Mom = [P(iP); Lmom(iL)];
H   = jacobian(Mom, xd);
JE  = jacobian([vE; wE], xd);
JE  = JE(iE, :);

Hvv = H(1:nP, 1:nP);     Hvw = H(1:nP, nP+1:nb);     Hvq = H(1:nP, nb+1:end);
Hwv = H(nP+1:end, 1:nP); Hww = H(nP+1:end, nP+1:nb); Hwq = H(nP+1:end, nb+1:end);
Jb  = JE(:, 1:nb);       Jm  = JE(:, nb+1:end);

%% ---------------- momentum conservation -> base velocity map ----------
% P = 0 : vB = -Hvv \ (Hvw wB + Hvq qd)       (Hvv = M*I, trivially inverted)
% L = 0 : Hs wB + Hsq qd = 0
Mtot = mB + sum(m);
Hs   = Hww - Hwv * Hvw / Mtot;
Hsq  = Hwq - Hwv * Hvq / Mtot;
Kw   = -(Hs \ Hsq);                    % wB = Kw qd   (scalar division if planar)
Kv   = -(Hvw * Kw + Hvq) / Mtot;       % vB = Kv qd
K    = [Kv; Kw];

%% ---------------- generalized Jacobian ---------------------------------
Jstar = Jm + Jb * K;
Jstar = simplify(Jstar);               % optional: may be slow for N > 3

%% ---------------- planar barycentric closed form (virtual manipulator) --
if planar
    n    = N + 1;                              % bodies 0..N (0 = base)
    mm   = [mB; m];   II = [IBzz; Izz];
    aa   = [sym(0); ac];   LL = [b0; L];
    muin = cumsum(mm);
    c    = (LL .* muin - mm .* aa) / Mtot;     % virtual-manipulator lengths
    k    = mm .* aa + LL .* (Mtot - muin);     % first moments
    W    = sym(zeros(n));
    for j = 1:n
        for l = 1:n
            if j < l,     W(j,l) = c(j) * k(l);
            elseif j > l, W(j,l) = c(l) * k(j);
            else,         W(j,j) = c(j) * k(j) - mm(j) * aa(j) * (LL(j) - aa(j));
            end
        end
    end
    rel = [sym(0); cumsum(q)];                 % phi_j - theta_0
    G   = sym(zeros(n));
    for l = 1:n
        for kk = 1:n
            G(l,kk) = W(l,kk) * cos(rel(kk) - rel(l)) + double(l == kk) * II(l);
        end
    end
    h    = sum(G, 1).';                        % angular-momentum weights
    Htot = sum(h);
    Jth  = [sym(0); sym(0)];
    for i = 1:n
        Jth = Jth + c(i) * [-sin(rel(i)); cos(rel(i))];
    end
    Jcf = sym(zeros(3, N));
    for j = 1:N
        Hj  = sum(h(j+1:n));
        Jq  = [sym(0); sym(0)];
        for i = j+1:n
            Jq = Jq + c(i) * [-sin(rel(i)); cos(rel(i))];
        end
        Jcf(1:2, j) = Jq - Jth * Hj / Htot;
        Jcf(3, j)   = 1 - Hj / Htot;
    end
end

%% ---------------- numerical consistency check ---------------------------
params = [mB; IBzz; b0; m; Izz; L; ac];
if planar
    pv  = 0.2 + rand(numel(params), 1);
    qv  = pi * (2 * rand(N, 1) - 1);
    vars = [q; params];   vals = [qv; pv];
    Jn_rec = double(subs(Jstar, vars, vals));
    Jn_cf  = double(subs(Jcf,   vars, vals));
    fprintf('max |J*_recursive - J*_closed_form| = %.3e\n', max(abs(Jn_rec(:) - Jn_cf(:))));
end

%% ---------------- code generation --------------------------------------
if exist('matlabFunction', 'file')
    matlabFunction(Jstar, 'File', 'Jstar_fun', 'Vars', {q, params});
    matlabFunction(K,     'File', 'Kbase_fun', 'Vars', {q, params});
end

```

### 10.1 Structure of the script

| Block | Theory |
|---|---|
| Symbols, DH table, mount frame | §2 |
| Outward loop (`w`, `v`, `rc`, `vc`) | §3.3 |
| `jacobian(Mom, xd)`, `jacobian([vE; wE], xd)` | §4, §5.2 |
| `Hs`, `Hsq`, `Kw`, `Kv` | §6.2 |
| `Jstar = Jm + Jb*K` | §6.3 |
| Barycentric block (`c`, `k`, `W`, `G`, `h`, `Jcf`) | §8 |
| `matlabFunction` export | Evaluation of $J^*(\theta)$ and $K(\theta)$ |

### 10.2 Extending the script

**More joints.** Set `N`. Every block is written for general $N$.

**Spatial arm.**

1. Set `planar = false`.
2. Fill `alph` and `d` from the DH table.
3. Replace `Ic{i}` and `IB` with full inertia tensors.
4. Replace the centre-of-mass offset `[ac(i) - a(i); 0; 0]` with the actual offset vector of each link in its frame.

The planar closed-form block and its numerical check apply only when `planar = true`.

**Inertial components.** Pre-multiply the exported $J^*$ by $\operatorname{blkdiag}(R^{\mathcal F}_{\mathcal B},R^{\mathcal F}_{\mathcal B})$ (§7).

**Expression size.** The recursive $J^*$ is exact but large, because it keeps $1/\hat H_\omega$ unexpanded inside every entry. For analysis, the closed form of §8 is more readable. For evaluation, `matlabFunction` with its default optimization gives compact code from either form.
