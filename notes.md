#### 平移表示
$$
\frac{\mathrm d\boldsymbol{x}}{\mathrm dt}=\boldsymbol{v}, \frac{\mathrm d\boldsymbol{v}}{\mathrm dt}=\frac{\boldsymbol{f}}m,
$$$$
\boldsymbol{v}'\leftarrow \boldsymbol{v} + \Delta tM^{-1}\boldsymbol{f}, \\
\boldsymbol{x}'\leftarrow \boldsymbol{x} + \Delta t\boldsymbol{v}.
$$

#### 旋转表示
旋转矩阵
$$
\boldsymbol{R} = \left[\begin{matrix}
    r_{x0} & r_{y0} & r_{z0} \\
    r_{x1} & r_{y1} & r_{z1} \\
    r_{x2} & r_{y2} & r_{z2}
\end{matrix}\right]
$$欧拉角表示（分解为三个定轴的旋转）
$$
R=R_xR_yR_z, \\
R_x(\phi) = \left[\begin{matrix}
    1 & 0 & 0 \\
    0 & \cos\phi & -\sin\phi \\
    0 & \sin\phi & \cos\phi
\end{matrix}\right],
R_y(\phi) = \left[\begin{matrix}
    \cos\phi & 0 & \sin\phi \\
    0 & 1 & 0 \\
    -\sin\phi & 0 & \cos\phi
\end{matrix}\right],
R_z(\phi) = \left[\begin{matrix}
    \cos\phi & -\sin\phi & 0 \\
    \sin\phi & \cos\phi & 0 \\
    0 & 0 & 1
\end{matrix}\right]
$$四元数表示
$$
\boldsymbol{q} = (s, \boldsymbol{v}) = (s, x, y, z) = s + x\mathrm i + y\mathrm j + z\mathrm k, \\
\boldsymbol{q}_1\times\boldsymbol{q}_2 = (s_1s_2 - \boldsymbol{v}_1\boldsymbol{v}_2, s_1\boldsymbol{v}_2+s_2\boldsymbol{v}_1+\boldsymbol{v}_1\times\boldsymbol{v}_2), \\
\|\boldsymbol{q}\| = \sqrt{s^2+\boldsymbol{v}\cdot\boldsymbol{v}}.
$$则绕轴$\boldsymbol{n}$旋转角度$\phi$可以表示为$Q=(\cos(\phi/2), \boldsymbol{n}\sin(\phi/2))$，对于坐标$\boldsymbol{x}$，旋转变换后坐标变为$Q(0, \boldsymbol{x})Q^{-1}$.

#### 旋转动力
##### 惯性张量$\boldsymbol{I}$
$$
\boldsymbol{I} = \bold{Id}\mathrm{tr}(\boldsymbol{C}) - \boldsymbol{C}, \\
\boldsymbol{C} = \sum_i m_i\boldsymbol{x}_i\boldsymbol{x}_i^\bold{T}.
$$当物体作旋转变换$\boldsymbol{R}$，则
$$
\boldsymbol{I}' = \boldsymbol{RIR}^\bold T.
$$

##### 力矩$\boldsymbol{\tau}$
$$
\boldsymbol{\tau} = \sum_i (\boldsymbol{x}_i-\boldsymbol{x}_c) \times \boldsymbol{f}_i
$$

##### 角动量$\boldsymbol{L}$
$$
\frac{\mathrm d\boldsymbol{L}}{\mathrm dt} = \boldsymbol{\tau}
$$

##### 角速度$\boldsymbol{\omega}$
$\boldsymbol{\omega}$有$\boldsymbol{L}=\boldsymbol{I\omega}$.

综上，
$$
\boldsymbol{R} \leftarrow \mathrm{To~matrix}(\boldsymbol{q}), \\
\boldsymbol{\tau}_i \leftarrow (\boldsymbol{Rr}_i) \times \boldsymbol{f}_i, \\
\boldsymbol{L}' \leftarrow \boldsymbol{L} + \Delta t\boldsymbol{\tau}, \\
\boldsymbol{I} \leftarrow \boldsymbol{RI}_0\boldsymbol{R}^\bold T, \\
\boldsymbol{\omega} \leftarrow \boldsymbol{I}^{-1}\boldsymbol{L}, \\
\boldsymbol{q}' \leftarrow \boldsymbol{q} + (0, \Delta t\boldsymbol{\omega} / 2) \times \boldsymbol{q}.
$$

#### Impulse-based Collision
物体 A 与 B 碰撞，碰撞点$P$，A，B 质心为$p_A, p_B$，向量$r_A = P - p_A, r_B = P - p_B$，分别有线速度和角速度$v_A, v_B, \omega_A, \omega_B$，
则$P$处 A，B 的速度分别为$v_{pA} = v_A + \omega_A \times r_A, v_{pB} = v_B + \omega_B \times r_B$，相对速度$v_{rel} = v_{pB} - v_{pA}$.