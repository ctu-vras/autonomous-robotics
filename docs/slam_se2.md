---
title: SLAM in SE(2)
layout: default
nav_order: 3
mathjax: true
---


# Where the hell am I, and where is stuff around me?

SLAM in SE(2) with (i) measurement models of 2D/3D marker detectors, UWB, GPS/GNSS, odometry, and (ii) differential drive motion model.
We need to estimate the state of robot to decide the best following action.
We have states $x$, actions $u$ and measurements $z$.

We measure some object in robot coordinate frame (rcf) and, for known position of object in world coordinate frame (wcf), it is possible to locate robot in the world coordinate frame.

Measurements of marker in robot coordinate frame $m_r$ should be transformed to the world coordinate frame to get marker position in world coordinate frame $m_w$.

In robot coordinate frame, we introduce measurement inaccuracy

$$
p(z | x) = N(z; w2r(m, x_t), \Sigma),
$$

so the position $x_t$ can be expressed as
$$
x_t^* = argmax N(z; w2r(m, x_t), \Sigma) = argmin ||w2r(m, x_t)-z||_\Sigma ^2.
$$

For simirlarly accurate measurements, we get simplified equation
$$
x_t^* = argmin ||w2r(m, x_t) - z||^2
$$
for robot coordinate frame and
$$
x_t^* = argmin ||r2w(z, x_t) - m|| ^2
$$
for world coordinate frame.

For the known odometry, we express inaccuracy of odometry measurements
$$
x_t^*, x_{t+1}^* = \text{arg min}_{x_t, x_{t+1}} ||w2r(x_{t+1}, x_t) - z^{velocity}||_{\Sigma^{velocity}} ^2.
$$

The combination of marker-based localization and odometry-based localization is following
$$
x_t^* = argmin_{x_t} \Sigma ||w2r(m_i, x_t)-z_t^{m_i}||^2 + ||w2r(x_{t+1}, x_t)-z_{(t+1, t)}^{odom}||^2.
$$

Localization in SE2 with 2 known robot poses, 3 markers (measured) and known odometry is a multidimensional problem.
How many dimensions does the problem have?
15, bacause for each position and each robot measurement we consider 3 dimensions (x, y, angle), so $5 * 3 = 15$.

We need to solve ambiguity given by the fact that the marker position in relatation to the world coordinate frame origin is unknown and the found solution forms a subspace. This can be resolved by fixing one point position (absolute marker) or fixing starting robot position within the world coordinate frame.

## Factorgraph

Model used in robotics to represent a relationship between variables (robot poses) and measurements (marker measurements) is called factorgraph. It consists of variable nodes (variables we need to estimate such as poses) and n-ary factors (measurements related to variables or prior knowledge about variables).
In our previous SE2 and SE3 examples, we encountered odometry factors and observation factors (GPS measurements, marker measurements).

Advantages of using factorgraph are scalability (such as in number of markers or number of sensors and measurements) and flexibility (different types of measurements and constrains can be incorporated).

## Motion model

Diferential drive is often used to approximate wheeled robots, cars, or unmanned ground vehicles.

In SE2, the state in time $t$ consits of $x$,$y$ position and orientation $\Theta$.
The model is expressed as 

$$
\begin{bmatrix}
x_t \\
y_t \\
\Theta_t 
\end{bmatrix}
= 
\begin{bmatrix}
x_{t-1} + v_t \Delta t \Theta_{t-1} \\
y_{t-1} + v_t \Delta t \Theta_{t-1} \\
\Theta_{t-1} + \omega_t \Delta t
\end{bmatrix}
$$

For smooth approximation of all steps, we can use the following formula

$$
x_t = x_{t-1} + \int_0^{\Delta t} v_t cos(\Theta_{t-1} + \omega_t t) dt.
$$

After solving integral, we get following equation:
$$
x_t = x_{t-1} + \frac{v_t}{\omega_t}(sin (\Theta_{t-1}+\omega_t \Delta t) - sin(\Theta_{t-1}))
$$

$$
y_t = y_{t-1} + \frac{v_t}{\omega_t}(-cos (\Theta_{t-1}+\omega_t \Delta t) + cos(\Theta_{t-1})) \\
$$

$$
\Theta_t = \Theta_{t-1} + \omega \Delta t
$$

Note that these equations cannot be used if angular velocity is close or equal to zero, since division by zero is undefined.

Approximation by "half turn, then move, then half turn" approach is usually good enough for small time window and angular velocity. This simplification yields in following equations

$$
\begin{bmatrix}
x_t \\
y_t \\
\Theta_t 
\end{bmatrix}
= 
\begin{bmatrix}
x_{t-1} + v_t \Delta t cos (\Theta_{t-1} + \omega_t \frac{\Delta t}{2}) \\
y_{t-1} + v_t \Delta t sin (\Theta_{t-1} + \omega_t \frac{\Delta t}{2}) \\
\Theta_{t-1} + \omega_t \Delta t
\end{bmatrix}
$$

## Beacons(UWB)

Beacons are so-called 2D markers. The only information provided is distance from the robot.

For example, dimensionality of zero-loss subspace for 3 beacons and robot pose:

Beacons and robot pose are described by two coordinates each, so the space is eight dimensional, but with fixed 3 degrees of freedom, the dimensionality of zero-loss subspace is just 5. 

# How can I find myself without markers?

Use SLAM with lidar and camera and its efficient optimization on SE2/SE3 manifolds (Absolute orientation, Camera localization/calibration, ICP).

## Relative motion from known correspondences
In this case, we know which points in pointclouds should be mapped to eachother, therefore correspondences are know.

### How to solve the problem

It's nonlinear least square task since rotation matrix has nonlinear functions (sin, cos) inside of it. We can get rid of them by following steps.

### Absolute orientation problem in SE2

We have two pointclouds, p and q (taken with same sensor, one before movement and one after).

1. First substitute by subtraction of center of gravity from each point in both pointclouds
   $$
   p_i' = p_i - \frac{1}{N} \Sigma_i p_i,
   $$

   $$
   q_i' = q_i - \frac{1}{N} \Sigma_i q_i.
   $$

2. Solve $\theta^*$
    $$\quad \theta^{\star}=\arg \min _\theta \sum_i\left\|\mathbf{R}_\theta \mathbf{p}_i^{\prime}-\mathbf{q}_i^{\prime}\right\|^2 \quad.
    $$

3. For optimal rotation $\theta^*$, we need to find a translation, such that $arg min = 0$

    $$
    \mathbf{t}^{\star}=\arg \min _{\mathbf{t}}\left\|\mathbf{R}_{\theta \star} \tilde{\mathbf{p}}+\mathbf{t}-\tilde{\mathbf{q}}\right\|^2=\tilde{\mathbf{q}}-\mathbf{R}_{\theta \star} \tilde{\mathbf{p}}.
    $$

The second step is solved as following

$$
\begin{aligned}
\theta^{\star} & =\arg \min _\theta \sum_i\left\|\mathbf{R}_\theta \mathbf{p}_i^{\prime}-\mathbf{q}_i^{\prime}\right\|^2 \\
& =\arg \min _\theta \sum_{\mathbf{p}^{\prime}, \mathbf{q}^{\prime}}\left\|\left[\begin{array}{cc}
\cos (\theta) & -\sin (\theta) \\
\sin (\theta) & \cos (\theta)
\end{array}\right]\left[\begin{array}{l}
p_x^{\prime} \\
p_y^{\prime}
\end{array}\right]-\left[\begin{array}{l}
q_x^{\prime} \\
q_y^{\prime}
\end{array}\right]\right\|^2 \\
& =\arg \min _\theta \sum_{\mathbf{p}^{\prime}, \mathbf{q}^{\prime}}\left\|\begin{array}{l}
\cos (\theta) p_x^{\prime}-\sin (\theta) p_y^{\prime}-q_x^{\prime} \\
\sin (\theta) p_x^{\prime}+\cos (\theta) p_y^{\prime}-q_y^{\prime}
\end{array}\right\|^2 \\
& =\arg \min _\theta \sum_{\mathbf{p}^{\prime}, \mathbf{q}^{\prime}}\left(p_x^{\prime} \cos (\theta)-p_y^{\prime} \sin (\theta)-q_x^{\prime}\right)^2+\left(p_x^{\prime} \sin (\theta)+p_y^{\prime} \cos (\theta)-q_y^{\prime}\right)^2
\end{aligned}
$$

Derivative:
$$
\begin{aligned}
& \sum_{\mathbf{p}^{\prime}, \mathbf{q}^{\prime}} 2\left(p_x^{\prime} \cos (\theta)-p_y^{\prime} \sin (\theta)-q_x^{\prime}\right) \cdot\left(-p_x^{\prime} \sin (\theta)-p_y^{\prime} \cos (\theta)\right) \\
& +2\left(p_x^{\prime} \sin (\theta)+p_y^{\prime} \cos (\theta)-q_y^{\prime}\right) \cdot\left(p_x^{\prime} \cos (\theta)-p_y^{\prime} \sin (\theta)\right)=0
\end{aligned}
$$

Simplify:
$$
\begin{aligned}
& \sum_{\mathbf{p}^{\prime}, \mathbf{q}^{\prime}} p_x^{\prime 2} \cdot(-\cos (\theta) \sin (\theta)+\sin (\theta) \cos (\theta)) \\
& +p_y^{\prime 2} \cdot(\sin (\theta) \cos (\theta)-\cos (\theta) \sin (\theta)) \\
& +p_x^{\prime} p_y^{\prime} \cdot\left(-\cos ^2(\theta)+\sin ^2(\theta)+\cos ^2(\theta)-\sin ^2(\theta)\right) \\
& +p_x^{\prime} \cdot\left(q_x^{\prime} \sin (\theta)-q_y^{\prime} \cos (\theta)\right) \\
& +p_y^{\prime} \cdot\left(q_x^{\prime} \cos (\theta)+q_y^{\prime} \sin (\theta)\right)=0
\end{aligned}
$$

$$
\begin{aligned}
& \sum_{\mathbf{p}^{\prime}, \mathbf{q}^{\prime}} 
p_x^{\prime} \cdot\left(q_x^{\prime} \sin (\theta)-q_y^{\prime} \cos (\theta)\right)  +p_y^{\prime} \cdot\left(q_x^{\prime} \cos (\theta)+q_y^{\prime} \sin (\theta)\right)=0
\end{aligned}
$$
Substitution should get us only one trigonometric function in equation, so everything can be solved as linear least square problem.
$$
\begin{aligned}
& \sum_{\mathbf{p}^{\prime}, \mathbf{q}^{\prime}} \tan (\theta) \cdot\left(p_x^{\prime} q_x^{\prime}+p_y^{\prime} q_y^{\prime}\right)+\left(p_y^{\prime} q_x^{\prime}-p_x^{\prime} q_y^{\prime}\right)=0 \\
& \theta^{\star}=\arctan \left(\frac{\sum_{\mathbf{p}^{\prime}, \mathbf{q}^{\prime}} p_x^{\prime} q_y^{\prime}-p_y^{\prime} q_x^{\prime}}{\sum_{\mathbf{p}^{\prime}, \mathbf{q}^{\prime}} p_x^{\prime} q_x^{\prime}+p_y^{\prime} q_y^{\prime}}\right)=\arctan \left(\frac{H_{x y}-H_{y x}}{H_{x x}+H_{y y}}\right)
\end{aligned}
$$

where $H$ stands for covariance matrix.

$$
H = \Sigma_i p'_i {q'}_{i}^{T} \\
H_{xx} = p'_x {q'}_{x}^{T} \\
H_{xy} = p'_x {q'}_{y}^{T} \\
H_{yx} = p'_y {q'}_{x}^{T} \\
H_{yy} = p'_y {q'}_{y}^{T} \\
$$

For third step, translation estimation is derived as:

$$
\mathbf{t}^{\star}=\arg \min _{\mathbf{t}}\left\|\mathbf{R}_{\theta \star} \tilde{\mathbf{p}}+\mathbf{t}-\tilde{\mathbf{q}}\right\|^2=\tilde{\mathbf{q}}-\mathbf{R}_{\theta \star} \tilde{\mathbf{p}}
$$

Absolute orientation in SE3:

$$
\mathbf{z}^v=\arg \min _{\mathbf{t}, \mathbf{R}} \sum_i\|\mathbf{R p}+\mathbf{t}-\mathbf{q}\|^2=\arg \min _{\mathbf{t}, \mathbf{R}} \sum_i\left\|\mathbf{R} \mathbf{p}_i^{\prime}-\mathbf{q}_i^{\prime}\right\|^2+\|\mathbf{R} \tilde{\mathbf{p}}+\mathbf{t}-\tilde{\mathbf{q}}\|^2
$$

Substitution:
$$
\mathbf{p}_i^{\prime}=\mathbf{p}_i-\underbrace{\frac{1}{N} \sum_i \mathbf{p}_i}_{\tilde{\mathbf{p}}}, \quad \mathbf{q}_i^{\prime}=\mathbf{q}_i-\underbrace{\frac{1}{N} \sum_i \mathbf{q}_i}_{\tilde{\mathbf{q}}}
$$

Solution:
$$
\begin{aligned}
& \mathbf{H}=\sum_i \mathbf{p}_i^{\prime} \mathbf{q}_i^{\prime \top} \ldots \text { covariance matrix with SVD decomposition } \mathbf{H}=\mathbf{U S V}^{\top} \\
& \mathbf{R}^{\star}=\arg \min _{\mathbf{R}} \sum_i\left\|\mathbf{R} \mathbf{p}_i^{\prime}-\mathbf{q}_i^{\prime}\right\|^2=\mathbf{V} \mathbf{U}^{\top} \\
& \mathbf{t}^{\star}=\arg \min _{\mathbf{t}}\left\|\mathbf{R}^{\star} \tilde{\mathbf{p}}+\mathbf{t}-\tilde{\mathbf{q}}\right\|^2=\tilde{\mathbf{q}}-\mathbf{R}^{\star} \tilde{\mathbf{p}}
\end{aligned}
$$

Note that SVD decomposition may not return correct rotation matrix (Rotation matrix must be ortogonal with determinant 1. If determinant of matrix given by SVD is -1, just apply something like following code:)
$$
\texttt{Vt[:, -1] = Vt[:, -1] * -1}
$$
or apply following formula:
$$
\texttt{Vt[:, -1] = Vt[:, -1] * d}
$$
where d is the determinant or rotation matrix.

Note that this approach can be used only in static environment with known correspondences.

## Relative motion from unknown correspondences

No correspondences are known in this case.

Correspondences may be solved via KNN (K-nearest-neighbors) algorithm.

1. Initialize $\mathbf{R}^{\star}=\mathbf{R}_0, \mathbf{t}^{\star}=\mathbf{t}_0$

2. Solve nearest neighbour
   $$
   c(i)^{\star}=\underset{c(i)}{\arg \min } \sum_i\left\|\mathbf{R}^{\star} \mathbf{p}_i+\mathbf{t}^{\star}-\mathbf{q}_{c(i)}\right\|^2
   $$

3. Solve absolute orientation
   $$
   \mathbf{R}^{\star}, \mathbf{t}^{\star},=\underset{\mathbf{R} \in S O(3), \mathbf{t}}{\arg \min } \sum_i\left\|\mathbf{R} \mathbf{p}_i+\mathbf{t}-\mathbf{q}_{c(i)^{\star}}\right\|^2
   $$

4. Repeat steps 2 and 3 a few times to get the best solution.

Output: $\mathbf{R}^{\star}, \mathbf{t}^{\star} \Rightarrow \mathbf{z}_t^v$

This algorithm can be further improved by outlier rejection. Outliers are points without any close neighbors and should not be considered as all neighbors are too far away.

You can also use robust norms to improve the algorithm:

Given max $\Delta R$ and $\Delta t$ deviations and max $\Theta$ error.

1. Initialize $\mathbf{R}^{\star}=\mathbf{R}_0, \mathbf{t}^{\star}=\mathbf{t}_0$

2. Solve nearest neighbour
   $$
   c(i)^{\star}=\underset{c(i)}{\arg \min } \sum_i\left\|\mathbf{R}^{\star} \mathbf{p}_i+\mathbf{t}^{\star}-\mathbf{q}_{c(i)}\right\|^2
   $$

3. Reject outliers
   $$
   \texttt{if } \left\|\mathbf{R}^{\star} \mathbf{p}_i+\mathbf{t}^{\star}-\mathbf{q}_{c(i)}\right\|^2 \geq \Theta \texttt{then reject found correspondence}
   $$

4. Solve absolute orientation
   $$
   \mathbf{R}^{\star}, \mathbf{t}^{\star},=\underset{\mathbf{R} \in S O(3), \mathbf{t}}{\arg \min } \sum_i \rho \left\|\mathbf{R} \mathbf{p}_i+\mathbf{t}-\mathbf{q}_{c(i)^{\star}}\right\|^2
   $$

Output: $\mathbf{R}^{\star}, \mathbf{t}^{\star} \Rightarrow \mathbf{z}_t^v$

The previous mapping between two pointclouds is called point-to-point mapping. There are other mappings, such as point-to-plane.

## Point-to-Point

Works well with **unstructred environments** where no pattern can be recognized, when **pointcloud data are sparse** and when **computational power is constrained** (old hardware or high frequency requirements), since it's **faster than point-to-plane**. It's **more simple** and **versatile** than point-to-plane.

## Point-to-Plane

Works better with **structured environments with planar surfaces**, there is a **high accuracy** required for the task, and **pointcloud is dense**.
It has **better convergence** compared to point-to-point and is **less sensitive to noise** in lidar data.

### Point to plane measure model

$$
\mathbf{R}^{\star}, \mathbf{t}^{\star}, \underset{\mathbf{R} \in S O(3), \mathbf{t}}{\arg \min } \sum_i\left\|\mathbf{R} \mathbf{p}_i+\mathbf{t}-\hat{\mathbf{q}}_i\right\|^2
$$

Where $\hat{\mathbf{q}}$ is virtual pointcloud computed from original q pointcloud.
