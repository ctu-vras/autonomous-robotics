---
title: Kalman Filter
layout: default
nav_order: 5
mathjax: true
author: Ondřej Franek
---

# Bayes and Kalman Filter
{: .no_toc }
This chapter explains the basics of Bayes filter and of (extended) Kalman filter.


## Prerequisites

{: .definition } 
Event $\mathbf{A}$ is *conditionally independent* on $\mathbf{C}$ given $\mathbf{B}$ iff $p(\mathbf{A} \mid \mathbf{B}, \mathbf{C}) = p(\mathbf{A} \mid \mathbf{B})$.

{: .definition } 
State $\mathbf{x}_{t-1}$ is *complete* iff future $\mathbf{x}_t$ is conditionally independent of the past given $$\mathbf{x}_{t-1}$$.



Consequences:
- State-transition probability:

$$\begin{equation} \label{eq:st-pst}
p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t) = p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t, x_{0:t-2}, z_{1:t-1}, u_{1:t-1})
\end{equation}$$

- Measurement probability

$$\begin{equation} \label{eq:m-pst}
p(\mathbf{z}_t | \mathbf{x}_t) = p(\mathbf{z}_t | \mathbf{x}_t, x_{0:t-1}, z_{1:t-1}, u_{1:t})
\end{equation}$$

<figure style="text-align: center;">
  <img src="{{ site.baseurl }}/docs/kalman/complete_states.png" width="800">
  <figcaption><em>Figure: </em>Complete state illustration</figcaption>
</figure>







## Bayes Filter

The Bayes filter is an algorithm for probabilistic state estimation in dynamic systems. It predicts the state of a system over time, given control inputs and sensory measurements. The filter performs two main steps in a repetitive manner: prediction and measurement update.
- **Initialization:** The belief of the initial state is represented by $\text{bel}(\mathbf{x}_0)$ at time $t = 1$.
- **Prediction step:** Given the control action $u_t$ at time $t$, the prediction step computes a prior belief state:

$$\begin{equation} \label{eq:bayes-prior}
\overline{\text{bel}}(\mathbf{x}_t) = \int p(\mathbf{x}_t | \mathbf{x}_{t-1}, \mathbf{u}_t) \, \text{bel}(\mathbf{x}_{t-1}) \, \text{d}\mathbf{x}_{t-1} 
\end{equation}$$

- **Measurement update:** Upon receiving a new measurement $z_t$, the measurement update refines the predicted belief to produce a posterior belief state:


$$\begin{equation} \label{eq:bayes-posterior}
\text{bel}(\mathbf{x}_t) = \eta \, p(\mathbf{z}_t | \mathbf{x}_t) \, \overline{\text{bel}}(\mathbf{x}_t), 
\end{equation}$$

<span style="display:inline-block; margin-left: 1.5em;">where $\eta$ is a normalization constant that ensures the posterior belief is a valid probability distribution by integrating to 1.


## Kalman Filter

The Kalman filter is a powerful algorithm used for estimating the state of a system over time. It operates on a simple yet effective principle: predict the future state, then update this prediction with new observations. It handles systems with uncertainties and noise quickly and effectively, making it widely utilized.

### Linear System with Gaussian Noise

The Kalman filter assumes a linear system with Gaussian noise:

$$\begin{align} \label{eq:lin_sys}
\mathbf{x}_t &= \mathbf{A}_t \mathbf{x}_{t-1} + \mathbf{B}_t \mathbf{u}_t + \mathbf{w}_t, \\
\mathbf{z}_t &= \mathbf{C}_t \mathbf{x}_t + \mathbf{v}_t .
\end{align}$$

where $\mathbf{A}_t$, $\mathbf{B}_t$, $\mathbf{C}_t$ are matrices that define the system dynamics, $\mathbf{u}_t$ is the control input, $\mathbf{x}_t$ is state vector and
$\mathbf{w} \sim \mathcal{N}(0, \mathbf{R}_t)$ and $\mathbf{v} \sim \mathcal{N}(0, \mathbf{Q}_t)$ are the transition and measurement noise with covariance matrices $\mathbf{R}_t$ and $\mathbf{Q}_t$, respectively. This implies that all probability distributions involved are Gaussian, which simplifies computation. The state-transition and measurement probabilities are modeled as

$$\begin{align} \label{eq:gauss-st-pst}
p(\mathbf{x}_t | \mathbf{x}_{t-1}, \mathbf{u}_t) &= \mathcal{N}({\mathbf{x}_t};\mathbf{A}_t\mathbf{x}_{t-1} + \mathbf{B}_t\mathbf{u}_t, \mathbf{R}_t), \\ \label{eq:gauss-meas-pst} 
p(\mathbf{z}_t | \mathbf{x}_t) &= \mathcal{N}({\mathbf{z}_t};\mathbf{C}_t\mathbf{x}_t, \mathbf{Q}_t). 
\end{align}$$



### Implementation of Kalman Filter

Combining the principles of the Bayes filter (equations \ref{eq:bayes-prior} and \ref{eq:bayes-posterior}), with prerequisities (equations \ref{eq:st-pst} and \ref{eq:m-pst}) and the properties of linear Gaussian systems (equations \ref{eq:gauss-st-pst} and \ref{eq:gauss-meas-pst}), we can derive the Kalman filter update equations:


- **Prediction step** (new action $\mathbf{u}_t$):
$$\begin{align}
  \overline{\boldsymbol{\mu}}_t &= \mathbf{A}_t \boldsymbol{\mu}_{t-1} + \mathbf{B}_t \mathbf{u}_t \\
  \overline{\boldsymbol{\Sigma}}_t &= \mathbf{A}_t \boldsymbol{\Sigma}_{t-1} \mathbf{A}_t^\top + \mathbf{R}_t \\
  \overline{\text{bel}}(\mathbf{x}_t) &= \mathcal{N}({\mathbf{x}_t};\overline{\boldsymbol{\mu}}_t, \overline{\boldsymbol{\Sigma}}_t) 
\end{align}$$

- **Measurement update** (new measurement $\mathbf{z}_t$):
$$\begin{align}
  \mathbf{K}_t &= \overline{\boldsymbol{\Sigma}}_t \mathbf{C}_t^\top (\mathbf{C}_t \overline{\boldsymbol{\Sigma}}_t \mathbf{C}_t^\top + \mathbf{Q}_t)^{-1} \\
  \boldsymbol{\mu}_t &= \overline{\boldsymbol{\mu}}_t + \mathbf{K}_t(\mathbf{z}_t - \mathbf{C}_t \overline{\boldsymbol{\mu}}_t) \\
  \boldsymbol{\Sigma}_t &= (\mathbf{I} - \mathbf{K}_t \mathbf{C}_t) \overline{\boldsymbol{\Sigma}}_t \\
  \text{bel}(\mathbf{x}_t) &= \mathcal{N}({\mathbf{x}_t};\boldsymbol{\mu}_t, \boldsymbol{\Sigma}_t)
\end{align}$$
where $\boldsymbol{\mu}_t$ is the estimated mean, $\boldsymbol{\Sigma}_t$ is the estimated uncertainty (covariance), and $\mathbf{K}_t$ is the Kalman gain which minimizes the estimated uncertainty.


In the following figure, we can see the propagation of the state estimate and its uncertainty through a linear function in the prediction step. The <span style="color:#2ca02c"><strong>Green Gaussian</strong></span> represents the prior belief, and the <span style="color:#d62728"><strong>Red Gaussian</strong></span> represents the posterior belief.


<figure style="text-align: center;">
  <img src="{{ site.baseurl }}/docs/kalman/predict_intuition_KF.png" width="600">
  <figcaption><em>Figure: </em>Linear system propagation</figcaption>
</figure>




### Kalman Filter Example: Squeezed Gaussian

In this example of a Kalman Filter, the state of a system is represented by position and velocity of the robot. The state can be expressed as a vector $\textbf{x} = \begin{bmatrix} x \\ v \end{bmatrix}$. In this example the state-transition probability is defined as

$$
p(\mathbf{x}_t | \mathbf{x}_{t-1}, \mathbf{u}_t) = \mathcal{N} ( \mathbf{x}_t; \underbrace{\begin{bmatrix} 1 & 1 \\ 0 & 1 \end{bmatrix}}_{\mathbf{A}_t} \mathbf{x}_{t-1} + \underbrace{\begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}}_{\mathbf{B}_t} \mathbf{u}_t, \underbrace{\begin{bmatrix} 0.01 & 0 \\ 0 & 0.01 \end{bmatrix}}_{\mathbf{R}_t} )
$$

and measurement probability is 

$$
p(\mathbf{z}_t | \mathbf{x}_t) = \mathcal{N} ( \mathbf{z}_t; \underbrace{\begin{bmatrix} 1 & 0 \end{bmatrix}}_{\mathbf{C}_t} \mathbf{x}_{t-1}, \underbrace{\begin{bmatrix} 0.3 \end{bmatrix}}_{\mathbf{Q}_t} ) \text{.}
$$


When there are no measurements available the probability distribution (modeled as a Gaussian) tends to become more squeezed and skewed over time. This is due to the growing uncertainty in predictions as time progresses. Additionally, the skewing of the distribution is caused by the linear dependence of position on velocity. The following sequence of images demonstrates how the distribution evolves from $x_0$ to $x_5$, visually representing the increasing uncertainty at different time steps.


<figure style="text-align: center;">
  <div style="display: flex; justify-content: center; gap: 10px; flex-wrap: wrap;">
    
    <div style="flex: 1; max-width: 32%;">
      <img src="{{ site.baseurl }}/docs/kalman/squeezing_x0.png" style="width: 100%;">
      <figcaption>time \(t = 0\)</figcaption>
    </div>
    
    <div style="flex: 1; max-width: 32%;">
      <img src="{{ site.baseurl }}/docs/kalman/squeezing_x3.png" style="width: 100%;">
      <figcaption>time \(t = 3\)</figcaption>
    </div>
    
    <div style="flex: 1; max-width: 32%;">
      <img src="{{ site.baseurl }}/docs/kalman/squeezing_x5.png" style="width: 100%;">
      <figcaption>time \(t = 5\)</figcaption>
    </div>
    
  </div>
  <figcaption><em>Figure: </em>Visualization of state squeezing and skewing at different time steps.</figcaption>
</figure>



If the robot receives some measurements, the probability distribution undergoes a significant correction. Each measurement provides additional information that helps refine the estimates of the robot's state, reducing uncertainty and compensating for the previous skewing and squeezing effects. The corrected probability Gaussian is depicted in the following figure.

<figure style="text-align: center;">
  <img src="{{ site.baseurl }}/docs/kalman/squeezing_end.png" width="500">
  <figcaption><em>Figure:</em> Probability after the measurement step.</figcaption>
</figure>



### Conclusion

The Kalman Filter is a fundamental tool in control systems and signal processing, efficiently estimating states in linear systems with Gaussian noise. Its recursive structure is well-suited for real-time applications in various fields like navigation and tracking.
However, the effectiveness of the Kalman Filter is constrained by its assumption that both system dynamics and measurement functions are linear. This limitation restricts its applicability to linear systems, as it struggles to accurately address deviations arising from non-linear behaviors in real-world scenarios. To manage non-linearities, adaptations such as the Extended Kalman Filter or alternative approaches like the Particle Filter are commonly employed.


{: .important-title }
> Summary
>
> - The Kalman filter is a special case of the Bayes filter for linear Gaussian systems.
> - Assumptions:
>     - The system model is *linear*  
>     - Process and measurement noise are Gaussian-distributed
> - State estimation is done in two recursive steps:
>     - **Prediction**: Estimate the state based on previous belief and control input.  
>     - **Update**: Refine the estimate using the latest measurement.
> - The Kalman Filter minimizes the uncertainty in the state estimate by optimally combining predictions and noisy measurements.
> - In the absence of measurements, the uncertainty grows and the Gaussian distribution becomes squeezed/skewed over time.
> - When measurements are available, the filter re-centers and narrows the distribution — reducing uncertainty and correcting drift.




## Extended Kalman Filter

The Extended Kalman Filter (EKF) is an adaptation of the Kalman Filter for non-linear systems. While the Kalman Filter is restricted to linear models, the EKF allows for a broader range of applications by linearizing about the current estimate.

### Non-Linear System with Gaussian Noise

The state and measurement models for a non-linear system can be expressed as:
$$ \begin{align}
\mathbf{x}_t &= \mathbf{g}(\mathbf{x}_{t-1}, \mathbf{u}_t) + \mathbf{w}, \\
\mathbf{z}_t &= \mathbf{h}(\mathbf{x}_t) + \mathbf{v},
\end{align} $$
where $\mathbf{g}$ and $\mathbf{h}$ are non-linear functions of the state and control inputs, and $\mathbf{w} \sim \mathcal{N}(0, \mathbf{R}_t)$ and $\mathbf{v} \sim \mathcal{N}(0, \mathbf{Q}_t)$ represent the transition and measurement noise with covariance matrices $\mathbf{R}_t$ and $\mathbf{Q}_t$, respectively.


### Linearization
The EKF approximates these non-linear functions using the first-order Taylor expansion around the current estimate:
$$\begin{align}
\mathbf{g}(\mathbf{u}_t, \mathbf{x}_{t-1}) &\approx \mathbf{g}(\mathbf{u}_t, \boldsymbol{\mu}_{t-1}) + \mathbf{G}_t(\mathbf{x}_{t-1} - \boldsymbol{\mu}_{t-1}), \label{ekf-approx} \\
\mathbf{h}(\mathbf{x}_t) &\approx \mathbf{h}(\boldsymbol{\mu}_t) + \mathbf{H}_t(\mathbf{x}_t - \boldsymbol{\mu}_t), \label{ekf-measurement-approx}
\end{align}$$
where $\mathbf{G}_t$ and $\mathbf{H}_t$ are defined as
$$\begin{align}
\mathbf{G}_t &=  \frac{\partial \mathbf{g}(\mathbf{x} = \boldsymbol{\mu}_{t-1}, \mathbf{u} = \mathbf{u}_t)}{\partial \mathbf{x}} \label{jacobian-matrix}, \\
\mathbf{H}_t &= \frac{\partial \mathbf{h}(\mathbf{x} = \boldsymbol{\mu}_t)}{\partial \mathbf{x}}.
\label{jacobian-measurement-matrix}
\end{align}$$

The state-transition and measurement probabilities are modeled as
$$\begin{align}
p(\mathbf{x}_t | \mathbf{x}_{t-1}, \mathbf{u}_t) &\approx \mathcal{N}\left(\mathbf{x}_t; \mathbf{g}(\boldsymbol{\mu}_{t-1}, \mathbf{u}_t) + \mathbf{G}_t(\mathbf{x}_{t-1} - \boldsymbol{\mu}_{t-1}), \mathbf{R}_t\right), \label{nonlin-st-pst} \\
p(\mathbf{z}_t | \mathbf{x}_t) &\approx \mathcal{N}\left(\mathbf{z}_t; \mathbf{h}(\overline{\boldsymbol{\mu}}_t) + \mathbf{H}_t(\mathbf{x}_t - \overline{\boldsymbol{\mu}}_t), \mathbf{Q}_t\right)\label{nonlin-meas-pst}
\end{align}$$


In the following figures we can observe the propagation of the prior belief, represented by the <span style="color:#2ca02c"><strong>Green Gaussian</strong></span>, through a non-linear and a linearized system.

<figure style="text-align: center;">
  <div style="display: flex; justify-content: center; gap: 20px; flex-wrap: wrap;">

    <div style="flex: 1; max-width: 48%;">
      <img src="{{ site.baseurl }}/docs/kalman/nonlin_gauss_intuition.png" style="width: 100%;">
      <figcaption><em>Figure:</em> Non-linear system propagation.</figcaption>
    </div>

    <div style="flex: 1; max-width: 48%;">
      <img src="{{ site.baseurl }}/docs/kalman/linearized_intuition_EKF.png" style="width: 100%;">
      <figcaption><em>Figure:</em> Linearized system propagation.</figcaption>
    </div>

  </div>
</figure>





### Implementation of Extended Kalman Filter

Combining the principles of the Bayes filter (equations \ref{eq:bayes-prior} and \ref{eq:bayes-posterior}), with prerequisities (equations \ref{eq:st-pst} and \ref{eq:m-pst}) and the properties of linearized Gaussian systems (equations \ref{nonlin-st-pst} and \ref{nonlin-meas-pst}), we can derive the Kalman filter update equations:

- **Prediction step** (new action $\mathbf{u}_t$):
$$\begin{align}
  \overline{\boldsymbol{\mu}}_t &= \mathbf{g}(\boldsymbol{\mu}_{t-1}, \mathbf{u}_t) \\
  \overline{\boldsymbol{\Sigma}}_t &= \mathbf{G}_t \boldsymbol{\Sigma}_{t-1} \mathbf{G}_t^\top + \mathbf{R}_t \\
  \overline{\text{bel}}(\mathbf{x}_t) &= \mathcal{N}({\mathbf{x}_t};\overline{\boldsymbol{\mu}}_t, \overline{\boldsymbol{\Sigma}}_t) 
\end{align}$$

- **Measurement update** (new mwasurement $\mathbf{z}_t$)
$$\begin{align}
  \mathbf{K}_t &= \overline{\boldsymbol{\Sigma}}_t \mathbf{H}_t^\top (\mathbf{H}_t \overline{\boldsymbol{\Sigma}}_t \mathbf{H}_t^\top + \mathbf{Q}_t)^{-1} \\
  \boldsymbol{\mu}_t &= \overline{\boldsymbol{\mu}}_t + \mathbf{K}_t (\mathbf{z}_t - \mathbf{h}(\overline{\boldsymbol{\mu}}_t)) \\
  \boldsymbol{\Sigma}_t &= (\mathbf{I} - \mathbf{K}_t \mathbf{H}_t) \overline{\boldsymbol{\Sigma}}_t \\
  \text{bel}(\mathbf{x}_t) &= \mathcal{N}({\mathbf{x}_t};\boldsymbol{\mu}_t, \boldsymbol{\Sigma}_t)
\end{align}$$


### Conclusion

The Extended Kalman Filter extends the Kalman Filter to non-linear systems by linearizing about the current state estimate. However, this approach introduces errors due to approximation, particularly when the system dynamics or measurements are highly non-linear. These errors can lead to inaccuracies and potential divergence of the filter, especially with poor initial estimates or significant non-linear deviations. Thus, for systems with strong non-linearities, more robust methods like the Unscented Kalman Filter or Particle Filter may provide better performance



## Lecture notes from previous year (2023/24)

<object data="{{ site.url }}/docs/kalman/kalmanfilter.pdf" width="800" height="1000" type="application/pdf"></object>
