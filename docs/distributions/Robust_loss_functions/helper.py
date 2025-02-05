import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm, uniform

# Define the x values
x = np.linspace(-3, 3, 1000)

# Define the Gaussian distribution (normal distribution)
normal_dist = norm.pdf(x, 0, 1)
box_dist = np.where(np.abs(x) <= 2.0, 0.25, 0) # Box distribution with width 2 and height 0.5

# Define a heavy-tailed "Gaussian" as a mixture of a Gaussian and a uniform distribution
uniform_dist = uniform.pdf(x, -5, 10)  # uniform distribution covering the range [-5, 5]
heavy_tail_dist = 0.8 * normal_dist + 0.2 * uniform_dist  # 90% Gaussian, 10% uniform


# Define the Welsch loss correctly
welsch_loss = 1 - np.exp(-0.5 * x**2)  # e^(-c*x^2), ensuring loss is low near zero
l2_loss = x**2
box_loss = np.where(np.abs(x) <= 1.0, 0, 1)  # Box loss with width 2 and height 0.5

# Plotting the distributions
plt.figure(figsize=(12, 5))

# Plot for probability distributions
plt.subplot(1, 2, 1)
plt.plot(x, normal_dist, 'b', label='Gaussian')
plt.plot(x, heavy_tail_dist, 'r', label='Heavy-tail Gaussian')
plt.plot(x, box_dist, 'g', label='Heavy-tail Gaussian')
plt.fill_between(x, 0, normal_dist, color='blue', alpha=0.2)
plt.fill_between(x, 0, heavy_tail_dist, color='red', alpha=0.2)
plt.fill_between(x, 0, box_dist, color='green', alpha=0.2)
plt.grid()
plt.title('Probability distributions')
plt.xlabel('x')
plt.ylabel('Probability density')
plt.legend()

# Plot for corresponding losses
plt.subplot(1, 2, 2)
#plt.plot(x, welsch_loss, 'r', label='Heavy-tail Gaussian')
plt.plot(x, l2_loss, 'b', label='Gaussian')
plt.plot(x, box_loss, 'g', label='Box')
#plt.fill_between(x, 0, welsch_loss, color='red', alpha=0.2)
plt.fill_between(x, 0, l2_loss, color='blue', alpha=0.2)
plt.fill_between(x, 0, box_loss, color='green', alpha=0.2)
plt.title('Corresponding losses')
plt.xlabel('x')
plt.grid()
plt.ylabel('Loss')
plt.legend()

plt.tight_layout()
plt.show()