import numpy as np
import matplotlib.pyplot as plt

# Define weights and calculate the cumulative sum
weights = np.array([0.1, 0.2, 0.1, 0.6])
positions = np.array([1.0, 1.5, 2.0, 2.3])
cumulative_weights = np.cumsum(weights)
print(cumulative_weights)
# Generate random numbers
random_values = np.array([0.21,0.46,0.71,0.96])
colors = ['blue', 'green', 'red', 'purple']
# Create the plot
fig, ax = plt.subplots()
ax.set_ylim([0, 0.3])
start = 0.0
for i in range(len(weights)):

    ax.barh(y=0.0, width=weights[i], left=start, height=0.1, color=colors[i], edgecolor='black', alpha=0.5)
    start += weights[i]
    ax.scatter(x=random_values[i], y=0.025, color='black')
# Set labels and title
ax.set_xlabel('Cumulative Weight')
ax.set_yticks([])  # Remove y-axis as it's not needed for this visualization
ax.set_title('Visual Representation of Cumulative Weights')

# Show the plot
plt.show()