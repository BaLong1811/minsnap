# import matplotlib.pyplot as plt
# import pandas as pandas

# # Load data from CSV file
# filename = '/home/balong18/aerial_robot/minsnap_test/vel_values.csv'
# data = pandas.read_csv(filename, header=None, names=['x', 'y', 'z'])

# # Plot each parameter
# plt.figure(figsize=(10, 6))

# # Plot x parameter
# plt.subplot(3, 1, 1)
# plt.plot(data['x'], label='x', marker='o')
# plt.ylabel('X Values')
# plt.legend()

# # Plot y parameter
# plt.subplot(3, 1, 2)
# plt.plot(data['y'], label='y', marker='o', color='orange')
# plt.ylabel('Y Values')
# plt.legend()

# # Plot z parameter
# plt.subplot(3, 1, 3)
# plt.plot(data['z'], label='z', marker='o', color='green')
# plt.xlabel('Index')
# plt.ylabel('Z Values')
# plt.legend()

# # Adjust layout and show plot
# plt.tight_layout()
# plt.show()

import matplotlib.pyplot as plt
import pandas as pd

# Function to plot data from a CSV file
def plot_csv(ax, filename, label_prefix):
    data = pd.read_csv(filename, header=None, names=['x', 'y', 'z'])
    ax[0].plot(data['x'], label=f'{label_prefix} x', marker='o')
    ax[0].set_ylabel('X Values')
    ax[0].legend()

    ax[1].plot(data['y'], label=f'{label_prefix} y', marker='o', color='orange')
    ax[1].set_ylabel('Y Values')
    ax[1].legend()

    ax[2].plot(data['z'], label=f'{label_prefix} z', marker='o', color='green')
    ax[2].set_xlabel('Index')
    ax[2].set_ylabel('Z Values')
    ax[2].legend()

# Create three separate plots with three subplots each
fig1, ax1 = plt.subplots(3, 1, figsize=(10, 10))
fig2, ax2 = plt.subplots(3, 1, figsize=(10, 10))
fig3, ax3 = plt.subplots(3, 1, figsize=(10, 10))

plot_csv(ax1, '/home/balong18/aerial_robot/minsnap_test/position_values.csv', 'Data1')
fig1.suptitle('Position Plots')
fig1.tight_layout()

plot_csv(ax2, '/home/balong18/aerial_robot/minsnap_test/vel_values.csv', 'Data2')
fig2.suptitle('Velocity Plots')
fig2.tight_layout()

plot_csv(ax3, '/home/balong18/aerial_robot/minsnap_test/acc_values.csv', 'Data3')
fig3.suptitle('Acceleration Plots')
fig3.tight_layout()

plt.show()


