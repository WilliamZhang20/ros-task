import matplotlib.pyplot as plt
import numpy as np

def plot_xy(filename="workspace/data.txt", output_filename="plots/plot-position.png"):
    try:
        data = np.loadtxt(filename)
        x = data[:, 0]
        y = data[:, 1]

        plt.figure(figsize=(10, 6))
        plt.plot(x, y, marker='o', linestyle='-', markersize=3)
        plt.title("Plot of Position")
        plt.xlabel("X coordinate")
        plt.ylabel("Y coordinate")
        plt.grid(True)
        plt.savefig(output_filename)
        plt.show()
        print(f"Plot saved to {output_filename}")

    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

def plot_z_time(filename="workspace/data.txt", output_filename="plots/plot-yaw.png"):
    try:
        data = np.loadtxt(filename)
        z = data[:, 2]  # Third column
        num_points = len(z)
        time = np.linspace(0, num_points / 30, num_points)  # 30 points per second

        plt.figure(figsize=(10, 6))
        plt.plot(time, z, marker='o', linestyle='-', markersize=3)
        plt.title("Plot of Yaw vs. Time")
        plt.xlabel("Time")
        plt.ylabel("Yaw Value (radians)")
        plt.grid(True)

        rightmost_time = time[-1]
        rightmost_z = z[-1]
        plt.annotate(f"{rightmost_z:.4f}",  # Format z-value
                     xy=(rightmost_time, rightmost_z),
                     xytext=(rightmost_time, rightmost_z - 0.3),  # Adjust bubble position
                     arrowprops=dict(facecolor='black', arrowstyle='->'),
                     bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.5))

        plt.savefig(output_filename)
        plt.show()
        print(f"Plot saved to {output_filename}")

    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

def plot_w_time(filename="workspace/data.txt", output_filename="plots/plot-distance.png"):
    try:
        data = np.loadtxt(filename)
        w = data[:, 3]  # Fourth column
        num_points = len(w)
        time = np.linspace(0, num_points / 30, num_points)  # 30 points per second

        plt.figure(figsize=(10, 6))
        plt.plot(time, w, marker='o', linestyle='-', markersize=3)
        plt.title("Plot of Distance vs. Time")
        plt.xlabel("Time")
        plt.ylabel("Distance (meters)")
        plt.grid(True)

        rightmost_time = time[-1]
        rightmost_w = w[-1]
        plt.annotate(f"{rightmost_w:.4f}",  # Format w-value
                     xy=(rightmost_time, rightmost_w),
                     xytext=(rightmost_time, rightmost_w - 0.3),  # Adjust bubble position
                     arrowprops=dict(facecolor='red', arrowstyle='->'),
                     bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.5))

        plt.savefig(output_filename)
        plt.show()
        print(f"Plot saved to {output_filename}")

    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage:
plot_xy()
plot_z_time()
plot_w_time()