import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
from scipy.signal import find_peaks
from scipy.stats import linregress

def get_average_velocity(df, start_percent = 0.0, end_percent = 1.0):
    if df.empty or not 0 <= start_percent < end_percent <= 1:
        print("Invalid input or empty DataFrame.")
        return None

    min_time, max_time = df["time"].min(), df["time"].max()
    start_time = min_time + (max_time - min_time) * start_percent
    end_time = min_time + (max_time - min_time) * end_percent

    filtered_df = df[(df["time"] >= start_time) & (df["time"] <= end_time)]

    if filtered_df.empty:
        print("No data found in the specified time range.")
        return None

    return filtered_df["velocity"].mean()

def plot_stuck_car_heatmap(df, velocity_threshold=1, bin_size=10):
 
    stuck_df = df[df["velocity"] < velocity_threshold].copy()
    stuck_df["offset_bin"] = (stuck_df["offset"] // bin_size) * bin_size

    if stuck_df.empty:
        return

    heatmap_data = stuck_df.groupby(["lane_id", "offset_bin"]).size().unstack(fill_value=0)

    plt.figure(figsize=(14, 6))
    sns.heatmap(
        heatmap_data,
        cmap="magma",
        cbar_kws={'label': 'Number of Stuck Cars'},
        linewidths=0.3,
        linecolor='gray'
    )
    plt.title("🚗 Heatmap of Stuck Cars by Lane and Offset", fontsize=16)
    plt.xlabel(f"Offset Position (binned every {bin_size} px)", fontsize=12)
    plt.ylabel("Lane ID", fontsize=12)
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.show()


def compute_avg_velocity_over_time(df, N_total=None, plot=False):
    """Computes and optionally plots average velocity over time."""
    grouped = df.groupby("time").agg(
        total_velocity=("velocity", "sum"),
        active_cars=("car_id", "nunique")
    ).reset_index()

    grouped["real_avg_velocity"] = grouped["total_velocity"] / grouped["active_cars"]

    if N_total:
        grouped["padded_avg_velocity"] = grouped["total_velocity"] / N_total

    if plot:
        plt.figure(figsize=(10, 4))
        if N_total:
            plt.plot(grouped["time"], grouped["padded_avg_velocity"], label="Padded Avg Velocity", color='green')
        plt.plot(grouped["time"], grouped["real_avg_velocity"], label="Real Avg Velocity", color='red')
        plt.xlabel("Time (s)")
        plt.ylabel("Average Velocity")
        plt.title("Average Velocity Over Time")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

    return grouped


def plot_velocity_distribution(df, bins=30):
    """Plots a histogram of all recorded velocities."""
    plt.figure(figsize=(8, 4))
    plt.hist(df["velocity"], bins=bins, color="skyblue", edgecolor="black")
    plt.xlabel("Velocity (px/s)")
    plt.ylabel("Frequency")
    plt.title("Velocity Distribution")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_lane_occupancy(df):
    """Plots car count per lane over time."""
    lane_counts = df.groupby(["time", "lane_id"])["car_id"].nunique().unstack(fill_value=0)
    plt.figure(figsize=(12, 6))
    for lane in lane_counts.columns:
        plt.plot(lane_counts.index, lane_counts[lane], label=f"Lane {lane}")
    plt.xlabel("Time (s)")
    plt.ylabel("Cars per Lane")
    plt.title("Lane Occupancy Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


import matplotlib.pyplot as plt

def plot_average_velocity_per_lane(df):

    avg_velocity_lane = df.groupby(["time", "lane_id"])["velocity"].mean().unstack(fill_value=0)

    plt.figure(figsize=(12, 5))
    for lane in avg_velocity_lane.columns:
        plt.plot(avg_velocity_lane.index, avg_velocity_lane[lane], label=f"Lane {lane}")
    plt.xlabel("Time (s)")
    plt.ylabel("Average Velocity (px/s)")
    plt.title("Average Velocity per Lane Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()



def plot_lane_traffic_stats(df):
    # Calculate number of unique cars per lane over time
    lane_counts = df.groupby(["time", "lane_id"])["car_id"].nunique().unstack(fill_value=0)

    # Compute percentage share per lane
    lane_percentages = lane_counts.div(lane_counts.sum(axis=1), axis=0) * 100

    # === Plot 1: Line plot of traffic share percentages ===
    plt.figure(figsize=(12, 5))
    for lane in lane_percentages.columns:
        plt.plot(lane_percentages.index, lane_percentages[lane], label=f"Lane {lane}")
    plt.xlabel("Time (s)")
    plt.ylabel("Traffic Share (%)")
    plt.title("Percentage of Traffic in Each Lane Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # === Plot 2: Heatmap of raw lane occupancy ===
    plt.figure(figsize=(10, 6))
    sns.heatmap(lane_counts.T, cmap="YlGnBu", cbar_kws={"label": "Car Count"})
    plt.title("Density Heatmap: Cars per Lane per Time")
    plt.xlabel("Time Step")
    plt.ylabel("Lane ID")
    plt.tight_layout()
    plt.show()

    import matplotlib.pyplot as plt

def plot_graphs(df, N_total=100):
    """
    Combines multiple traffic plots into a 2x2 subplot layout, ending with a Seaborn heatmap.
    """
    # Prepare aggregated data
    grouped = df.groupby("time").agg(
        total_velocity=("velocity", "sum"),
        active_cars=("car_id", "nunique")
    ).reset_index()
    grouped["real_avg_velocity"] = grouped["total_velocity"] / grouped["active_cars"]
    grouped["padded_avg_velocity"] = grouped["total_velocity"] / N_total

    lane_counts = df.groupby(["time", "lane_id"])["car_id"].nunique().unstack(fill_value=0)
    avg_velocity_lane = df.groupby(["time", "lane_id"])["velocity"].mean().unstack(fill_value=0)

    # Create subplots
    fig, axs = plt.subplots(2, 2, figsize=(16, 10))
    axs = axs.flatten()

    # Plot 1: Average Velocity Over Time
    axs[0].plot(grouped["time"], grouped["real_avg_velocity"], label="Real Avg Velocity", color='red')
    axs[0].set_title("Average Velocity Over Time")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Velocity")
    axs[0].legend()
    axs[0].grid(True)

    # Plot 2: Lane Occupancy
    for lane in lane_counts.columns:
        axs[1].plot(lane_counts.index, lane_counts[lane], label=f"Lane {lane}")
    axs[1].set_title("Lane Occupancy Over Time")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Cars per Lane")
    axs[1].legend()
    axs[1].grid(True)

    # Plot 3: Average Velocity Per Lane
    for lane in avg_velocity_lane.columns:
        axs[2].plot(avg_velocity_lane.index, avg_velocity_lane[lane], label=f"Lane {lane}")
    axs[2].set_title("Avg Velocity per Lane")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Velocity")
    axs[2].legend()
    axs[2].grid(True)

    # Plot 4: Heatmap of Cars per Lane per Time
    plt.sca(axs[3])  # Set current axis for seaborn heatmap
    sns.heatmap(lane_counts.T, cmap="YlGnBu", cbar_kws={"label": "Car Count"})
    axs[3].set_title("Density Heatmap: Cars per Lane per Time")
    axs[3].set_xlabel("Time Step")
    axs[3].set_ylabel("Lane ID")

    plt.tight_layout()
    plt.show()


def plot_density_by_space(df, road_length):

    num_time_bins = int(road_length / 6.5)
    num_space_bins = int(road_length / 6.5)

    # Prepare bins
    time_bins = np.linspace(df['time'].min(), df['time'].max(), num_time_bins)
    space_bins = np.linspace(0, road_length, num_space_bins)
    # Compute 2D histogram for density
    density, xedges, yedges = np.histogram2d(df['offset'], df['time'], bins=[space_bins, time_bins])

    vmin = np.percentile(density, 5)   # Lower 5% cutoff
    vmax = np.percentile(density, 95)

    plt.figure(figsize=(10, 6))
    img = plt.imshow(
        density.T, 
        aspect='auto', 
        origin='lower', 
        extent=[space_bins[0], space_bins[-1], time_bins[0], time_bins[-1]],
        cmap='plasma',
        vmin=vmin,
        vmax=vmax  # Clipping here works!
    )
    plt.colorbar(img, label='Car Density')  # Attach colorbar to the image object!
    plt.title('Car Density as a Function of Position and Time')

    plt.xlabel('Position on Road')
    plt.ylabel('Time')
    plt.tight_layout()
    plt.show()



def calculate_wave_speed_by_peak_tracking(density, dt, dx):

    tracked_peaks = []

    for t, row in enumerate(density):
        peaks, _ = find_peaks(row, height=np.percentile(row, 80))  # Adjust threshold as needed
        for p in peaks:
            tracked_peaks.append((t * dt, p * dx))  # time in seconds, position in meters

    # Convert to arrays for fitting
    times = np.array([tp[0] for tp in tracked_peaks])
    positions = np.array([tp[1] for tp in tracked_peaks])

    # Linear regression (positions vs. time)
    slope, intercept, r_value, p_value, std_err = linregress(times, positions)
    print(f"Estimated wave speed: {slope:.2f} m/s")

    return slope


