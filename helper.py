import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

def get_last_frame_average_velocity(df, num_frames = 5):
    """Returns the average velocity at the final logged timestep."""
    if df.empty:
        print("No data logged.")
        return None

    last_times = sorted(df["time"].unique())[-num_frames:]
    recent_df = df[df["time"].isin(last_times)]

    if recent_df.empty:
        print("No data found in the last frames.")
        return None

    return recent_df["velocity"].mean()


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
