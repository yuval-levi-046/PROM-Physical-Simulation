import numpy as np
from simulation.core import Road
from config import LANE_WIDTH
 
def create_circular_roads(n_roads, n_lanes, radius, center=(400, 300), lane_width=30, max_speed=80):
    roads = []
    angle_step = 2 * np.pi / n_roads

    # Step 1: Create roads
    for i in range(n_roads):
        theta1 = i * angle_step
        theta2 = (i + 1) % n_roads * angle_step

        start = np.array(center) + radius * np.array([np.cos(theta1), np.sin(theta1)])
        end = np.array(center) + radius * np.array([np.cos(theta2), np.sin(theta2)])

        road = Road(num_lanes=n_lanes, max_speed=max_speed, start_pos=start, end_pos=end, lane_width=lane_width)
        roads.append(road)

    # Step 2: Link next_road and next_lane
    for i, road in enumerate(roads):
        next_road = roads[(i + 1) % len(roads)]
        road.next_road = next_road

        for j, lane in enumerate(road.lanes):
            try:
                lane.next_lane = next_road.lanes[j]
            except IndexError:
                lane.next_lane = next_road.lanes[0]  # fallback if lane index doesn't exist

    return roads