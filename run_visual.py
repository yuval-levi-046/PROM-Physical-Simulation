from simulation.core import System, Car
from simulation.controller import TrafficController
from simulation.data_logger import DataLogger
from simulation.drivers import RandomDriverModel
from simulation.spawners import timed_spawner
from visuals.simulation import run_visual_simulation
from simulation.core import Road, Car, Obstacle

dt = 0.1
sim_time = 200
system = System(dt=dt, final_time=sim_time)

# === Create Main Road ===
start_pos = (0, 300)
end_pos = (800, 300)
main_road = Road(num_lanes=4, max_speed=60, start_pos=start_pos, end_pos=end_pos)

# === Add Roads to System ===
system.add_road(main_road)

main_road.set_next_road(main_road)

# === Add Logger + Controller ===
logger = DataLogger(expected_total_cars=100, tag="merge_test")
controller = TrafficController(system, logger=logger)

# # Spawn cars on the merge road that try to enter lane 0

system.gaussian_spread_car_creator(1, speed=0, lane_index=3, road_index=0)
# system.gaussian_spread_car_creator(10, speed=0, lane_index=2, road_index=0)
# system.gaussian_spread_car_creator(10, speed=0, lane_index=1, road_index=0)
# system.gaussian_spread_car_creator(10, speed=0, lane_index=0, road_index=0)


# === Run the simulation ===
run_visual_simulation(system, controller, logger)
