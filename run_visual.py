from simulation.core import System, Car
from simulation.controller import TrafficController
from simulation.data_logger import DataLogger
from simulation.drivers import RandomDriverModel
from simulation.spawners import timed_spawner, density_random_lane_spawner
from visuals.simulation import run_visual_simulation
from simulation.core import Road, Car, Obstacle

dt = 0.1
sim_time = 4000

road_length = 800
max_speed = 100

num_lanes = 2

density = 0.08

system = System(dt=dt, final_time=sim_time)

start_pos = (0, 300)
end_pos = (road_length, 300)
road = Road(num_lanes, max_speed, start_pos, end_pos)

system = System(dt=dt, final_time=sim_time)

start_pos = (0, 300)
end_pos = (road_length, 300)
road = Road(num_lanes, max_speed, start_pos, end_pos)

system.add_road(road)
logger = DataLogger(tag="mix_driver_density")
controller = TrafficController(system, logger=logger)

system.equal_distance_car_creator(int(road_length*density), road_index = 0, lane_index = 0, driver_type="mix", speed = 0)
system.equal_distance_car_creator(int(road_length*density), road_index = 0, lane_index = 1, driver_type="mix", speed = 0)

road.set_next_road(road)

# === Run the simulation ===
run_visual_simulation(system, controller, logger)
