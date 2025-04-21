from simulation.core import System, Car
from simulation.controller import TrafficController
from simulation.data_logger import DataLogger
from simulation.drivers import RandomDriverModel
from simulation.spawners import timed_spawner, density_random_lane_spawner
from visuals.simulation import run_visual_simulation
from simulation.core import Road, Car, Obstacle

dt = 0.1
sim_time = 400

num_cars = 100
starting_speed = 60

road_length = 2000
max_speed = 60

num_lanes = 5

system = System(dt=dt, final_time=sim_time)

start_pos = (0, 300)
end_pos = (road_length, 300)
road = Road(num_lanes, max_speed, start_pos, end_pos)

system.add_road(road)
logger = DataLogger(tag="basic_one_road_test")
controller = TrafficController(system, logger=logger)

controller.add_spawn_rule(density_random_lane_spawner(0.1, 0, num_cars, speed=starting_speed))

system.block_lane_from_offsets(0, 0, 500, 700)
system.block_lane_from_offsets(0, 4, 350, 450)
# === Run the simulation ===
run_visual_simulation(system, controller, logger)
