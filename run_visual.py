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

num_lanes = 1

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
controller = TrafficController(system)

logger.enable_sensor_logger(sensor_location=500, aggregation_window=30)
controller.add_exit_gate(1600)

controller.add_spawn_rule(density_random_lane_spawner(2500, 0, 200, "calibration", 0))

road.set_next_road(road)

# === Run the simulation ===
run_visual_simulation(system, controller, logger)
