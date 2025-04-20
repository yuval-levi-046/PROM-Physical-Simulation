from simulation.controller import TrafficController
from simulation.data_logger import DataLogger
from simulation.spawners import timed_spawner, density_random_lane_spawner
from visuals.simulation import run_visual_simulation
from simulation.core import Road, System

dt = 0.1
sim_time = 300

system = System(dt=dt, final_time=sim_time)

start_pos = (0, 300)
end_pos = (3000, 300)
road = Road(5, 60, start_pos, end_pos)

system.add_road(road)
logger = DataLogger(expected_total_cars=10000, tag="basic_one_road_test")
controller = TrafficController(system, logger=logger)

controller.add_spawn_rule(density_random_lane_spawner(5, 0, 200, speed=50))

system.block_lane_from_offsets(0, 0, 2500, 2700)
system.block_lane_from_offsets(0, 1, 2550, 2650)


# === Run the simulation ===
run_visual_simulation(system, controller, logger)
