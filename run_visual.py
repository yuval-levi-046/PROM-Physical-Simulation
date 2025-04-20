from simulation.core import System, Car
from simulation.controller import TrafficController
from simulation.data_logger import DataLogger
from simulation.drivers import RandomDriverModel
from simulation.spawners import timed_spawner
from visuals.simulation import run_visual_simulation
from simulation.core import Road, Car, Obstacle

dt = 0.1
sim_time = 300

system = System(dt=dt, final_time=sim_time)

start_pos = (0, 300)
end_pos = (3000, 300)
road = Road(5, 60, start_pos, end_pos)

system.add_road(road)
logger = DataLogger(expected_total_cars=10000, tag="basic_one_road_test")
controller = TrafficController(system, logger=logger)

controller.add_spawn_rule(timed_spawner(0.3, 0, 100, 0, "aggressive", speed=0))
controller.add_spawn_rule(timed_spawner(0.3, 0, 100, 1, "aggressive", speed=5))
controller.add_spawn_rule(timed_spawner(0.3, 0, 100, 2, speed=0))
controller.add_spawn_rule(timed_spawner(0.3, 0, 100, 3, speed=5))
controller.add_spawn_rule(timed_spawner(0.3, 0, 100, 4, speed=0))

system.block_lane_from_offsets(0, 0, 500, 700)
system.block_lane_from_offsets(0, 1, 550, 650)




# === Run the simulation ===
run_visual_simulation(system, controller, logger)
