from simulation.core import System, Car
from simulation.controller import TrafficController
from simulation.data_logger import DataLogger
from simulation.spawners import timed_spawner
from visuals.simulation import run_visual_simulation
from simulation.core import Road, Car

dt = 0.05
sim_time = 100

system = System(dt=dt, final_time=sim_time)

start_pos = (0, 300)
end_pos = (800, 300)
road = Road(2, 80, start_pos, end_pos)
obstacle1 = Car(-2, road.lanes[0], 780)
obstacle1.is_obstacle = True
obstacle2 = Car(-1, road.lanes[1], 780)
obstacle2.is_obstacle = True

system.add_road(road)
road.lanes[0].add_car(obstacle1)
road.lanes[1].add_car(obstacle2)

logger = DataLogger(tag="basic_one_road_test")
controller = TrafficController(system, logger=logger)

controller.add_spawn_rule(timed_spawner(1, 0, 20, 1))



# === Run the visual simulation ===
run_visual_simulation(system, controller, logger)
