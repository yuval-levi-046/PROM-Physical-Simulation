from simulation.core import System, Car
from simulation.helpers import create_circular_roads
from simulation.controller import TrafficController
from simulation.data_logger import DataLogger
from simulation.spawners import timed_spawner
from visuals.simulation import run_visual_simulation
from simulation.core import Road, Car

dt = 0.1
sim_time = 100

system = System(dt=dt, final_time=sim_time)

start_pos = (100, 300)
end_pos = (700, 300)
roads = create_circular_roads(10, 1, 300, max_speed=200)

for road in roads:
    system.add_road(road)

logger = DataLogger(tag="basic_one_road_test")
controller = TrafficController(system, logger=logger)

controller.equal_distance_car_creator(20, lane_index=0)






# === Run the visual simulation ===
run_visual_simulation(system, controller, logger)
