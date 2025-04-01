from simulation.core import System, Car
from simulation.helpers import create_circular_roads
from simulation.controller import TrafficController
from simulation.data_logger import DataLogger
from simulation.spawners import timed_spawner
from visuals.simulation import run_visual_simulation
from simulation.core import Road, Car

dt = 0.01
sim_time = 100

system = System(dt=dt, final_time=sim_time)

start_pos = (100, 300)
end_pos = (700, 300)
roads = create_circular_roads(10, 1, 200, max_speed=50)

for road in roads:
    system.add_road(road)

logger = DataLogger(tag="basic_one_road_test")
controller = TrafficController(system, logger=logger)

controller.add_spawn_rule(timed_spawner(interval=1.5, road_index=0, num_cars = 10))
controller.add_slow_car(10,5,0)



# === Run the visual simulation ===
run_visual_simulation(system, controller, logger)
