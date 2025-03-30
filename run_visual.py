from simulation.core import System, Car
from simulation.helpers import create_circular_roads
from simulation.controller import TrafficController
from simulation.data_logger import DataLogger
from simulation.spawners import timed_spawner
from visuals.simulation import run_visual_simulation
from simulation.core import Road

dt = 0.01
sim_time = 30

system = System(dt=dt, final_time=sim_time)

start_pos = (100, 300)
end_pos = (700, 300)
road = Road(num_lanes=1, max_speed=80, start_pos=start_pos, end_pos=end_pos, lane_width=30)

system.add_road(road)

logger = DataLogger(tag="basic_one_road_test")
controller = TrafficController(system, logger=logger)

controller.add_spawn_rule(timed_spawner(interval=1.5, road_index=0))

# === Run the visual simulation ===
run_visual_simulation(system, controller, logger)
