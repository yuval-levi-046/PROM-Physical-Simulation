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
end_pos = (800, 300)
road = Road(5, 60, start_pos, end_pos)

system.add_road(road)
logger = DataLogger(expected_total_cars=10000, tag="basic_one_road_test")
controller = TrafficController(system, logger=logger)

# obstacle = Obstacle(-1, 500, 30)
# road.lanes[0].add_car(obstacle)

road.set_next_road(road)

system.gaussian_spread_car_creator(40, "random", 0, 0)



# === Run the visual simulation ===
run_visual_simulation(system, controller, logger)
