from simulation.core import Car
import random

def safe_add_car(lane, car):
    # Check if there is enough space
    if lane.cars:
        if abs(lane.cars[0].offset) < car.threshold + car.length:
            return None  # too close to spawn, skip
    lane.add_car(car)
    return car

def timed_spawner(interval, road_index):
    timer = [0]
    def rule(dt, system):
        timer[0] += dt
        if timer[0] >= interval:
            timer[0] = 0
            road = system.roads[road_index]
            lane_index = random.randint(0, road.num_lanes - 1)
            lane = road.lanes[lane_index]
            car = Car(lane, offset=0, speed=10, next_car=None, driver_profile="basic")
            safe_add_car(lane, car,)
    return rule

def load_based_spawner(road_index, lane_index, max_cars=5, cooldown=2.0):
    timer = [0]
    def rule(dt, system):
        timer[0] += dt
        lane = system.roads[road_index].lanes[lane_index]
        if timer[0] >= cooldown and len(lane.cars) < max_cars:
            car = Car(lane, offset=0, speed=10, next_car=None, driver_profile="basic")
            safe_add_car(lane, car)
            timer[0] = 0
    return rule

def probabilistic_spawner(prob, road_index, lane_index):
    def rule(dt, system):
        if random.random() < prob:
            lane = system.roads[road_index].lanes[lane_index]
            car = Car(lane, offset=0, speed=10, next_car=None, driver_profile="basic")
            safe_add_car(lane, car)
    return rule
