from simulation.core import Car
import random

def safe_add_car(lane, car):
    if lane.cars:
        if abs(lane.cars[0].offset) < car.threshold + car.length:
            return False  
    lane.add_car(car)
    print(f"adding car with index: " + str(car.id))
    return True

def timed_spawner(interval, road_index, num_cars):
    timer = [0]
    def rule(dt, system, index):

        if index >= num_cars:
            return False
        
        timer[0] += dt
        if timer[0] >= interval:
            timer[0] = 0
            road = system.roads[road_index]
            lane_index = random.randint(0, road.num_lanes - 1)
            lane = road.lanes[lane_index]
            car = Car(index, lane)

            return safe_add_car(lane, car)


    return rule

# def load_based_spawner(road_index, lane_index, max_cars=5, cooldown=2.0):
#     timer = [0]
#     def rule(dt, system):
#         timer[0] += dt
#         lane = system.roads[road_index].lanes[lane_index]
#         if timer[0] >= cooldown and len(lane.cars) < max_cars:
#             car = Car(id, lane, offset=0, speed=10, next_car=None, driver_profile="basic")
#             safe_add_car(lane, car)
#             timer[0] = 0
#     return rule

# def probabilistic_spawner(prob, road_index, lane_index):
#     def rule(dt, system):
#         if random.random() < prob:
#             lane = system.roads[road_index].lanes[lane_index]
#             car = Car(id, lane, offset=0, speed=10, next_car=None, driver_profile="basic")
#             safe_add_car(lane, car)
#     return rule
