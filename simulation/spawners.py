from simulation.core import Car
import numpy as np
import random

def safe_add_car(lane, car, time):
    if lane.cars:
        front_car = lane.cars[0]  # car at the front of the lane
        spacing = front_car.offset - car.offset - front_car.length

        if spacing < car.length + 1:
            return False
        
        car.max_speed = lane.max_speed
        car.next_car = front_car
        car.lane = lane

        acc = car.driver_model.compute_idm_acceleration(car, front_car)

        if acc < car.driver_model.safety_constraint:
            return False
        
    lane.add_car(car)
    print(f"Inserting car with index:  {str(car.id)} at time: {time}")
    return True

def timed_spawner(interval, road_index, num_cars, lane_index=None, driver_type="basic", speed=0):

    timer = [0]
    local_num_cars = [0]
    def rule(dt, system, index):

        if local_num_cars[0] >= num_cars:
            return False
        
        timer[0] += dt
        if timer[0] >= interval:
            road = system.roads[road_index]
            chosen_lane_index = lane_index if lane_index is not None else random.randint(0, road.num_lanes - 1)
            lane = road.lanes[chosen_lane_index]
            car = Car(index, speed=speed, driver_type=driver_type)

            if safe_add_car(lane, car, system.time):
                local_num_cars[0] += 1
                timer[0] = 0
                system.index += 1
                return True

        return False

    return rule


def random_interval_spawner(min_interval, max_interval, road_index, num_cars, lane_index=None, driver_type="basic", speed=0):
    timer = [0]
    local_num_cars = [0]
    next_interval = [random.uniform(min_interval, max_interval)]

    def rule(dt, system, index):
        if local_num_cars[0] >= num_cars:
            return False

        timer[0] += dt
        if timer[0] >= next_interval[0]:
            road = system.roads[road_index]
            chosen_lane_index = lane_index if lane_index is not None else random.randint(0, road.num_lanes - 1)
            lane = road.lanes[chosen_lane_index]
            car = Car(index, speed=speed, driver_type=driver_type)

            if safe_add_car(lane, car, system.time):
                local_num_cars[0] += 1
                timer[0] = 0
                next_interval[0] = random.uniform(min_interval, max_interval)
                return True

        return False

    return rule

def density_based_spawner(min_gap, road_index, num_cars, lane_index=None, driver_type="basic", speed=0):
    local_num_cars = [0]

    def rule(dt, system, index):
        if local_num_cars[0] >= num_cars:
            return False

        road = system.roads[road_index]
        chosen_lane_index = lane_index if lane_index is not None else random.randint(0, road.num_lanes - 1)
        lane = road.lanes[chosen_lane_index]

        if not lane.cars or lane.cars[0].offset > min_gap:
            car = Car(index, speed=0, driver_type=driver_type)
            if safe_add_car(lane, car, system.time):
                local_num_cars[0] += 1
                return True

        return False

    return rule





