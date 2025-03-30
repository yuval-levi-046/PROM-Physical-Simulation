# simulation/core.py
import numpy as np
from globals.driver_profiles import BASIC_DRIVER, DRIVER_PROFILES
from config import CAR_COLOR, CAR_WIDTH, CAR_LENGTH

class Car:
    def __init__(self, lane, offset, speed, next_car, driver_profile):
        self.lane = lane
        self.offset = offset  # position along the lane
        self.velocity_magnitude = speed  # scalar speed
        self.next_car = next_car  # car in front

        self.driver_profile = driver_profile
        driver_type = DRIVER_PROFILES[driver_profile]

        self.max_speed = driver_type["max_speed"]
        self.threshold = driver_type["threshold"] # minimum position to maintain from the car infront
        self.acc = driver_type["acceleration"]        
        self.length = CAR_LENGTH
        self.stopped = False  # for collision handling

    def update(self, dt):
        if self.stopped:
            self.decelerate_to_stop(dt)
            self.move(dt)
            return

        if self.next_car:
            spacing = self.next_car.offset - self.offset - self.length
            if spacing <= 0:
                self.stopped = False
                self.next_car.stopped = True
                return
            elif spacing < self.threshold:
                self.decelerate(dt)
            else:
                self.accelerate(dt)
        else:
            self.accelerate(dt)

        if self.velocity_magnitude > self.max_speed:
            self.decelerate(dt)

        self.move(dt)

    def move(self, dt):
        self.offset += self.velocity_magnitude * dt

    def accelerate(self, dt):
        self.velocity_magnitude += self.acc * dt
        self.velocity_magnitude = min(self.velocity_magnitude, self.max_speed)

    def decelerate(self, dt):
        self.velocity_magnitude -= self.acc * dt
        self.velocity_magnitude = max(self.velocity_magnitude, 0)

    def decelerate_to_stop(self, dt):
        self.velocity_magnitude -= self.acc * dt * 2  # stronger deceleration on collision
        self.velocity_magnitude = max(self.velocity_magnitude, 0)


class Lane:
    def __init__(self, start_pos, end_pos, max_speed):
        self.start_pos = np.array(start_pos, dtype=float)
        self.end_pos = np.array(end_pos, dtype=float)
        self.cars = []
        self.max_speed = max_speed
        self.length = np.linalg.norm(self.end_pos - self.start_pos)
        self.next_lane = None
        self.unit_direction = (self.end_pos - self.start_pos) / self.length

    def add_car(self, car):
        if self.cars:
            self.cars[-1].next_car = car
        self.cars.append(car)

    def update(self, dt):
        self.cars.sort(key=lambda c: c.offset)

        # Re-assign next_car based on order
        for i in range(len(self.cars) - 1):
            self.cars[i].next_car = self.cars[i + 1]
        if self.cars:
            self.cars[-1].next_car = None  # front-most car

        # Update all cars
        for car in self.cars:
            car.update(dt)


class Road:
    def __init__(self, num_lanes, max_speed, start_pos, end_pos, lane_width):
        self.lanes = []
        self.num_lanes = num_lanes
        self.start_pos = np.array(start_pos, dtype=float)
        self.end_pos = np.array(end_pos, dtype=float)
        self.length = np.linalg.norm(self.end_pos - self.start_pos)
        self.next_road = None
        self.unit_direction = (self.end_pos - self.start_pos) / self.length
        normal = np.array([-self.unit_direction[1], self.unit_direction[0]])

        for i in range(num_lanes):
            offset = (i - (num_lanes - 1) / 2) * lane_width
            offset_vector = normal * offset
            lane_start = self.start_pos + offset_vector
            lane_end = self.end_pos + offset_vector
            lane = Lane(lane_start, lane_end, max_speed)

            lane.road = self
            self.lanes.append(lane)

    def update(self, dt):
        for lane in self.lanes:
            lane.update(dt)

    def get_all_cars(self):
        return [car for lane in self.lanes for car in lane.cars]


class System:
    def __init__(self, dt=0.01, final_time=100):
        self.roads = []
        self.dt = dt
        self.final_time = final_time
        self.time = 0

    def add_road(self, road):
        self.roads.append(road)

    def update(self, logger = None):
        for road in self.roads:
            road.update(self.dt)
        self.time += self.dt

        if logger:
            logger.log(self)
