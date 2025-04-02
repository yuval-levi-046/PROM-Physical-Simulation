# simulation/core.py
import numpy as np
from globals.driver_profiles import BASIC_DRIVER, DRIVER_PROFILES
from config import CAR_COLOR, CAR_WIDTH, CAR_LENGTH, LANE_WIDTH
import random

class Car:
    def __init__(self, id, lane, offset = 0, speed = 0, next_car = None, driver_profile = "basic"): # MAKE CLASS OBSTACLE
        self.is_obstacle = False
        self.is_slowed = False
        self.timers = {
            "slowness_timer": 0
        }
        self.id = id

        self.probability = 0.05

        self.lane = lane
        self.offset = offset  # position along the lane
        self.velocity_magnitude = speed  # scalar speed
        self.next_car = next_car  # car in front

        self.driver_profile = driver_profile
        driver_type = DRIVER_PROFILES[driver_profile]

        self.max_speed = driver_type["max_speed"]
        self.threshold = driver_type["threshold"] # minimum position to maintain from the car infront
        self.acc = driver_type["acceleration"] 
        self.safety_threshold = 0   
           
        self.length = CAR_LENGTH
        self.stopped = False  # for collision handling

    def should_switch_right(self, neighbours): #Neighbour[0] is the one before you
        if neighbours == []: return 0

        threshold = self.safety_threshold + self.length
        
        difference_1 = abs(neighbours[0].offset - self.offset) if neighbours[0] else threshold

        difference_2 = abs(neighbours[1].offset - self.offset) if neighbours[1] else threshold

        if difference_1 >= threshold and difference_2 >= threshold:
            return self.probability
        
        return 0

    def should_switch_left(self, neighbours):
        if not neighbours: return 0

        threshold = self.safety_threshold + self.length
        
        difference_1 = abs(neighbours[0].offset - self.offset) if neighbours[0] else threshold

        difference_2 = abs(neighbours[1].offset - self.offset) if neighbours[1] else threshold

        if difference_1 >= threshold and difference_2 >= threshold:
            return self.probability
        
        return 0

    def update(self, dt):
        self.safety_threshold = self.velocity_magnitude ** 2 / (2 * self.acc) + self.threshold

        if self.is_obstacle:
            return
                
        if self.is_slowed:
            if self.timers["slowness_timer"] > 0:
                self.decelerate(dt)
                self.timers["slowness_timer"] = max(0, self.timers["slowness_timer"] - dt)
            else:
                self.is_slowed = False
            return
        
        if self.velocity_magnitude > self.max_speed:
            self.decelerate(dt)
            return

        if self.next_car: 
            if self.lane == self.next_car.lane:
                spacing = self.next_car.offset - self.offset - self.length 
            
            elif self.lane.next_lane == self.next_car.lane:
                spacing = self.lane.length - self.offset + self.next_car.offset - self.length 

            else:
                spacing = self.safety_threshold + 1
            
            if spacing <= 0:
                raise("Car has crashed")
            
            elif spacing < self.safety_threshold:
                self.decelerate(dt)
                return
            else:
                self.accelerate(dt)
                return
            
        self.accelerate(dt)

    def move(self, dt):
        self.offset += self.velocity_magnitude * dt

    def accelerate(self, dt):
        self.velocity_magnitude += self.acc * dt
        self.velocity_magnitude = min(self.velocity_magnitude, self.lane.max_speed)
        self.move(dt)

    def decelerate(self, dt):
        self.velocity_magnitude -= self.acc * dt
        self.velocity_magnitude = max(self.velocity_magnitude, 0)
        self.move(dt)

    def set_velocity(self, velocity):
        self.velocity_magnitude = velocity

    def apply_slowness(self, delta_time):
        self.timers["slowness_timer"] = delta_time
        self.is_slowed = True
        

class Lane:
    def __init__(self, id, start_pos, end_pos, max_speed):
        self.id = id
        self.start_pos = np.array(start_pos, dtype=float)
        self.end_pos = np.array(end_pos, dtype=float)
        self.cars = []
        self.max_speed = max_speed
        self.length = np.linalg.norm(self.end_pos - self.start_pos)
        self.next_lane = None
        self.unit_direction = (self.end_pos - self.start_pos) / self.length

    def add_car(self, car):
        self.cars.append(car)
        self.update_next_cars()

    def remove_car(self, car):
        self.cars.remove(car)
        self.update_next_cars()

    def update_next_cars(self):
        self.cars.sort(key=lambda c: c.offset) # ADD FOR LATER

        for i in range(len(self.cars) - 1):
            self.cars[i].next_car = self.cars[i + 1]

        if self.cars:
            if self.next_lane and self.next_lane.cars:
                self.cars[-1].next_car = self.next_lane.cars[0]

    def update(self, dt):
        # Update all cars
        for car in self.cars:
            car.update(dt)

    def update_next_lane(self, next_lane):
        self.next_lane = next_lane

    def find_car_by_offset(self, offset):
        before = None
        after = None

        for car in self.cars:
            if car.offset <= offset:
                before = car
            elif car.offset > offset:
                after = car
                break

        return before, after




class Road:
    def __init__(self, num_lanes, max_speed, start_pos, end_pos, lane_width=LANE_WIDTH):
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
            lane = Lane(i, lane_start, lane_end, max_speed)

            lane.road = self
            self.lanes.append(lane)

    def update(self, dt):
        for lane in self.lanes:
            lane.update(dt)

    def get_all_cars(self):
        return [car for lane in self.lanes for car in lane.cars]
    
    def get_neighbours(self, car):
        """
            Returns the list of neighbours to the right and then to the left 
            returns:
                arr_neightbours_right, arr_neightbours_left
        """
        lane_index = car.lane.id
        offset = car.offset
        if self.num_lanes == 1: return [], []
        if lane_index == 0 :
            return self.lanes[1].find_car_by_offset(offset), []
        
        if lane_index == self.num_lanes - 1:
            return [], self.lanes[lane_index - 1].find_car_by_offset(offset)
        
        return self.lanes[lane_index + 1].find_car_by_offset(offset), self.lanes[lane_index - 1].find_car_by_offset(offset)
        

    
    def update_next_lane(self, lane_idx, next_lane):
        self.lanes[lane_idx].update_next_lane(next_lane)

    def switch_lane(self, car, new_lane):
        car.lane.remove_car(car)
        car.lane = new_lane
        new_lane.add_car(car)




class System:
    def __init__(self, dt=0.01, final_time=100):
        self.roads = []
        self.dt = dt
        self.final_time = final_time
        self.total_length = 0
        self.time = 0

    def add_car(self, index, offset, speed, lane_index):        
        for road in self.roads:
            if offset > road.length:
                offset -= road.length
            else:
                car = Car(index, road.lanes[lane_index], offset=offset, speed=speed)
                road.lanes[lane_index].add_car(car)
                return

    def add_road(self, road):
        self.roads.append(road)
        self.total_length += road.length

    def update(self, logger = None):
        for road in self.roads:
            road.update(self.dt)
        self.time += self.dt

        if logger:
            logger.log(self)
