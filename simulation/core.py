# simulation/core.py
import numpy as np
from globals.driver_profiles import BASIC_DRIVER, DRIVER_PROFILES
from config import CAR_COLOR, CAR_WIDTH, CAR_LENGTH, LANE_WIDTH
import random

class DriverModel:
     
    def __init__(self, profile):
        driver_type = DRIVER_PROFILES[profile]

        self.profile_name = profile
        self.max_speed = driver_type["max_speed"]
        self.threshold = driver_type["threshold"]
        self.idm_params = driver_type.get("idm_params", {})  # Optional override
        self.velocity_threshold = 5
        self.p = driver_type.get("politeness", 0.3)
        self.a_threshold = driver_type.get("lane_change_threshold", 0.2)
        self.bias_left = driver_type.get("bias_left", 0.2)
        self.bias_right = driver_type.get("bias_right", -0.2)
        self.safety_constraint = -4.0  # CHECK THIS SAFETY_CONSTRAINT
    

    def compute_idm_acceleration(self, car, leader=None):
        a = self.idm_params.get("a", 1.5)
        b = self.idm_params.get("b", 2.0)
        delta = self.idm_params.get("delta", 4)
        s0 = self.idm_params.get("s0", 2.0)
        T = self.idm_params.get("T", 1.5)

        v = car.velocity_magnitude
        v0 = car.desired_velocity

        if leader is None:
            leader = car.next_car

        if leader is None:
            s = float('inf')
            delta_v = 0
        else:
            s = leader.offset - car.offset - leader.length
            delta_v = v - leader.velocity_magnitude

        s_star = s0 + v * T + (v * delta_v) / (2 * (a * b)**0.5)
        acc = a * (1 - (v / v0)**delta - (s_star / s)**2)

        return acc
    
    def evaluate_lane_change(self, car, direction, road):

        if car.velocity_magnitude < self.velocity_threshold:
            return False

        current_lane = car.lane
        lane_index = current_lane.id

        if direction == "left" and lane_index == 0:
            return False
        if direction == "right" and lane_index == road.num_lanes - 1:
            return False
        
        target_lane_index = lane_index + 1 if direction == "right" else lane_index - 1
        target_lane = road.lanes[target_lane_index]

        follower, leader = target_lane.find_car_by_offset(car.offset)
        old_follower = current_lane.get_follower(car)

        a_current = self.compute_idm_acceleration(car)
        a_new = self.compute_idm_acceleration(car, leader)

        impact_new_follower = 0
        impact_old_follower = 0

        if follower:
            a_before = self.compute_idm_acceleration(follower, leader)
            a_after = self.compute_idm_acceleration(follower, car)
            impact_new_follower = a_after - a_before

        if old_follower:
            a_before = self.compute_idm_acceleration(old_follower, car)
            a_after = self.compute_idm_acceleration(old_follower, car.next_car)
            impact_old_follower = a_after - a_before

        bias = self.bias_left if direction == "left" else self.bias_right

        incentive = (a_new - a_current) - self.p * (impact_new_follower + impact_old_follower) + bias

        min_safe_gap = 1.0  # meters from bumper to bumper

        if follower:
            a_follower_after = self.compute_idm_acceleration(follower, car)

            # Car bodies go out half-length from center, so check front of follower to back of car
            follower_front = follower.offset + follower.length / 2
            car_back = car.offset - car.length / 2
            gap_to_switching_car = car_back - follower_front

            if a_follower_after < self.safety_constraint or gap_to_switching_car < min_safe_gap:
                return False

        if leader:
            car_front = car.offset + car.length / 2
            leader_back = leader.offset - leader.length / 2
            gap_to_leader = leader_back - car_front

            if gap_to_leader < min_safe_gap:
                return False

        return incentive > self.a_threshold

class Car:
    def __init__(self, id, offset = 0, speed = 0, next_car = None, driver_profile = "basic"): # MAKE CLASS OBSTACLE
        self.is_obstacle = False
        self.is_slowed = False
        self.timers = {
            "slowness_timer": 0
        }
        self.id = id

        self.probability = 0.05

        self.lane = None
        self.offset = offset  # position along the lane
        self.velocity_magnitude = speed  # scalar speed
        self.next_car = next_car  # car in front

        self.max_speed = 0

        self.driver_profile = driver_profile

        self.driver_model = DriverModel(driver_profile) 
           
        self.length = CAR_LENGTH
        self.stopped = False  # for collision handling

        self.lane_change_timer = 3.0  # VARIABLE TO CHANGE
        self.time_since_last_lane_change = 0

    def evaluate_lane_change(self, direction, road):
        return self.driver_model.evaluate_lane_change(self, direction, road)

    def update(self, dt):
        if self.is_obstacle:
            return
                
        if self.is_slowed:
            if self.timers["slowness_timer"] > 0:
                self.decelerate(dt)
                self.timers["slowness_timer"] = max(0, self.timers["slowness_timer"] - dt)
            else:
                self.is_slowed = False
            return
        
        self.acceleration = self.driver_model.compute_idm_acceleration(self)
        self.move(dt)

    def move(self, dt):
        self.velocity_magnitude += self.acceleration * dt
        self.offset += self.velocity_magnitude * dt
    
    def calculate_desired_velocity(self, max_speed): # IMPLEMENT THIS FURTHER
        self.desired_velocity = max_speed

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
        car.max_speed = self.max_speed
        car.lane = self
        car.calculate_desired_velocity(self.max_speed)
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

    def get_follower(self, car):
        index = self.cars.index(car)
        if index > 0:
            return self.cars[index-1]


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
        car.time_since_last_lane_change = 0
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
