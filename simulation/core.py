# simulation/core.py
import numpy as np
from config import OBSTACLE_LENGTH, CAR_LENGTH, LANE_WIDTH, DRIVER_TYPES
from simulation.drivers import DriverModel, RandomDriverModel
import random


    
class Car:
    def __init__(self, id, offset = 0, speed = 0, next_car = None, driver_type = "basic", length = CAR_LENGTH): # MAKE CLASS OBSTACLE
        self.is_obstacle = False
        self.is_slowed = False
        self.timers = {
            "slowness_timer": 0
        }
        self.id = id

        self.counter = 0

        self.probability = 0.05

        self.new_acceleration = 0

        self.lane = None
        self.offset = offset  # position along the lane
        self.velocity_magnitude = speed  # scalar speed
        self.next_car = next_car  # car in front

        self.max_speed = 0
        self.acceleration = 0

        self.driver_profile = driver_type
        
        if driver_type == "random":
            self.driver_model = RandomDriverModel()
        else:
            self.driver_model = DriverModel(driver_type)

        self.acc_value = self.driver_model.a
        self.decc_value = self.driver_model.b * 2
           
        self.length = length
        self.stopped = False  # for collision handling

        self.certainty_threshold = 0.5
        self.certainty_right_timer = 0
        self.certainty_left_timer = 0

        self.lane_change_threshold = 3.0  # VARIABLE TO CHANGE
        self.lane_change_timer = 0

    def evaluate_lane_change(self, direction, road):
        if self.is_obstacle:
            return False
        return self.driver_model.evaluate_lane_change(self, direction, road)

    
    def distance_to_car_ahead(self, other_car):
        if self.id == other_car.id:
            return float('inf')

        self_offset = self.offset
        other_offset = other_car.offset

        total_road_length = self.lane.road.length  # Works because all roads share a total circular structure


        if self_offset < other_offset:
            return other_offset - self_offset
        else:
            return total_road_length - self_offset + other_offset   
        

    def update(self, dt):
        self.counter += 1

        if self.is_obstacle:
            return

        if self.driver_model.braking_chance > random.uniform(0, 1):
            self.apply_slowness(10*dt)

        self.acceleration = self.driver_model.compute_idm_acceleration(self, is_test=False)

        if self.is_slowed:
            if self.timers["slowness_timer"] > 0:
                self.deccelerate(dt)
                self.timers["slowness_timer"] = max(0, self.timers["slowness_timer"] - dt)
            else:
                self.is_slowed = False

        self.move(dt)

    def move(self, dt):
        self.velocity_magnitude = max(self.velocity_magnitude + self.acceleration * dt, 0)
        self.offset += self.velocity_magnitude * dt
    
    def calculate_desired_velocity(self, max_speed): # IMPLEMENT THIS FURTHER
        self.desired_velocity = max_speed

    def accelerate(self, dt):
        self.velocity_magnitude = max(self.velocity_magnitude + self.acc_value * dt, 0)

    def deccelerate(self, dt):
        self.velocity_magnitude = max(self.velocity_magnitude - self.decc_value * dt, 0)

    def apply_slowness(self, delta_time):
        self.timers["slowness_timer"] = delta_time
        self.is_slowed = True

    def has_lane(self):
        return self.lane is not None
       
        

class Lane:
    def __init__(self, id, start_pos, end_pos, max_speed):
        self.id = id
        self.start_pos = np.array(start_pos, dtype=float)
        self.end_pos = np.array(end_pos, dtype=float)
        self.cars = []
        self.obstacles = []
        self.max_speed = max_speed
        self.length = np.linalg.norm(self.end_pos - self.start_pos)
        self.next_lane = None
        self.unit_direction = (self.end_pos - self.start_pos) / self.length

        self.num_obstacles = 0

    def add_obstacle(self, obstacle):
        self.cars.append(obstacle)
        self.num_obstacles += 1
        obstacle.lane = self
        self.update_next_cars()

    def add_car(self, car):
        self.cars.append(car)
        car.max_speed = self.max_speed
        car.lane = self
        car.calculate_desired_velocity(self.max_speed)
        self.update_next_cars()

    def remove_car(self, car):
        car.next_car = None
        self.cars.remove(car)
        self.update_next_cars()

    def update_next_cars(self):
        self.cars.sort(key=lambda c: c.offset)

        for i in range(len(self.cars) - 1):
            front = self.cars[i + 1]
            rear = self.cars[i]
            rear.next_car = front

        if self.cars:
            if self.next_lane:
                self.cars[-1].next_car = self.next_lane.cars[0]
            else:
                self.cars[-1].next_car = None  # Last car should have no next_car


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
        found = False

        for car in self.cars:
            if car.offset <= offset:
                before = car
            elif car.offset > offset and not found:
                after = car
                found = True
                break

        if not after and self.road.next_road:
            try:
                next_lane = self.road.next_road.lanes[self.id]
                if next_lane.cars:
                    after = next_lane.cars[0]  # First car in the next segment
            except IndexError:
                pass  # Lane index doesn't exist in next road

        return before, after
    
    def has_cars(self):
        return len(self.cars) > self.num_obstacles
        




class Road:
    def __init__(self, num_lanes, max_speed, start_pos, end_pos, lane_width=LANE_WIDTH):
        self.lanes = []

        self.num_lanes = num_lanes
        self.start_pos = np.array(start_pos, dtype=float)
        self.end_pos = np.array(end_pos, dtype=float)
        self.length = np.linalg.norm(self.end_pos - self.start_pos)
        self.next_road = None
        self.unit_direction = (self.end_pos - self.start_pos) / self.length
        self.dt = 0
        self.normal = np.array([-self.unit_direction[1], self.unit_direction[0]])
        self.max_speed = max_speed

        for i in range(num_lanes):
            offset = (i - (num_lanes - 1) / 2) * lane_width
            offset_vector = self.normal * offset
            lane_start = self.start_pos + offset_vector
            lane_end = self.end_pos + offset_vector
            lane = Lane(i, lane_start, lane_end, max_speed)

            lane.road = self
            self.lanes.append(lane)

    def set_next_road(self, next_road):
        self.next_road = next_road
        for i in range(len(self.lanes)):
            self.lanes[i].next_lane = self.next_road.lanes[i]
            self.lanes[i].update_next_cars()

    
    def update(self, dt):
        for lane in self.lanes:
            lane.update(dt)

    def get_all_cars(self):
        return [car for lane in self.lanes for car in lane.cars]       

    def update_next_lane(self, lane_idx, next_lane):
        self.lanes[lane_idx].update_next_lane(next_lane)

    def switch_lane(self, car, new_lane):
        old_lane = car.lane
        car.lane = new_lane 
        old_lane.remove_car(car)
        car.lane_change_timer = 0
        car.certainty_right_timer = 0
        car.certainty_left_timer = 0
        new_lane.add_car(car)

    def has_cars(self) -> bool:
        for lane in self.lanes:
            if lane.has_cars(): return True

        return False
            

class System:
    def __init__(self, dt=0.01, final_time=100):
        self.roads = []
        self.dt = dt
        self.final_time = final_time
        self.total_length = 0
        self.time = 0
        self.index = 1
        self.obstacle_index = -1
        self.num_roads = 0
        self.obstacle_length = OBSTACLE_LENGTH

    def increment_index(self):
        self.index += 1

    def add_car(self, offset, driver_type, speed, lane_index, road_index):
        car = Car(self.index, offset=offset, speed=speed, driver_type=driver_type)
        self.roads[road_index].lanes[lane_index].add_car(car)

        self.increment_index()


    def add_road(self, road):
        self.roads.append(road)
        self.total_length += road.length
        self.num_roads += 1
        road.dt = self.dt
        self.time_stopper = 50

    def update(self, logger = None):
        for road in self.roads:
            road.update(self.dt)
        self.time += self.dt

        if self.time > self.time_stopper:
            print(f"The current time now is: {self.time}")
            self.time_stopper += 50

        if logger:
            logger.log(self)
            logger.log_density_and_flow(self)
        

    def equal_distance_car_creator(self, num_cars, driver_type = "basic", speed=0, lane_index=0, road_index = 0, length = None, offset = 0):
        if num_cars <= 0:
            return
        if not length: length = self.total_length

        split_length = length / num_cars

        for i in range(num_cars):
            if driver_type == "mix":
                driver = random.choice(DRIVER_TYPES)
            else:
                driver = driver_type
            self.add_car(offset + split_length*(i), driver, speed, lane_index, road_index)

    def gaussian_spread_car_creator(self, num_cars, driver_type="basic", speed=0, lane_index=0, mean=None, std_dev=None, road_index = 0):

        road_length = self.roads[road_index].length  # assuming single road for now
        car_length = CAR_LENGTH  # adjust based on your Car class
        min_spacing = car_length + 1  # buffer to avoid overlaps

        if mean is None:
            mean = road_length / 2
        if std_dev is None:
            std_dev = road_length / 6  

        positions = []
        attempts = 0
        max_attempts = 10000

        while len(positions) < num_cars and attempts < max_attempts:
            pos = np.clip(np.random.normal(mean, std_dev), 0, road_length) 
            if all(abs(pos - p) > min_spacing for p in positions):
                positions.append(pos)
            attempts += 1

        if len(positions) < num_cars:
            print(f"Warning: Could only place {len(positions)} cars without overlap.")

        positions.sort()

        for pos in positions:
            self.add_car(pos, driver_type, speed, lane_index, road_index)

    
    def block_lane_from_offsets(self, road_index, lane_index, start_offset, end_offset=None ):
        if road_index >= len(self.roads):
            return
        if lane_index >= len(self.roads[road_index].lanes):
            return
        
        lane = self.roads[road_index].lanes[lane_index]

        if end_offset is None:
            end_offset = lane.length
        
        if start_offset is None:
            start_offset = 0

        start_offset += self.obstacle_length / 2
        min_space = 1

        while start_offset < end_offset - self.obstacle_length:
            obstacle = Obstacle(self.obstacle_index, start_offset, self.obstacle_length)
            lane.add_obstacle(obstacle)
            self.obstacle_index -= 1
            start_offset += self.obstacle_length + min_space


        
        


class Obstacle(Car):
    def __init__(self, id, offset, length):
        super().__init__(id, offset, speed=0, length=length)
        self.is_obstacle = True
        self.velocity_magnitude = 0  # Make sure it's truly static
        self.desired_velocity = 0
        self.acceleration = 0