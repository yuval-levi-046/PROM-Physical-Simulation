# simulation/core.py
import numpy as np
from config import CAR_COLOR, CAR_WIDTH, CAR_LENGTH, LANE_WIDTH
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
            self.apply_slowness(20*dt)

        self.acceleration = self.driver_model.compute_idm_acceleration(self)

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
        self.velocity_magnitude = max(self.velocity_magnitude - self.decc_value * self.velocity_magnitude / 100 * dt, 0)

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

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

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
            

class System:
    def __init__(self, dt=0.01, final_time=100):
        self.roads = []
        self.dt = dt
        self.final_time = final_time
        self.total_length = 0
        self.time = 0
        self.index = 0
        self.num_roads = 0

    def increment_index(self):
        self.index += 1

    def add_car(self, offset, driver_type, speed, lane_index, road_index):
        car = Car(self.index, offset=offset, speed=speed, driver_type=driver_type)
        self.roads[road_index].lanes[lane_index].add_car(car)
        print(f"Inserting car with index: {car.id} at time: {self.time}")

        self.increment_index()


    def add_road(self, road):
        self.roads.append(road)
        self.num_roads += 1
        road.dt = self.dt

    def update(self, logger = None):
        for road in self.roads:
            road.update(self.dt)
        self.time += self.dt

        if 0 < ((self.time) % 10) < 1e-3:
            print(f"Current time passed: {self.time}")


        if logger:
            logger.log(self)

    def equal_distance_car_creator(self, num_cars, driver_type = "basic", speed=0, lane_index=0, road_index = None):
        split_length = self.total_length / num_cars
        for i in range(num_cars):
            self.add_car(split_length*(i), driver_type, speed, lane_index, road_index)

    def gaussian_spread_car_creator(self, num_cars, driver_type="basic", speed=0, lane_index=0, mean=None, std_dev=None, road_index = None):

        road_index = 0
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
            pos = np.random.normal(mean, std_dev) % road_length  # wrap around
            if all(abs(pos - p) > min_spacing for p in positions):
                positions.append(pos)
            attempts += 1

        if len(positions) < num_cars:
            print(f"Warning: Could only place {len(positions)} cars without overlap.")

        positions.sort()

        for pos in positions:
            self.add_car(pos, driver_type, speed, lane_index, road_index)


class Obstacle(Car):
    def __init__(self, id, offset, length):
        super().__init__(id, offset, speed=0, length=length)
        self.is_obstacle = True
        self.velocity_magnitude = 0  # Make sure it's truly static
        self.desired_velocity = 0
        self.acceleration = 0