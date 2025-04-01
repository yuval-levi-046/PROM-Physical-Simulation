#simulation/controller.py

from simulation.spawners import *

class TrafficController:
    def __init__(self, system, logger=None):
        self.system = system
        self.logger = logger
        self.spawn_rules = []
        self.rules = []
        self.index = 0

    def control(self, dt):
        """Runs all registered spawn rules and handles car transfer between roads."""
        for rule in self.spawn_rules:
            if rule(dt, self.system, self.index):
                self.index += 1
        
        for road in self.system.roads:
            for lane in road.lanes:                        
                for car in lane.cars[:]:  # safe iteration'
                    for rule in self.rules:
                        if abs(self.system.time - rule[0]) <= 1e-1: # INTERFERENCE
                            car_index = rule[2]
                            if car.id == car_index:
                                car.apply_slowness(rule[1])

                    if car.offset >= lane.length:
                        # Remove from current lane
                        lane.cars.remove(car)

                        # Reset position and reassign
                        car.offset = 0

                        # Follow the linked lane if available
                        target_lane = lane.next_lane or (lane.road.next_road.lanes[0] if lane.road.next_road else None)

                        if target_lane:
                            car.lane = target_lane
                            target_lane.add_car(car)
                        else:
                            if hasattr(self, 'logger') and self.logger:
                                self.logger.log_exit(car, self.system.time)

    def equal_distance_car_creator(self, num_cars, speed=0, lane_index = 0):

        split_length = self.system.total_length / num_cars
        for i in range(num_cars):
            self.system.add_car(self.index, split_length*(i), speed, lane_index)
            self.index += 1

    def add_slow_car(self, time, delta_time, car_index):
        self.rules.append([time, delta_time, car_index])


    def add_spawn_rule(self, rule_func):
        """Registers a spawn rule function: rule(dt, system)"""
        self.spawn_rules.append(rule_func)
