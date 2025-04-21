#simulation/controller.py

from simulation.spawners import *
from config import CAR_LENGTH

class TrafficController:
    def __init__(self, system, logger=None):
        self.system = system
        self.logger = logger
        self.spawn_rules = []
        self.rules = []

    def control(self, dt):
        """Runs all registered spawn rules and handles car transfer between roads."""
        for rule in self.spawn_rules:
            if rule(dt, self.system, self.system.index):
                if hasattr(self, 'logger') and self.logger:
                    self.logger.log_entry(self.system.index, self.system.time)
        
        for road in self.system.roads:

            if not road.has_cars() and self.system.time > 10:
                print("All car's have exited at time: ", self.system.time)
                return True


            for lane in road.lanes:                                                            
                for car in lane.cars[:]:  # safe iteration'
                    for rule in self.rules:
                        if abs(self.system.time - rule[0]) <= 1e-1: # INTERFERENCE
                            car_index = rule[2]
                            if car.id == car_index:
                                car.apply_slowness(rule[1])

                    flag = True                   
                    car.lane_change_timer += dt
                    if car.lane_change_timer > car.lane_change_threshold:
                        if car.evaluate_lane_change("right", road):
                            car.certainty_right_timer += dt
                            if car.certainty_right_timer > car.certainty_threshold:
                                road.switch_lane(car, road.lanes[lane.id + 1])
                                flag = False
                        else:
                            car.certainty_right_timer = 0

                        if car.evaluate_lane_change("left", road) and flag:
                            car.certainty_left_timer += dt
                            if car.certainty_left_timer > car.certainty_threshold:
                                road.switch_lane(car, road.lanes[lane.id - 1])
                        else:
                            car.certainty_left_timer = 0
                    
                    if car.offset >= lane.length: # can switch to next road
                        try:
                            lane.remove_car(car)
                        except:
                            print("Warning removed car that doesn't exist!", car.id)
                        
                        car.offset = 0

                        if car.lane.next_lane:
                            lane.next_lane.add_car(car)
                        else:
                            print("Removing car with index: ", car.id)
                            if hasattr(self, 'logger') and self.logger:
                                self.logger.log_exit(car, self.system.time)

        return False


    def add_slow_car(self, time, delta_time, car_index):
        self.rules.append([time, delta_time, car_index])


    def add_spawn_rule(self, rule_func):
        """Registers a spawn rule function: rule(dt, system)"""
        self.spawn_rules.append(rule_func)
