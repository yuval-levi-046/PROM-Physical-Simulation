#simulation/controller.py

from simulation.spawners import *
from config import CAR_LENGTH

class TrafficController:
    def __init__(self, system, logger=None):
        self.system = system
        self.logger = logger
        self.spawn_rules = []
        self.rules = []
        self.exit_gate = None

    def control(self, dt, should_print = False):
        """Runs all registered spawn rules and handles car transfer between roads."""

        self.exit_gate.update(dt)

        for rule in self.spawn_rules:
            # if self.system.time > 10:
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
                        if self.exit_gate and self.exit_gate.can_exit():
                            try:
                                lane.remove_car(car)
                                car.offset = 0

                                if car.lane.next_lane:
                                    lane.next_lane.add_car(car)
                                else:
                                    if hasattr(self, 'logger') and self.logger:
                                        self.logger.log_exit(car, self.system.time)

                            except:
                                if should_print:
                                    print("Warning removed car that doesn't exist!", car.id)
                        else:
                            # print("Not letting them exit", car.offset)
                            car.velocity_magnitude = 0
                        
                        

        return False


    def add_slow_car(self, time, delta_time, car_index):
        self.rules.append([time, delta_time, car_index])

    def add_exit_gate(self, outflow_rate):
        self.exit_gate = ExitGate(outflow_rate, self.system.dt)


    def add_spawn_rule(self, rule_func):
        """Registers a spawn rule function: rule(dt, system)"""
        self.spawn_rules.append(rule_func)


class ExitGate:
    def __init__(self, outflow_rate, dt):
        self.dt = dt
        self.exit_interval = 3600 / outflow_rate  # seconds between allowed exits
        self.time_since_last_exit = 0.0

    def can_exit(self):
        if self.time_since_last_exit >= self.exit_interval:
            self.time_since_last_exit = 0.0
            return True
        return False

    def update(self, dt):
        self.time_since_last_exit += dt
