#simulation/controller.py

from simulation.spawners import *

class TrafficController:
    def __init__(self, system, logger=None):
        self.system = system
        self.logger = logger
        self.spawn_rules = []

    def control(self, dt):
        """Runs all registered spawn rules and handles car transfer between roads."""
        # Handle spawning logic
        for rule in self.spawn_rules:
            rule(dt, self.system)

        # Handle car transfers at the end of lanes
        for road in self.system.roads:
            for lane in road.lanes:
                for car in lane.cars[:]:  # safe iteration
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


    def add_spawn_rule(self, rule_func):
        """Registers a spawn rule function: rule(dt, system)"""
        self.spawn_rules.append(rule_func)
