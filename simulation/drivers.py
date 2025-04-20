from globals.driver_profiles import DRIVER_PROFILES
import numpy as np

class DriverModel:
     
    def __init__(self, profile):
        driver_type = DRIVER_PROFILES[profile]

        self.profile_name = profile
        self.idm_params = driver_type.get("idm_params", {})  # Optional override

        self.velocity_threshold = 0

        self.p = driver_type.get("politeness", 0.3)
        self.a_threshold = driver_type.get("lane_change_threshold", 0.2)
        self.bias_left = driver_type.get("bias_left", 0.2)
        self.bias_right = driver_type.get("bias_right", -0.2)

        self.braking_chance = driver_type.get("braking_chance", 1e-3)

        self.speed_difference = driver_type.get("speed_difference", 0)
        self.desired_speed = np.clip(np.random.normal(self.speed_difference, 5), self.speed_difference-10, self.speed_difference+10)

        self.a = self.idm_params.get("a", 1.5)
        self.b = self.idm_params.get("b", 2.0)
        self.delta = self.idm_params.get("delta", 4)
        self.s0 = self.idm_params.get("s0", 2.0)
        self.T = self.idm_params.get("T", 1.5)

        self.safety_constraint = -self.b * 4
        self.speed_factor = 1.5

    def compute_idm_acceleration(self, car, leader=0, is_test = True):
        
        v = car.velocity_magnitude
        v0 = car.max_speed*self.speed_factor + self.desired_speed

        if leader == 0:
            leader = car.next_car

        if leader is None:
            s = float('inf')
            delta_v = 0

        else:
            distance = car.distance_to_car_ahead(leader)

            s = max(distance - leader.length, 1)

            delta_v = v - leader.velocity_magnitude

        s_star = self.s0 + v * self.T + (v * delta_v) / (2 * (self.a * self.b)**0.5)
        
        perception_error = 1
        if is_test:
            perception_error = np.clip(np.random.normal(1.0, 0.4), 0.6, 1.4)

        s_star *= perception_error

        acceleration = self.a * (1 - (v / v0)**self.delta - (s_star / s)**2)

        acceleration = np.clip(acceleration, -self.b*5, self.a*6)

        return acceleration
    
    def evaluate_lane_change(self, car, direction, road):

        if car.velocity_magnitude < self.velocity_threshold:
            return False
        if abs(car.offset - car.lane.length) < car.length*1.5:
            return False
        elif abs(car.offset) < car.length*1.5:
            return False

        current_lane = car.lane
        lane_index = current_lane.id

        if direction == "left" and lane_index == 0:
            return False
        if direction == "right" and lane_index == road.num_lanes - 1:
            return False
        
        bias = self.bias_left if direction == "left" else self.bias_right
        
        target_lane_index = lane_index + 1 if direction == "right" else lane_index - 1
        target_lane = road.lanes[target_lane_index]

        follower, leader = target_lane.find_car_by_offset(car.offset)
        old_follower = current_lane.get_follower(car)
        
        if car.next_car and car.next_car.is_obstacle:
            bias += 1

        if leader and leader.is_obstacle:
            bias -= 0.3

        if direction == "left" and leader and not leader.is_obstacle:
            if leader.velocity_magnitude < car.velocity_magnitude and leader.velocity_magnitude > 5:
                car.deccelerate(road.dt)

        if direction == "right" and old_follower:
            if old_follower.velocity_magnitude > self.desired_speed + car.max_speed:
                bias += (1 - self.p) * self.bias_right * 2


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

        if car.velocity_magnitude < 3:
            politeness_bias = -self.p * 0.8
        else:
            politeness_bias = 0

        incentive = (a_new - a_current) - (self.p + politeness_bias) * (impact_new_follower + impact_old_follower) + bias

        min_safe_gap = 1.0  

        if follower:
            a_follower_after = self.compute_idm_acceleration(follower, car)

            gap_to_switching_car = follower.distance_to_car_ahead(car) - (follower.length / 2 + car.length / 2)

            if a_follower_after < self.safety_constraint or gap_to_switching_car < min_safe_gap:
                return False

        if leader:
            gap_to_leader = car.distance_to_car_ahead(leader) - (car.length / 2 + leader.length / 2)

            if gap_to_leader < min_safe_gap:

                return False
            
        return incentive > self.a_threshold
        

class RandomDriverModel(DriverModel):

    def __init__(self, v_mean=0, v_std=20, t_mean=1.5, t_std=1):
        super().__init__("basic")
        self.desired_speed = float(np.clip(np.random.normal(v_mean, v_std), -v_std, v_std))
        self.idm_params["T"] = float(np.clip(np.random.normal(t_mean, t_std), 0.5, 3))
