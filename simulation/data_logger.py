# simulation/data_logger.py
import pandas as pd
import numpy as np

class DataLogger:
    def __init__(self, bin_size = 6.5, road_length = 800, expected_total_cars=None, tag=None):
        self.records = []
        self.exits = []
        self.lane_switches = []
        self.active_cars_log = []
        self.entries = {}
        self.metadata = {
            "tag": tag,
            "num_roads": 0,
            "total_lanes": 0,
            "total_cars": 0,
            "road_length": road_length,
            "dt": None,
            "total_sim_time": None,
            "time_steps": 0,
            "expected_total_cars": expected_total_cars,
        }

        self.density_log = []
        self.flow_log = []
        self.previous_offsets = {}  # Track previous car positions
        self.segment_size = 6.5
        self.road_length = road_length
        self.segments = np.arange(0, road_length + self.segment_size, self.segment_size)

    def log(self, system):
        

        for road_id, road in enumerate(system.roads):
            for lane_id, lane in enumerate(road.lanes):
                for car in lane.cars:
                    if not car.is_obstacle:
                        record = {
                            "time": round(system.time, 4),
                            "car_id": car.id,
                            "lane_id": lane.id,
                            "offset": car.offset,
                            "velocity": car.velocity_magnitude,
                            "acceleration": car.acceleration,
                            "driver_type": car.driver_profile,
                            "desired_speed": getattr(car.driver_model, "desired_speed", None)
                        }
                        self.records.append(record)

        # Update dynamic metadata
        self.metadata["time_steps"] += 1

    def log_density_and_flow(self, system):
        time_now = round(system.time, 4)
        dt = system.dt  # Use your system's dt if stored in metadata

        segment_counts = [0] * (len(self.segments) - 1)  # For density
        flow_per_segment = [0] * (len(self.segments) - 1)  # For flow

        # One pass through sorted cars
        for road in system.roads:
            for lane in road.lanes:
                segment_index = 0
                for car in lane.cars:
                    if car.is_obstacle:
                        continue

                    current_offset = car.offset
                    prev_offset = self.previous_offsets.get(car.id, None)

                    # === DENSITY COUNT ===
                    while segment_index < len(self.segments) - 1 and current_offset >= self.segments[segment_index + 1]:
                        segment_index += 1
                    if segment_index < len(self.segments) - 1:
                        segment_counts[segment_index] += 1

                    # === FLOW CHECK ===
                    if prev_offset is not None:
                        for i in range(len(self.segments) - 1):
                            boundary = self.segments[i + 1]
                            if prev_offset < boundary <= current_offset:
                                flow_per_segment[i] += 1  # Crossing detected

                    # Update previous offset for next step
                    self.previous_offsets[car.id] = current_offset

        # Log both density and flow
        for i in range(len(segment_counts)):
            segment_start = self.segments[i]
            segment_end = self.segments[i + 1]
            segment_length = segment_end - segment_start

            density = segment_counts[i] / segment_length if segment_length > 0 else 0
            flow = flow_per_segment[i] / dt  # Flow in vehicles per second

            self.density_log.append({
                "time": time_now,
                "segment_start": segment_start,
                "segment_end": segment_end,
                "density": density
            })

            self.flow_log.append({
                "time": time_now,
                "segment_start": segment_start,
                "segment_end": segment_end,
                "flow": flow
            })

    def enable_sensor_logger(self, sensor_location=500, aggregation_window=30):
        self.sensor_location = sensor_location
        self.aggregation_window = aggregation_window
        self.sensor_time_bin = 0.0
        self.sensor_speed_samples = []
        self.sensor_speed_log = []
        self.sensor_previous_offsets = {}

    def log_sensor_speed(self, system):
        self.sensor_time_bin += system.dt

        for road in system.roads:
            for lane in road.lanes:
                for car in lane.cars:
                    if car.is_obstacle:
                        continue

                    prev = self.sensor_previous_offsets.get(car.id, car.offset)
                    curr = car.offset

                    # Sensor crossing condition: crossed the sensor in this step
                    if prev < self.sensor_location <= curr:
                        self.sensor_speed_samples.append(car.velocity_magnitude)

                    self.sensor_previous_offsets[car.id] = curr

        if self.sensor_time_bin >= self.aggregation_window:
            avg_speed = np.mean(self.sensor_speed_samples) if self.sensor_speed_samples else 0.0
            count = len(self.sensor_speed_samples)

            self.sensor_speed_log.append({
                "time": round(system.time, 2),
                "average_speed": avg_speed,
                "count": count
            })

            self.sensor_time_bin = 0.0
            self.sensor_speed_samples = []

    def to_sensor_dataframe(self):
        import pandas as pd
        return pd.DataFrame(self.sensor_speed_log)

    def log_entry(self, index, system_time):
        self.entries[index] = round(system_time, 4)

    def log_lane_switch(self, car, old_lane_id, new_lane_id, time):
        self.lane_switches.append({
            "car_id": car.id,
            "from_lane": old_lane_id,
            "to_lane": new_lane_id,
            "time": round(time, 4)
        })

    def log_exit(self, car, system_time):
        entry_time = self.entries.get(car.id, None)
        duration = round(system_time - entry_time, 4) if entry_time else system_time
            
        self.exits.append({
            "car_id": id(car),
            "exit_time": round(system_time, 4),
            "last_offset": car.offset,
            "last_velocity": car.velocity_magnitude,
            "duration": duration
        })

    def to_exit_dataframe(self):
        import pandas as pd
        return pd.DataFrame(self.exits)

    def finalize_metadata(self, system):
        self.metadata.update({
            "num_roads": len(system.roads),
            "total_lanes": sum(len(road.lanes) for road in system.roads),
            "total_cars": sum(len(lane.cars) for road in system.roads for lane in road.lanes),
            "dt": system.dt,
            "total_sim_time": system.final_time,
        })

    def to_dataframe(self):
        return pd.DataFrame(self.records)

    def to_csv(self, path="log.csv"):
        df = self.to_dataframe()
        df.to_csv(path, index=False)

    def print_metadata(self):
        print("Simulation Metadata:")
        for key, value in self.metadata.items():
            print(f" - {key}: {value}")

    def to_density_dataframe(self):
        return pd.DataFrame(self.density_log)

    def to_flow_dataframe(self):
        return pd.DataFrame(self.flow_log)
