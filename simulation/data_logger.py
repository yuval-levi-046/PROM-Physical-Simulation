# simulation/data_logger.py
import pandas as pd

class DataLogger:
    def __init__(self, expected_total_cars=None, tag=None):
        self.records = []
        self.exits = []
        self.lane_switches = []
        self.entries = {}
        self.metadata = {
            "tag": tag,
            "num_roads": 0,
            "total_lanes": 0,
            "total_cars": 0,
            "lane_width": None,
            "dt": None,
            "total_sim_time": None,
            "time_steps": 0,
            "expected_total_cars": expected_total_cars,
        }

    def log(self, system):
        for road_id, road in enumerate(system.roads):
            for lane_id, lane in enumerate(road.lanes):
                for car in lane.cars:
                    record = {
                        "time": round(system.time, 4),
                        "car_id": car.id,
                        "lane_id": lane.id,
                        "offset": car.offset,
                        "velocity": car.velocity_magnitude,
                        "acceleration": car.acceleration,
                        "driver_type": car.driver_profile
                    }
                    self.records.append(record)

        # Update dynamic metadata
        self.metadata["time_steps"] += 1

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
