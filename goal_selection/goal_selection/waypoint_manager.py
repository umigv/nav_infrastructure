import json
from os import path
from ament_index_python.packages import get_package_share_directory

class GPSWaypoint:
    def __init__(self, lat: float, lon: float):
        self.lat = lat
        self.lon = lon

    def __repr__(self):
        return f"(lat={self.lat}, lon={self.lon})"

class WaypointManager:
    def __init__(self, waypoints_file_name, rcl_logger):
        """
        Assumes waypoints_file_name is within goal_selection/config; should be a JSON file
        with a list of waypoints in the following format:
        {
            "waypoints": [
                {
                    "lat": 0.0,
                    "lon": 0.0
                },
                ...
            ]
        }
        """
        self.waypoints = WaypointManager.parse_waypoints(waypoints_file_name)
        self.logger = rcl_logger
        self.current_index = 0

        self.logger.info(f"WaypointManager initialized with {len(self.waypoints)} waypoints: {self.waypoints}")

    @staticmethod
    def parse_waypoints(waypoints_file_name: str) -> list[GPSWaypoint]:
        goal_selection_dir = get_package_share_directory('goal_selection')
        waypoints_path = path.join(goal_selection_dir, 'config', waypoints_file_name)
        with open(waypoints_path, 'r') as file:
            waypoints_data = json.load(file)["waypoints"]
        
        waypoints = []
        for waypoint in waypoints_data:
            lat = waypoint["lat"]
            lon = waypoint["lon"]
            waypoints.append(GPSWaypoint(lat, lon))

        return waypoints

    def get_next_waypoint(self) -> GPSWaypoint:
        if self.current_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_index]
            self.current_index += 1
            return waypoint
        else:
            return None

    def reset(self):
        self.current_index = 0