import json
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional

class LogManager:
    def __init__(self, log_file: str = "logs/fleet_logs.txt"):
        self.log_file = Path(log_file)
        self.log_file.parent.mkdir(exist_ok=True)
        
        # Configure logging
        logging.basicConfig(
            filename=self.log_file,
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        
        # Also log to console for immediate feedback
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        logging.getLogger().addHandler(console_handler)

class ConfigManager:
    @staticmethod
    def load_nav_graph(file_path: str) -> dict:
        """Load and validate navigation graph from JSON."""
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
                
            # Validate structure
            if not isinstance(data, dict):
                raise ValueError("Navigation graph must be a dictionary")
            if "vertices" not in data or "lanes" not in data:
                raise ValueError("Navigation graph must contain 'vertices' and 'lanes'")
                
            return data
        except Exception as e:
            logging.error(f"Error loading navigation graph: {e}")
            raise

class PerformanceMonitor:
    def __init__(self):
        self.metrics = {
            'start_time': datetime.now(),
            'robot_stats': {},
            'system_stats': {
                'total_tasks_completed': 0,
                'total_distance_traveled': 0,
                'total_collisions_avoided': 0,
                'total_waiting_time': 0
            }
        }
    
    def update_robot_stats(self, robot_id: int, stats: dict):
        """Update statistics for a specific robot."""
        self.metrics['robot_stats'][robot_id] = stats
        
    def generate_report(self) -> dict:
        """Generate comprehensive performance report."""
        current_time = datetime.now()
        runtime = (current_time - self.metrics['start_time']).total_seconds()
        
        return {
            'timestamp': current_time.strftime("%Y-%m-%d %H:%M:%S"),
            'runtime': runtime,
            'robot_statistics': self.metrics['robot_stats'],
            'system_statistics': self.metrics['system_stats']
        }

class PathOptimizer:
    @staticmethod
    def smooth_path(path: List[Tuple[float, float]], smoothing_factor: float = 0.5) -> List[Tuple[float, float]]:
        """Smooth a path for more natural robot movement."""
        if len(path) < 3:
            return path
            
        smoothed = path.copy()
        change = True
        while change:
            change = False
            for i in range(1, len(smoothed) - 1):
                original = smoothed[i]
                smoothed[i] = (
                    smoothed[i][0] + smoothing_factor * (
                        (smoothed[i-1][0] + smoothed[i+1][0])/2 - smoothed[i][0]
                    ),
                    smoothed[i][1] + smoothing_factor * (
                        (smoothed[i-1][1] + smoothed[i+1][1])/2 - smoothed[i][1]
                    )
                )
                if abs(original[0] - smoothed[i][0]) > 0.1 or \
                   abs(original[1] - smoothed[i][1]) > 0.1:
                    change = True
        return smoothed

class CollisionPredictor:
    @staticmethod
    def predict_collisions(paths: Dict[int, List[Tuple[float, float]]], 
                         time_horizon: float = 5.0,
                         robot_radius: float = 15.0) -> List[Tuple[int, int, float]]:
        """Predict potential collisions between robot paths."""
        collisions = []
        for r1_id, path1 in paths.items():
            for r2_id, path2 in paths.items():
                if r1_id >= r2_id:
                    continue
                    
                # Check for path intersections
                for t in range(int(time_horizon * 10)):  # Check every 0.1 seconds
                    time = t / 10
                    pos1 = PathOptimizer.interpolate_position(path1, time)
                    pos2 = PathOptimizer.interpolate_position(path2, time)
                    
                    if pos1 and pos2:
                        distance = ((pos1[0] - pos2[0])**2 + 
                                  (pos1[1] - pos2[1])**2)**0.5
                        if distance < 2 * robot_radius:
                            collisions.append((r1_id, r2_id, time))
                            break
        
        return collisions

    @staticmethod
    def interpolate_position(path: List[Tuple[float, float]], 
                           time: float) -> Optional[Tuple[float, float]]:
        """Interpolate position along path at given time."""
        if not path:
            return None
            
        if time <= 0:
            return path[0]
        if time >= len(path) - 1:
            return path[-1]
            
        index = int(time)
        fraction = time - index
        
        if index + 1 >= len(path):
            return path[-1]
            
        return (
            path[index][0] + fraction * (path[index+1][0] - path[index][0]),
            path[index][1] + fraction * (path[index+1][1] - path[index][1])
        ) 