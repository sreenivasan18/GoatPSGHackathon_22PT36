from typing import List, Optional, Dict, Tuple
import logging
from datetime import datetime
from ..models.robot import Robot, RobotStatus
from ..models.nav_graph import NavigationGraph
from .traffic_manager import TrafficManager
import json
import random
from ..utils.logger import FleetLogger

class FleetManager:
    def __init__(self, nav_graph: NavigationGraph):
        self.nav_graph = nav_graph
        self.robots = {}
        self.robot_paths: Dict[int, List[int]] = {}
        self.traffic_manager = TrafficManager()
        self.logger = FleetLogger()
        self.next_robot_id = 0
        self.paused = False
        self.auto_mode = False
        self.auto_task_interval = 3.0  
        self.auto_task_timer = 0.0
        self.selected_robot: Optional[Robot] = None
        self.task_queue = []  
        self.performance_metrics = {
            'completed_tasks': 0,
            'total_distance': 0,
            'total_wait_time': 0,
            'collisions_avoided': 0,
            'battery_efficiency': 0
        }
        self.setup_logging()
        
    def setup_logging(self) -> None:
        logging.basicConfig(
            filename='src/logs/fleet_logs.txt',
            level=logging.INFO,
            format='%(asctime)s - %(message)s'
        )
        
    def spawn_robot(self, vertex_id: int) -> bool:
        if vertex_id not in self.nav_graph.vertices:
            return False
            
        if vertex_id in self.traffic_manager.vertex_occupancy:
            return False
            
        position = self.nav_graph.vertices[vertex_id]
        robot = Robot(self.next_robot_id, position, vertex_id, self.nav_graph)
        
        self.robots[self.next_robot_id] = robot
        self.traffic_manager.vertex_occupancy[vertex_id] = self.next_robot_id
        
        self.next_robot_id += 1
        return True
    
    def select_robot(self, robot_id: int) -> None:
        for robot in self.robots.values():
            if robot.robot_id == robot_id:
                self.selected_robot = robot
                logging.info(f"Selected Robot {robot_id}")
                return
        self.selected_robot = None
        
    def assign_task(self, target_vertex: int, robot_id: int) -> bool:
        robot = self.robots[robot_id]
        blocked_vertices = {
            r.current_vertex for r in self.robots.values() 
            if r.robot_id != robot_id
        }
        shortest_path = self.nav_graph.get_shortest_path(
            robot.current_vertex,
            target_vertex,
            blocked_vertices=blocked_vertices
        )
        
        if not shortest_path or self.check_path_conflicts(shortest_path):
            logging.info(f"Robot {robot_id} cannot take shortest path - attempting alternative route")
            alt_path = self.nav_graph.get_alternative_path(
                robot.current_vertex,
                target_vertex,
                blocked_vertices=blocked_vertices
            )
            
            if not alt_path:
                logging.warning(f"No valid path found for Robot {robot_id}")
                return False
                
            path = alt_path
            logging.info(f"Robot {robot_id} taking alternative path: {path}")
        else:
            path = shortest_path
        
        if not self.traffic_manager.reserve_path(robot_id, path):
            logging.warning(f"Could not reserve path for Robot {robot_id}")
            return False
        
        robot.path = path
        robot.target_vertex = target_vertex
        robot.state = "moving"
        return True

    def assign_charging_task(self, robot_id: int) -> bool:
        if robot_id not in self.robots:
            return False
            
        robot = self.robots[robot_id]
        min_dist = float('inf')
        nearest_charger = None
        
        for vertex_id in self.nav_graph.vertices:
            if self.nav_graph.is_charging_station(vertex_id):
                path = self.nav_graph.get_shortest_path(robot.current_vertex, vertex_id)
                if path:
                    dist = len(path)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_charger = vertex_id
        
        if nearest_charger is None:
            return False
            
        return self.assign_task(nearest_charger, robot_id)

    def update(self, dt: float):
        for robot in self.robots.values():
            robot.update(dt)
            if robot.needs_charging():
                self.route_to_charger(robot)

        self.process_task_queue()

    def route_to_charger(self, robot: Robot):
        if robot.state != "idle":
            return

        blocked_vertices = {r.current_vertex for r in self.robots.values() if r.robot_id != robot.robot_id}
        nearest_charger = self.nav_graph.find_nearest_charger(robot.current_vertex, blocked_vertices)
        
        if nearest_charger is not None:
            path = self.nav_graph.get_path_to_vertex(robot.current_vertex, nearest_charger, blocked_vertices)
            if path and self.traffic_manager.reserve_path(robot.robot_id, path):
                robot.path = path
                robot.target_vertex = nearest_charger
                robot.state = "moving"
                self.logger.info(f"Robot {robot.robot_id} routing to charger at vertex {nearest_charger}")

    def process_task_queue(self):
        self.task_queue.sort(key=lambda x: x[0].priority, reverse=True)
        
        for robot, target_vertex in self.task_queue[:]:
            if robot.state == "idle":
                if self.assign_task(target_vertex, robot.robot_id):
                    self.task_queue.remove((robot, target_vertex))

    def complete_robot_task(self, robot: Robot) -> None:
        del self.robot_paths[robot.robot_id]
        robot.target_vertex = None
        
        if not robot.is_charging:
            robot.state = "idle"
            
        logging.info(f"Robot {robot.robot_id} completed task")

    def get_idle_robots(self) -> List[Robot]:
        return [robot for robot in self.robots.values() 
                if robot.state == "idle" and not robot.emergency_stop]

    def save_state(self, filename: str = "fleet_state.json") -> bool:
        try:
            state = {
                "robots": {
                    rid: robot.get_status_dict() 
                    for rid, robot in self.robots.items()
                },
                "paths": self.robot_paths,
                "next_robot_id": self.next_robot_id
            }
            
            with open(filename, 'w') as f:
                json.dump(state, f, indent=2)
            
            logging.info(f"Fleet state saved to {filename}")
            return True
            
        except Exception as e:
            logging.error(f"Error saving fleet state: {e}")
            return False

    def load_state(self, filename: str = "fleet_state.json") -> bool:
        try:
            with open(filename, 'r') as f:
                state = json.load(f)
                
            self.robots.clear()
            self.robot_paths.clear()
            
            for rid, robot_data in state["robots"].items():
                self.robots[int(rid)] = Robot.from_dict(robot_data)
                
            self.robot_paths = {
                int(rid): path 
                for rid, path in state["paths"].items()
            }
            
            self.next_robot_id = state["next_robot_id"]
            
            logging.info(f"Fleet state loaded from {filename}")
            return True
            
        except Exception as e:
            logging.error(f"Error loading fleet state: {e}")
            return False

    def emergency_stop_all(self) -> None:
        for robot in self.robots.values():
            robot.emergency_stop_toggle()
        logging.warning("Emergency stop triggered for all robots")

    def resume_all(self) -> None:
        for robot in self.robots.values():
            if robot.emergency_stop:
                robot.emergency_stop_toggle()
        logging.info("Resumed all robots from emergency stop")

    def assign_random_tasks(self) -> None:
        idle_robots = self.get_idle_robots()
        available_vertices = [
            v for v in self.nav_graph.vertices.keys()
            if not self.traffic_manager.vertex_occupancy.get(v)
        ]
        
        for robot in idle_robots:
            if available_vertices:
                target = random.choice(available_vertices)
                if self.assign_task(target, robot.robot_id):
                    available_vertices.remove(target)

    def get_robot_at_position(self, x: float, y: float, radius: float) -> Optional[Robot]:
        for robot in self.robots.values():
            robot_pos = robot.get_current_position(self.nav_graph)
            distance = ((robot_pos[0] - x) ** 2 + (robot_pos[1] - y) ** 2) ** 0.5
            if distance <= radius:
                return robot
        return None 

    def clear_all_robots(self):
        self.robots.clear()
        self.traffic_manager.clear_all()
        self.next_robot_id = 0  

    def is_vertex_occupied(self, vertex_id):
        return any(robot.current_vertex == vertex_id for robot in self.robots.values()) 

    def optimize_task_assignment(self) -> None:
        idle_robots = self.get_idle_robots()
        pending_tasks = self.task_queue.copy()
        
        assignments = {}
        for robot in idle_robots:
            if not pending_tasks:
                break
                
            task_scores = []
            for task in pending_tasks:
                score = self._calculate_task_score(robot, task)
                task_scores.append((score, task))
                
            if task_scores:
                best_score, best_task = max(task_scores)
                assignments[robot.robot_id] = best_task
                pending_tasks.remove(best_task)
                
        for robot_id, task in assignments.items():
            self.assign_task(task['target'], robot_id)
            
    def _calculate_task_score(self, robot, task) -> float:
        distance_score = 1.0 / (1.0 + self._calculate_path_length(
            robot.current_vertex, task['target']))
        battery_score = robot.battery / 100.0
        urgency_score = task.get('priority', 1.0)
        
        return (
            0.4 * distance_score +
            0.3 * battery_score +
            0.3 * urgency_score
        )
        
    def generate_performance_report(self) -> dict:
        report = {
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'metrics': self.performance_metrics.copy(),
            'robots': {
                rid: {
                    'tasks_completed': robot.task_count,
                    'distance_traveled': robot.distance_traveled,
                    'wait_time': robot.wait_time,
                    'collisions_avoided': robot.collision_count,
                    'battery_efficiency': robot.battery
                }
                for rid, robot in self.robots.items()
            }
        }
        return report 

    def check_path_conflicts(self, path: List[int]) -> bool:
        if not path:
            return False

        occupied_vertices = {
            robot.current_vertex: robot.robot_id 
            for robot in self.robots.values()
        }

        for i in range(len(path) - 1):
            start, end = path[i], path[i + 1]
            
            if start in occupied_vertices or end in occupied_vertices:
                return True
                
            if self.traffic_manager.is_lane_occupied(start, end):
                return True
            
            for robot in self.robots.values():
                if robot.target_vertex is not None:
                    robot_path = self.nav_graph.get_shortest_path(
                        robot.current_vertex,
                        robot.target_vertex
                    )
                    if self._paths_conflict(path, robot_path):
                        return True

        return False

    def _paths_conflict(self, path1: List[int], path2: List[int]) -> bool:
        if not path1 or not path2:
            return False

        shared_vertices = set(path1[1:-1]).intersection(set(path2[1:-1]))
        if shared_vertices:
            return True
                            
        for i in range(len(path1) - 1):
            for j in range(len(path2) - 1):
                if (path1[i] == path2[j+1] and path1[i+1] == path2[j]) or \
                   (path1[i] == path2[j] and path1[i+1] == path2[j+1]):
                    return True

        return False 