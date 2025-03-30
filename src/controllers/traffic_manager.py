from typing import Dict, List, Set, Tuple
from src.models.robot import Robot, RobotStatus
from ..models.nav_graph import NavigationGraph
import logging
from collections import defaultdict
import time

class TrafficManager:
    def __init__(self):  
        self.occupied_vertices = set()
        self.waiting_queues = defaultdict(list)
        self.reserved_paths = {}
        self.vertex_occupancy = {}  
        self.lane_occupancy = {}  
        self.deadlock_detection_enabled = True
        self.collision_zones = {}  
        self.traffic_density = defaultdict(int)  
        self.deadlock_timeout = 5.0  
        self.vertex_reservations = defaultdict(list)  
        
    def check_path_availability(self, robot_id: int, path: List[int]) -> bool:
        if not path:
            return False

        for vertex in path[1:-1]:
            if vertex in self.occupied_vertices:
                return False

        current_time = time.time()
        for vertex in path[1:-1]:
            reservations = self.vertex_reservations[vertex]
            reservations = [r for r in reservations if r['time'] > current_time]
            if any(r['robot_id'] != robot_id for r in reservations):
                return False

        return True

    def reserve_path(self, robot_id: int, path: List[int]) -> bool:
        if not self.check_path_availability(robot_id, path):
            return False

        self.reserved_paths[robot_id] = path
        current_time = time.time()
        
        for i, vertex in enumerate(path[1:-1], 1):
            self.vertex_reservations[vertex].append({
                'robot_id': robot_id,
                'time': current_time + i * 2  
            })

        return True

    def update_robot_position(self, robot_id: int, old_vertex: int, new_vertex: int):
        if old_vertex in self.occupied_vertices:
            self.occupied_vertices.remove(old_vertex)
            
        self.occupied_vertices.add(new_vertex)

        if self.waiting_queues[old_vertex]:
            waiting_robot_id = self.waiting_queues[old_vertex].pop(0)
            return waiting_robot_id

        return None

    def add_to_waiting_queue(self, vertex_id: int, robot_id: int) -> None:
        if robot_id not in self.waiting_queues[vertex_id]:
            self.waiting_queues[vertex_id].append(robot_id)
            logging.info(f"Robot {robot_id} waiting at vertex {vertex_id}")

    def remove_from_waiting_queue(self, vertex_id: int, robot_id: int) -> None:
        if vertex_id in self.waiting_queues and robot_id in self.waiting_queues[vertex_id]:
            self.waiting_queues[vertex_id].remove(robot_id)

    def check_deadlock(self) -> Set[int]:
        if not self.deadlock_detection_enabled:
            return set()

        deadlocked_robots = set()
        visited = set()

        def check_cycle(robot_id: int, path: Set[int]) -> bool:
            if robot_id in path:
                deadlocked_robots.update(path)
                return True
            
            if robot_id in visited:
                return False
                
            visited.add(robot_id)
            path.add(robot_id)
            
            for vertex, queue in self.waiting_queues.items():
                if robot_id in queue:
                    blocking_robot = self.vertex_occupancy.get(vertex)
                    if blocking_robot and check_cycle(blocking_robot, path):
                        return True
                        
            path.remove(robot_id)
            return False

        for vertex, queue in self.waiting_queues.items():
            for robot_id in queue:
                check_cycle(robot_id, set())

        if deadlocked_robots:
            logging.warning(f"Deadlock detected involving robots: {deadlocked_robots}")
            
        return deadlocked_robots

    def resolve_deadlock(self, deadlocked_robots: Set[int]) -> None:
        robots = [(rid, self.fleet_manager.robots[rid]) for rid in deadlocked_robots]
        robots.sort(key=lambda x: (
            x[1].battery,  
            -x[1].wait_time,  
            -x[1].task_count  
        ))
        
        for robot_id, robot in robots:
            if robot.target_vertex:
                alt_paths = self.nav_graph.get_alternative_paths(
                    robot.current_vertex,
                    robot.target_vertex
                )
                
                for path in alt_paths:
                    if self.check_path_availability(robot_id, path):
                        self.fleet_manager.assign_new_path(robot_id, path)
                        break
                else:
                    self.backup_robot(robot_id)

    def clear_all_occupancy(self) -> None:
        self.vertex_occupancy.clear()
        self.lane_occupancy.clear()
        self.waiting_queues.clear()

    def initialize_occupancy_tracking(self) -> None:
        for i in range(self.nav_graph.get_vertex_count()):
            self.vertex_occupancy[i] = -1
            
        for start, end, _ in self.nav_graph.lanes:  
            self.lane_occupancy[(start, end)] = -1
            self.lane_occupancy[(end, start)] = -1  
            
    def update_occupancy(self, robots: List[Robot]) -> None:
        self.initialize_occupancy_tracking()
        for robot in robots:
            if robot.status == RobotStatus.MOVING and len(robot.path) >= 2:
                self.vertex_occupancy[robot.path[0]] = robot.robot_id
                
                current_lane = (robot.path[0], robot.path[1])
                self.lane_occupancy[current_lane] = robot.robot_id
            else:
                self.vertex_occupancy[robot.current_vertex] = robot.robot_id
    
    def is_vertex_occupied(self, vertex_id: int) -> bool:
        return self.vertex_occupancy[vertex_id] != -1
    
    def is_lane_occupied(self, start: int, end: int) -> bool:
        return (start, end) in self.lane_occupancy
    
    def get_occupying_robots(self, vertex_id: int) -> int:
        return self.vertex_occupancy[vertex_id]
    
    def get_lane_occupying_robots(self, start_vertex: int, end_vertex: int) -> int:
        return self.lane_occupancy[(start_vertex, end_vertex)]
    
    def can_enter_vertex(self, robot_id: int, vertex_id: int) -> bool:
        occupying_robot = self.vertex_occupancy[vertex_id]
        return occupying_robot == -1 or (occupying_robot == robot_id)
    
    def can_enter_lane(self, robot_id: int, start_vertex: int, end_vertex: int) -> bool:
        occupying_robot = self.lane_occupancy[(start_vertex, end_vertex)]
        return occupying_robot == -1 or (occupying_robot == robot_id)
    
    def find_alternative_path(self, robot: Robot, target_vertex: int) -> List[int]:
        current = robot.current_vertex
        path = [current]
        visited = {current}
        
        while current != target_vertex:
            neighbors = self.nav_graph.get_vertex_neighbors(current)
            valid_neighbors = [
                n for n in neighbors
                if n not in visited
                and self.can_enter_vertex(robot.robot_id, n)
                and self.can_enter_lane(robot.robot_id, current, n)
            ]
            
            if not valid_neighbors:
                return []  
                
            current = min(
                valid_neighbors,
                key=lambda n: abs(n - target_vertex)
            )
            
            path.append(current)
            visited.add(current)
            
        return path
    
    def handle_collision(self, robot: Robot) -> None:
        robot.status = RobotStatus.WAITING
        robot.path = []
        robot.target_vertex = None
    
    def clear_all_occupancy(self):
        self.vertex_occupancy.clear()
        self.lane_occupancy.clear()
        self.waiting_queues.clear()

    def predict_collisions(self, paths: Dict[int, List[int]]) -> List[Tuple[int, int, int]]:
        collisions = []
        for r1_id, path1 in paths.items():
            for r2_id, path2 in paths.items():
                if r1_id >= r2_id:
                    continue
                    
                for i in range(len(path1) - 1):
                    if i >= len(path2) - 1:
                        continue
                    
                    if (path1[i] == path2[i+1] and path1[i+1] == path2[i]) or \
                       (path1[i] == path2[i] and path1[i+1] == path2[i+1]):
                        collisions.append((r1_id, r2_id, path1[i]))
                        
        return collisions 

    def check_collision(self, robot_id: int, path: List[int]) -> bool:
        if len(path) < 2:
            return False

        for i in range(len(path) - 1):
            start, end = path[i], path[i + 1]
            
            if (start in self.vertex_occupancy and 
                self.vertex_occupancy[start] != robot_id):
                logging.warning(f"Collision detected at vertex {start}")
                return True
            
            if ((start, end) in self.lane_occupancy and 
                self.lane_occupancy[(start, end)] != robot_id):
                logging.warning(f"Collision detected on lane {start}->{end}")
                return True
        
        return False 

    def check_path_conflicts(self, robot_id: int, path: List[int], occupied_vertices: Dict[int, int]) -> bool:
        if len(path) < 2:
            return False

        for vertex_id, occupying_robot_id in occupied_vertices.items():
            if occupying_robot_id == robot_id:
                continue
                
            if vertex_id in path:
                return True
                
            for i in range(len(path) - 1):
                lane = (path[i], path[i + 1])
                if lane in self.lane_occupancy and self.lane_occupancy[lane] != robot_id:
                    return True

        return False 