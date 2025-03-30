from enum import Enum
from typing import List, Optional, Tuple
import time
import random
import logging
from datetime import datetime
import math
from ..models.nav_graph import NavigationGraph
import pygame

class RobotStatus(Enum):
    IDLE = "idle"
    MOVING = "moving"
    WAITING = "waiting"
    CHARGING = "charging"
    TASK_COMPLETE = "task_complete"

class Robot:
    COLORS = [
        (255, 100, 100),  # Red
        (100, 255, 100),  # Green
        (100, 100, 255),  # Blue
        (255, 255, 100),  # Yellow
        (255, 100, 255),  # Magenta
        (100, 255, 255),  # Cyan
    ]

    def __init__(self, robot_id: int, position: Tuple[float, float], vertex_id: int, nav_graph: NavigationGraph):
        self.robot_id = robot_id
        self.position = list(position)
        self.current_vertex = vertex_id
        self.target_vertex = None
        self.battery = 100.0
        self.battery_drain_rate = 0.1
        self.state = "idle"
        self.is_charging = False
        self.is_dead = False
        self.path = []
        self.move_speed = 0.9
        self.color = self.COLORS[robot_id % len(self.COLORS)]
        self.emergency_stop = False
        self.status_history = []  # Track status changes for logging
        self.wait_time = 0  # Track total waiting time
        self.distance_traveled = 0  # Track total distance traveled
        self.task_count = 0  # Track number of completed tasks
        self.collision_count = 0  # Track near-collision incidents
        self.nav_graph = nav_graph
        self.last_battery_update = pygame.time.get_ticks()
        self.waiting_time = 0
        self.priority = 0  # Higher priority for low battery or long waiting time
        
        logging.info(f"Robot {robot_id} created at vertex {vertex_id}")

    def move_to_vertex(self, target_vertex: int, path: List[int]):
        """Start moving to target vertex along given path."""
        self.target_vertex = target_vertex
        self.path = path
        self.state = "moving"

    def update(self, dt: float) -> Optional[str]:
        try:
            # Update battery
            if not self.is_charging:
                self.battery -= self.battery_drain_rate * dt
                if self.battery <= 20.0 and self.state != "charging":
                    self.priority = max(self.priority, 5)  # Increase priority when battery is low

            if self.is_charging and self.battery < 100.0:
                self.battery = min(100.0, self.battery + 5.0 * dt)
                if self.battery >= 100.0:
                    self.is_charging = False
                    self.state = "idle"

            if self.state == "waiting":
                self.waiting_time += dt
                if self.waiting_time > 10.0:  # Timeout after 10 seconds
                    self.priority += 1  # Increase priority for waiting robots

            # Handle movement
            if self.state == "moving" and self.path:
                next_vertex = self.path[0]
                target_pos = self.nav_graph.vertices[next_vertex]
                
                dx = target_pos[0] - self.position[0]
                dy = target_pos[1] - self.position[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < 0.05:  # Keep this threshold small for accurate positioning
                    self.position = list(target_pos)
                    self.current_vertex = next_vertex
                    self.path.pop(0)
                    
                    if not self.path:
                        self.state = "idle"
                        self.target_vertex = None
                        return f"Robot {self.robot_id} reached destination"
                else:
                    # Smooth movement
                    move_distance = min(self.move_speed * dt, distance)
                    self.position[0] += (dx/distance) * move_distance
                    self.position[1] += (dy/distance) * move_distance

            return None
        except Exception as e:
            self.state = "idle"
            return f"Error updating robot {self.robot_id}: {str(e)}"

    def move_towards(self, target_pos: Tuple[float, float], dt: float) -> bool:
        """Move towards target position, return True if reached."""
        if self.state == RobotStatus.WAITING or self.state == RobotStatus.CHARGING:
            return False

        dx = target_pos[0] - self.position[0]
        dy = target_pos[1] - self.position[1]
        distance = (dx * dx + dy * dy) ** 0.5

        if distance < 0.1:  # Reached target
            self.position = target_pos
            return True

        # Move towards target
        move_distance = min(self.move_speed * dt, distance)
        self.position[0] += (dx / distance) * move_distance
        self.position[1] += (dy / distance) * move_distance
        return False

    def start_charging(self):
        self.is_charging = True
        self.state = RobotStatus.CHARGING
        logging.info(f"Robot {self.robot_id} started charging")

    def stop_charging(self):
        self.is_charging = False
        self.state = RobotStatus.IDLE
        logging.info(f"Robot {self.robot_id} stopped charging")

    def emergency_stop_toggle(self) -> None:
        """Toggle emergency stop state."""
        self.emergency_stop = not self.emergency_stop
        self.state = "emergency" if self.emergency_stop else "idle"
        logging.warning(f"Robot {self.robot_id} emergency stop: {self.emergency_stop}")

    def get_status_dict(self) -> dict:
        """Get robot status as dictionary for saving/loading."""
        return {
            "robot_id": self.robot_id,
            "position": self.position,
            "current_vertex": self.current_vertex,
            "target_vertex": self.target_vertex,
            "battery": self.battery,
            "state": self.state,
            "is_waiting": self.state == RobotStatus.WAITING,
            "is_charging": self.state == RobotStatus.CHARGING,
            "emergency_stop": self.emergency_stop
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'Robot':
        """Create robot instance from dictionary data."""
        robot = cls(data["robot_id"], data["position"], data["current_vertex"], None)
        robot.target_vertex = data["target_vertex"]
        robot.battery = data["battery"]
        robot.state = data["state"]
        robot.is_charging = data["is_charging"]
        robot.emergency_stop = data["emergency_stop"]
        return robot

    def _generate_random_color(self) -> Tuple[int, int, int]:
        """Generate a random color for the robot."""
        return (
            random.randint(50, 255),  # Red
            random.randint(50, 255),  # Green
            random.randint(50, 255)   # Blue
        )
    
    def assign_task(self, target_vertex: int, path: List[int]) -> None:
        """Assign a new navigation task to the robot."""
        self.target_vertex = target_vertex
        self.path = path
        self.state = RobotStatus.MOVING
        self.progress = 0.0
        self.last_update_time = time.time()
    
    def get_current_position(self, nav_graph) -> Tuple[float, float]:
        """Get the current position of the robot."""
        if not self.path or len(self.path) < 2:
            return nav_graph.get_vertex_position(self.current_vertex)
            
        start_pos = nav_graph.get_vertex_position(self.path[0])
        end_pos = nav_graph.get_vertex_position(self.path[1])
        
        # Interpolate between start and end positions based on progress
        x = start_pos[0] + (end_pos[0] - start_pos[0]) * self.progress
        y = start_pos[1] + (end_pos[1] - start_pos[1]) * self.progress
        
        return (x, y)
    
    def is_waiting(self) -> bool:
        """Check if the robot is in a waiting state."""
        return self.state == RobotStatus.WAITING
    
    def is_charging(self) -> bool:
        """Check if the robot is charging."""
        return self.state == RobotStatus.CHARGING
    
    def is_task_complete(self) -> bool:
        """Check if the robot has completed its task."""
        return self.state == RobotStatus.TASK_COMPLETE
    
    def get_status_text(self) -> str:
        """Get a human-readable status text."""
        return f"Robot {self.robot_id}: {self.state.value} (Battery: {self.battery:.1f}%)"

    def log_status_change(self, new_status: str):
        """Log status changes with timestamp."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.status_history.append({
            'timestamp': timestamp,
            'status': new_status,
            'battery': self.battery,
            'position': self.position,
            'vertex': self.current_vertex
        })
        logging.info(f"Robot {self.robot_id} status change: {new_status}")

    def needs_charging(self) -> bool:
        """Check if robot needs charging"""
        return self.battery <= 20.0 and not self.is_charging 