import pygame
import sys
import os
import math
from typing import Optional, Tuple, List, Dict, Set
from datetime import datetime
from collections import deque
import networkx as nx
from networkx.algorithms.shortest_paths.astar import astar_path
import random
import pygame.gfxdraw
import logging

from ..models.nav_graph import NavigationGraph
from ..controllers.fleet_manager import FleetManager
from src.models.robot import RobotStatus
from ..models.robot import Robot
from ..utils.logger import FleetLogger

# Color Constants
DARK_THEME = (18, 18, 18)
GRID_COLOR = (30, 30, 30)
TEXT_COLOR = (255, 255, 255)
BUTTON_COLOR = (45, 45, 48)
BUTTON_HOVER = (60, 60, 65)
BUTTON_ACTIVE = (70, 130, 180)
ACCENT_BLUE = (0, 120, 215)
ACCENT_GREEN = (35, 200, 95)
ACCENT_RED = (235, 50, 35)
ACCENT_GOLD = (255, 185, 0)
VERTEX_COLOR = (128, 128, 128)
VERTEX_HIGHLIGHT = (150, 150, 150)
PATH_COLOR = (0, 255, 0, 128)
ROBOT_COLOR = (0, 191, 255)
CHARGING_COLOR = (0, 255, 0)
WAITING_COLOR = (255, 165, 0)
SELECTED_COLOR = (0, 191, 255)
BATTERY_GREEN = (0, 255, 0)
BATTERY_YELLOW = (255, 255, 0)
BATTERY_RED = (255, 0, 0)

# Complete Color Scheme
ACCENT_PURPLE = (147, 112, 219) # Purple
ACCENT_TEAL = (0, 128, 128)    # Teal
ACCENT_RED = (220, 20, 60)     # Crimson red
ACCENT_GREEN = (50, 205, 50)   # Lime green
ACCENT_BLUE = (65, 105, 225)   # Royal blue
ACCENT_GOLD = (255, 215, 0)    # Gold
ACCENT_RED = (220, 20, 60)     # Crimson red
ACCENT_GREEN = (50, 205, 50)   # Lime green
GRAY = (128, 128, 128)         # Standard gray
BLACK = (0, 0, 0)              # Black
WHITE = (255, 255, 255)        # White
YELLOW = (255, 255, 0)         # Yellow
RED = (255, 0, 0)              # Pure red
GREEN = (0, 255, 0)            # Pure green
BLUE = (0, 0, 255)             # Pure blue
PURPLE = (128, 0, 128)         # Pure purple

# Colors with alpha
HIGHLIGHT = (255, 255, 255, 100)  # Semi-transparent white
TRANSPARENT_BLUE = (*ACCENT_BLUE, 128)
TRANSPARENT_RED = (*ACCENT_RED, 128)
TRANSPARENT_GREEN = (*ACCENT_GREEN, 128)
TRANSPARENT_GOLD = (*ACCENT_GOLD, 128)

class Button:
    def __init__(self, x: int, y: int, width: int, height: int, text: str, color: tuple, callback):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = tuple(min(c + 30, 255) for c in color)  # Lighter version of color
        self.callback = callback
        self.font = pygame.font.Font(None, 32)
        self.is_hovered = False

    def draw(self, screen):
        color = self.hover_color if self.is_hovered else self.color
        pygame.draw.rect(screen, color, self.rect)
        text_surface = self.font.render(self.text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.callback()
        elif event.type == pygame.MOUSEMOTION:
            self.is_hovered = self.rect.collidepoint(event.pos)

class SidePanel:
    def __init__(self, x: int, y: int, width: int, height: int):
        self.rect = pygame.Rect(x, y, width, height)
        self.scroll_y = 0
        self.max_scroll = 0
        
    def draw(self, surface: pygame.Surface, fleet_manager, font: pygame.font.Font):
        # Panel background
        panel_surface = pygame.Surface((self.rect.width, self.rect.height), pygame.SRCALPHA)
        pygame.draw.rect(panel_surface, (*DARK_THEME, 200), panel_surface.get_rect())
        
        # Statistics
        y = 10 - self.scroll_y
        stats = [
            f"Total Robots: {len(fleet_manager.robots)}",
            f"Active Tasks: {len(fleet_manager.robot_paths)}",
            f"Idle Robots: {len(fleet_manager.get_idle_robots())}",
            f"Auto Mode: {'On' if fleet_manager.auto_mode else 'Off'}",
            "Emergency Stop: {'Active' if any(r.emergency_stop for r in fleet_manager.robots.values()) else 'Inactive'}"
        ]
        
        for stat in stats:
            text = font.render(stat, True, TEXT_COLOR)
            panel_surface.blit(text, (10, y))
            y += 25
            
        # Robot details
        y += 10
        for robot in fleet_manager.robots.values():
            # Robot header
            header = f"Robot {robot.robot_id} ({robot.state})"
            text = font.render(header, True, ACCENT_BLUE)
            panel_surface.blit(text, (10, y))
            y += 25
            
            # Battery status with color coding
            battery_color = ACCENT_GREEN if robot.battery > 50 else (
                ACCENT_GOLD if robot.battery > 20 else ACCENT_RED
            )
            battery_text = f"Battery: {robot.battery:.1f}%"
            text = font.render(battery_text, True, battery_color)
            panel_surface.blit(text, (20, y))
            y += 20
            
            # Current task
            if robot.target_vertex is not None:
                task = f"Task: → Vertex {robot.target_vertex}"
                text = font.render(task, True, TEXT_COLOR)
                panel_surface.blit(text, (20, y))
                y += 20
                
            y += 10
        
        self.max_scroll = max(0, y - self.rect.height)
        surface.blit(panel_surface, self.rect)
        
    def handle_scroll(self, event):
        if event.type == pygame.MOUSEWHEEL:
            self.scroll_y = max(0, min(self.scroll_y - event.y * 20, self.max_scroll))

class Notification:
    def __init__(self, message: str, type: str = "info", duration: float = 3.0):
        self.message = message
        self.type = type
        self.creation_time = pygame.time.get_ticks()
        self.duration = duration * 1000
        self.alpha = 0
        self.target_alpha = 255

class PathFinder:
    def __init__(self, nav_graph):
        self.nav_graph = nav_graph
        
    def find_path(self, start: int, end: int) -> List[int]:
        """Find path using A* algorithm."""
        return self.nav_graph.get_shortest_path(start, end)

class FleetGUI:
    def __init__(self, nav_graph: NavigationGraph, fleet_manager: FleetManager):
        pygame.init()
        self.width = 1200
        self.height = 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Fleet Management System")
        
        self.COLORS = {
            'background': (20, 20, 30),    # Dark blue-gray
            'text': (255, 255, 255),       # White
            'vertex': (150, 150, 150),     # Gray
            'lane': (100, 100, 100),       # Darker gray for lanes
            'charging': (0, 255, 0),       # Green for charging stations
            'warning': (255, 165, 0),      # Orange
            'error': (255, 0, 0),          # Red
            'success': (0, 255, 0),        # Green
            'highlight': (0, 191, 255),    # Deep sky blue
            'path': (0, 255, 255),         # Cyan
            'panel': (30, 30, 40),         # Slightly lighter than background
            'button': (60, 60, 70),        # Button background
            'grid': (40, 40, 50),          # Grid color
            'button_hover': (80, 80, 90),   # Button hover state
            'robots': [                    # Colors for different robots
                (0, 191, 255),    # Deep sky blue
                (255, 105, 180),  # Hot pink
                (50, 205, 50),    # Lime green
                (255, 215, 0),    # Gold
                (138, 43, 226),   # Blue violet
                (255, 99, 71),    # Tomato
                (30, 144, 255),   # Dodger blue
                (255, 0, 255),    # Magenta
            ]
        }
        
        # Enhanced UI colors
        self.COLORS.update({
            'panel_border': (40, 44, 52),
            'box_bg': (25, 29, 37),
            'box_border': (45, 49, 57),
            'critical': (255, 55, 55),
            'charging': (50, 255, 100)
        })
        
        # Initialize components
        self.nav_graph = nav_graph
        self.fleet_manager = fleet_manager
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.large_font = pygame.font.Font(None, 32)
        
        # State management
        self.mode = "SPAWN"
        self.task_queue = []
        self.current_paths = {}
        self.notifications = []
        
        # Side panel dimensions
        self.panel_width = 250
        self.main_surface_width = 1200 - self.panel_width
        
        # Create buttons and calculate transform
        self.create_buttons()
        self.calculate_graph_transform()
        
        # State management
        self.selected_robot = None
        self.logger = FleetLogger()  # Add logger to GUI class

    def create_buttons(self):
        """Create GUI buttons"""
        button_width = 120
        button_height = 40
        padding = 10
        
        # Create mode buttons
        self.buttons = [
            Button(
                padding,                    # x
                padding,                    # y
                button_width,               # width
                button_height,              # height
                "SPAWN MODE",               # text
                (0, 191, 255),             # color
                self.toggle_mode            # callback
            ),
            Button(
                padding * 2 + button_width,
                padding,
                button_width,
                button_height,
                "TASK MODE",
                (255, 105, 180),
                self.toggle_mode
            ),
            Button(
                padding * 3 + button_width * 2,
                padding,
                button_width,
                button_height,
                "CLEAR ALL",
                (255, 165, 0),
                self.clear_all
            )
        ]

    def handle_task_assignment(self, vertex_id: int):
        """Handle task assignment with proper path finding."""
        if not self.task_queue:
            self.task_queue = list(self.fleet_manager.robots.values())
            self.task_queue.sort(key=lambda x: x.robot_id)
            return

        current_robot = self.task_queue[0]
        
        # Get blocked vertices
        blocked_vertices = {
            r.current_vertex for r in self.fleet_manager.robots.values() 
            if r.robot_id != current_robot.robot_id
        }
        
        # Get path to target
        path = self.nav_graph.get_path_to_vertex(
            current_robot.current_vertex,
            vertex_id,
            blocked_vertices
        )
        
        if not path:
            self.show_notification(
                f"No valid path found for Robot {current_robot.robot_id} to vertex {vertex_id}",
                self.COLORS['warning']
            )
            return False
        
        # Check if it's shortest or longest path
        shortest = nx.astar_path(
            self.nav_graph.graph,
            current_robot.current_vertex,
            vertex_id,
            heuristic=self.nav_graph._manhattan_distance
        )
        
        if len(path) > len(shortest):
            self.show_notification(
                f"Robot {current_robot.robot_id} taking longer path to vertex {vertex_id}",
                self.COLORS['text']
            )
        else:
            self.show_notification(
                f"Robot {current_robot.robot_id} taking shortest path to vertex {vertex_id}",
                self.COLORS['text']
            )

        # Assign the path to the robot
        current_robot.move_to_vertex(vertex_id, path)
        self.current_paths[current_robot.robot_id] = path
        self.task_queue.pop(0)
        return True

    def show_path_blocking_notification(self, robot_id: int, target_vertex: int):
        """Show detailed notification box for path blocking."""
        # Create notification box
        box_width = 400
        box_height = 150
        box_x = (self.screen.get_width() - box_width) // 2
        box_y = (self.screen.get_height() - box_height) // 2
        
        # Draw semi-transparent background
        s = pygame.Surface((self.screen.get_width(), self.screen.get_height()))
        s.set_alpha(128)
        s.fill((0, 0, 0))
        self.screen.blit(s, (0, 0))
        
        # Draw notification box
        pygame.draw.rect(self.screen, self.COLORS['box_bg'], 
                        (box_x, box_y, box_width, box_height))
        pygame.draw.rect(self.screen, self.COLORS['critical'], 
                        (box_x, box_y, box_width, box_height), 2)
        
        # Draw text
        lines = [
            f"No Valid Path Available",
            f"Robot {robot_id} cannot reach vertex {target_vertex}",
            "All possible routes are blocked",
            "Robot will remain stationary",
            "",
            "Click anywhere to continue"
        ]
        
        y_offset = box_y + 20
        for line in lines:
            text_surf = self.font.render(line, True, self.COLORS['text'])
            text_rect = text_surf.get_rect(centerx=box_x + box_width//2, y=y_offset)
            self.screen.blit(text_surf, text_rect)
            y_offset += 25
        
        # Update display and wait for click
        pygame.display.flip()
        waiting_for_click = True
        while waiting_for_click:
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    waiting_for_click = False
                    break

    def draw_path_preview(self, robot: Robot, path: List[int]):
        """Draw preview of the chosen path."""
        if not path:
            return
        
        robot_color = self.COLORS['robots'][robot.robot_id % len(self.COLORS['robots'])]
        
        # Draw path with arrows to show direction
        for i in range(len(path) - 1):
            start_pos = self.transform_point(self.nav_graph.vertices[path[i]])
            end_pos = self.transform_point(self.nav_graph.vertices[path[i + 1]])
            
            # Draw dashed line
            draw_dashed_line(self.screen, robot_color, start_pos, end_pos, 10, 5)
            
            # Draw direction arrow
            mid_x = (start_pos[0] + end_pos[0]) / 2
            mid_y = (start_pos[1] + end_pos[1]) / 2
            pygame.draw.circle(self.screen, robot_color, (int(mid_x), int(mid_y)), 3)

    def show_notification(self, message: str, color=None):
        """Show notification with timeout."""
        if not color:
            color = self.COLORS['text']
        
        # Add timestamp to prevent duplicate messages
        self.notifications.append({
            'message': message,
            'color': color,
            'timestamp': pygame.time.get_ticks(),
            'duration': 3000  # Show for 3 seconds
        })

    def draw_notifications(self):
        """Draw all active notifications."""
        current_time = pygame.time.get_ticks()
        # Remove expired notifications
        self.notifications = [
            notif for notif in self.notifications 
            if current_time - notif['timestamp'] < 3000  # 3 seconds lifetime
        ]
        
        # Draw active notifications
        y_offset = 50
        for notif in self.notifications:
            text_surface = self.font.render(notif['message'], True, notif['color'])
            text_rect = text_surface.get_rect(center=(self.screen.get_width()/2, y_offset))
            self.screen.blit(text_surface, text_rect)
            y_offset += 30  # Space between notifications

    def run(self):
        """Main game loop with enhanced visuals."""
        running = True
        last_time = pygame.time.get_ticks()
        
        while running:
            current_time = pygame.time.get_ticks()
            dt = (current_time - last_time) / 1000.0
            last_time = current_time
            
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self.handle_click(event.pos)
            
            # Update
            self.fleet_manager.update(dt)
            self.update_button_states(pygame.mouse.get_pos())
            
            # Draw
            self.screen.fill(self.COLORS['background'])
            self.draw_graph()
            self.draw_robots()
            self.draw_paths()
            self.draw_buttons()
            self.draw_side_panel()  # Add side panel
            self.draw_notifications()
            
            pygame.display.flip()
            self.clock.tick(60)
        
        pygame.quit()

    def calculate_graph_transform(self):
        """Calculate scale and offset to fit graph in window."""
        x_coords = [pos[0] for pos in self.nav_graph.vertices.values()]
        y_coords = [pos[1] for pos in self.nav_graph.vertices.values()]
        
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # Add padding
        padding = 50
        width = self.screen.get_width() - 2 * padding
        height = self.screen.get_height() - 2 * padding
        
        # Calculate scale
        scale_x = width / (max_x - min_x) if max_x != min_x else 1
        scale_y = height / (max_y - min_y) if max_y != min_y else 1
        self.scale = min(scale_x, scale_y)
        
        # Calculate offset to center graph
        self.offset_x = padding + (width - (max_x - min_x) * self.scale) / 2 - min_x * self.scale
        self.offset_y = padding + (height - (max_y - min_y) * self.scale) / 2 - min_y * self.scale

    def transform_point(self, point):
        """Transform graph coordinates to screen coordinates."""
        x = point[0] * self.scale + self.offset_x
        y = point[1] * self.scale + self.offset_y
        return (x, y)

    def draw_graph(self):
        """Draw the navigation graph"""
        # Draw edges (lanes)
        for start, end in self.nav_graph.get_lanes():
            start_pos = self.nav_graph.vertices[start]
            end_pos = self.nav_graph.vertices[end]
            
            # Convert to screen coordinates
            start_screen = self.transform_point(start_pos)
            end_screen = self.transform_point(end_pos)
            
            # Draw the lane
            pygame.draw.line(self.screen, self.COLORS['lane'], 
                            start_screen, end_screen, 2)

        # Draw vertices
        for vertex_id, (x, y) in self.nav_graph.vertices.items():
            screen_x, screen_y = self.transform_point((x, y))
            
            # Draw vertex
            color = self.COLORS['charging'] if self.nav_graph.is_charging_station(vertex_id) else self.COLORS['vertex']
            pygame.draw.circle(self.screen, color, (screen_x, screen_y), 10)
            
            # Draw vertex name/ID
            font = pygame.font.Font(None, 24)
            name = self.nav_graph.get_vertex_name(vertex_id)
            text = font.render(str(name), True, self.COLORS['text'])
            self.screen.blit(text, (screen_x + 15, screen_y - 15))

    def draw_robots(self):
        """Draw robots with improved visuals"""
        for robot in self.fleet_manager.robots.values():
            screen_x, screen_y = self.transform_point(robot.position)
            
            # Draw glow effect for selected robot
            if self.selected_robot == robot.robot_id:
                pygame.draw.circle(self.screen, self.COLORS['highlight'], 
                                (screen_x, screen_y), 25, 2)

            # Draw robot body
            color = self.COLORS['robots'][robot.robot_id % len(self.COLORS['robots'])]
            pygame.draw.circle(self.screen, color, (screen_x, screen_y), 15)

            # Draw state indicator
            state_colors = {
                'idle': self.COLORS['text'],
                'moving': self.COLORS['success'],
                'waiting': self.COLORS['warning'],
                'charging': self.COLORS['charging']
            }
            pygame.draw.circle(self.screen, state_colors[robot.state], 
                             (screen_x, screen_y - 20), 5)

            # Draw robot ID
            font = pygame.font.Font(None, 24)
            text = font.render(f"R{robot.robot_id}", True, self.COLORS['text'])
            self.screen.blit(text, (screen_x - 10, screen_y - 10))

    def draw_robot_status(self, robot, screen_pos):
        """Draw robot status text at the bottom of the robot."""
        status_text = f"Status: {robot.status.value}"
        text_surf = self.font.render(status_text, True, TEXT_COLOR)
        text_rect = text_surf.get_rect(center=(screen_pos[0], screen_pos[1] + 15))
        self.screen.blit(text_surf, text_rect)

    def draw_battery_indicator(self, robot):
        """Draw battery level indicator with color gradient."""
        bar_width = 30
        bar_height = 4
        pos = robot.position
        
        # Background
        bar_rect = pygame.Rect(pos[0] - bar_width//2, pos[1] + 10, bar_width, bar_height)
        pygame.draw.rect(self.screen, GRID_COLOR, bar_rect)
        
        # Battery level with color gradient
        if robot.battery > 0:
            level_width = int(bar_width * robot.battery / 100)
            level_rect = pygame.Rect(bar_rect.left, bar_rect.top, level_width, bar_height)
            
            if robot.battery > 50:
                color = ACCENT_GREEN
            elif robot.battery > 20:
                color = ACCENT_GOLD
            else:
                color = ACCENT_RED
            
            pygame.draw.rect(self.screen, color, level_rect)

    def clear_all(self):
        """Clear all robots and reset the system"""
        self.fleet_manager.robots.clear()
        self.fleet_manager.next_robot_id = 0  # Reset robot ID counter
        self.task_queue.clear()
        self.current_paths.clear()
        self.mode = "SPAWN"
        self.buttons[0].is_active = True
        self.buttons[1].is_active = False
        self.show_notification("All robots cleared")

    def handle_auto_mode(self):
        """Handle automatic task assignment."""
        if not self.auto_mode:
            return

        current_time = pygame.time.get_ticks()
        if current_time - self.auto_timer >= self.AUTO_TASK_INTERVAL:
            self.auto_timer = current_time
            
            # Get idle robots
            idle_robots = self.fleet_manager.get_idle_robots()
            if not idle_robots:
                return

            # Get available vertices (excluding charging stations and occupied vertices)
            available_vertices = [
                v for v in self.nav_graph.vertices 
                if not self.fleet_manager.is_vertex_occupied(v) 
                and not self.nav_graph.is_charging_station(v)
            ]

            if available_vertices:
                for robot in idle_robots:
                    # Randomly select a target vertex
                    target = random.choice(available_vertices)
                    if self.fleet_manager.assign_task(target, robot.robot_id):
                        self.add_notification(f"Auto-assigned task to Robot {robot.robot_id}", "info")
                    available_vertices.remove(target)
                    if not available_vertices:
                        break

    def add_notification(self, message: str, type: str = "info"):
        """Add a new notification with proper object creation."""
        self.notifications.append(Notification(message, type))

    def draw_animated_background(self):
        """Draw animated grid background."""
        self.grid_offset = (self.grid_offset + self.grid_animation_speed) % 50
        
        # Draw grid lines
        for x in range(0, self.screen.get_width() + 50, 50):
            alpha = 128 * (1 - abs(x % 100 - 50) / 50)
            color = (*GRID_COLOR, int(alpha))
            start_pos = (x - self.grid_offset, 0)
            end_pos = (x - self.grid_offset, self.screen.get_height())
            pygame.draw.line(self.screen, color, start_pos, end_pos)
            
        for y in range(0, self.screen.get_height() + 50, 50):
            alpha = 128 * (1 - abs(y % 100 - 50) / 50)
            color = (*GRID_COLOR, int(alpha))
            start_pos = (0, y - self.grid_offset)
            end_pos = (self.screen.get_width(), y - self.grid_offset)
            pygame.draw.line(self.screen, color, start_pos, end_pos)

    def draw_vertex(self, pos: Tuple[float, float], vertex_id: int):
        """Draw a vertex with enhanced visual effects."""
        # Base circle
        pygame.draw.circle(self.graph_surface, VERTEX_COLOR, pos, 10)
        
        # Highlight for charging stations
        if self.nav_graph.is_charging_station(vertex_id):
            glow_surf = pygame.Surface((30, 30), pygame.SRCALPHA)
            pygame.draw.circle(glow_surf, (*CHARGING_COLOR, 64), (15, 15), 15)
            self.graph_surface.blit(glow_surf, 
                                  (pos[0] - 15, pos[1] - 15))
            pygame.draw.circle(self.graph_surface, CHARGING_COLOR, pos, 12, 2)

        # Vertex name
        name = self.nav_graph.get_vertex_name(vertex_id)
        text_surf = self.small_font.render(name, True, TEXT_COLOR)
        text_rect = text_surf.get_rect(center=(pos[0], pos[1] - 20))
        self.graph_surface.blit(text_surf, text_rect)

    def draw_robot(self, robot: Robot, pos: Tuple[float, float]):
        """Draw a single robot with enhanced visuals."""
        # Robot body with color based on index
        color = self.COLORS['robots'][robot.robot_id % len(self.COLORS['robots'])]
        if robot.is_charging:
            # Add charging animation
            charge_radius = 20 + math.sin(pygame.time.get_ticks() * 0.01) * 2
            pygame.draw.circle(self.screen, self.COLORS['charger'], pos, charge_radius, 2)
        
        # Draw robot
        pygame.draw.circle(self.screen, color, pos, 15)
        
        # Battery indicator with gradient
        battery_width = 30
        battery_height = 4
        battery_pos = (pos[0] - battery_width/2, pos[1] + 15)
        
        # Background
        pygame.draw.rect(self.screen, (40, 40, 40), 
                        (*battery_pos, battery_width, battery_height))
        
        # Battery level with color gradient
        if robot.battery > 0:
            level_width = int(battery_width * robot.battery / 100)
            if robot.battery > 70:
                level_color = (50, 255, 50)
            elif robot.battery > 30:
                level_color = (255, 255, 50)
            else:
                level_color = (255, 50, 50)
            pygame.draw.rect(self.screen, level_color,
                           (*battery_pos, level_width, battery_height))
        
        # Status text
        status = f"R{robot.robot_id}: {robot.state.capitalize()}"
        if robot.is_charging:
            status += f" ({robot.battery:.0f}%)"
        text_surf = self.font.render(status, True, self.COLORS['text'])
        text_rect = text_surf.get_rect(center=(pos[0], pos[1] - 25))
        self.screen.blit(text_surf, text_rect)

    def draw_paths(self):
        """Draw current robot paths with animations."""
        current_time = pygame.time.get_ticks()
        
        for robot_id, path in self.current_paths.items():
            if len(path) < 2:
                continue
            
            robot_color = self.COLORS['robots'][robot_id % len(self.COLORS['robots'])]
            
            # Draw path segments
            for i in range(len(path) - 1):
                start_pos = self.transform_point(self.nav_graph.vertices[path[i]])
                end_pos = self.transform_point(self.nav_graph.vertices[path[i + 1]])
                
                # Animated dash effect
                dash_length = 10
                dash_space = 5
                total_length = dash_length + dash_space
                offset = (current_time * 0.1) % total_length
                
                # Draw dashed line with alpha
                color_with_alpha = (*robot_color[:3], 180)
                draw_dashed_line(self.screen, color_with_alpha, 
                               start_pos, end_pos, 
                               dash_length, dash_space, offset)

    def draw_mode_indicator(self):
        """Draw mode indicator at the top of the screen."""
        mode_text = f"Mode: {self.mode}"
        text_surf = self.font.render(mode_text, True, self.COLORS['text'])
        text_rect = text_surf.get_rect(center=(self.screen.get_width() // 2, 10))
        self.screen.blit(text_surf, text_rect)

    def update(self):
        """Update GUI state."""
        mouse_pos = pygame.mouse.get_pos()
        
        # Update button hover states
        for button in self.buttons:
            button.is_hovered = button.rect.collidepoint(mouse_pos)
            
        # Update path preview in task mode
        if self.mode == "TASK" and self.selected_robot:
            hover_vertex = self.find_clicked_vertex(mouse_pos)
            if hover_vertex is not None:
                self.preview_path = self.nav_graph.get_shortest_path(
                    self.selected_robot.current_vertex,
                    hover_vertex
                )
            else:
                self.preview_path = None

    def update_graph_surface(self):
        """Create cached surface for graph visualization."""
        self.graph_surface = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        
    def save_fleet(self):
        """Save the fleet to a file."""
        # Implementation of save_fleet method
        pass

    def load_fleet(self):
        """Load the fleet from a file."""
        # Implementation of load_fleet method
        pass

    def reset_system(self):
        """Clear all robots and reset counters."""
        self.fleet_manager.clear_all_robots()
        self.fleet_manager.next_robot_id = 0  # Reset robot ID counter
        self.task_queue.clear()
        self.current_paths.clear()
        self.mode = "SPAWN"
        self.buttons[0].is_active = True
        self.buttons[1].is_active = False
        self.show_notification("System reset - all robots cleared")

    def update_button_states(self, mouse_pos):
        """Update hover and active states for all buttons."""
        for button in self.buttons:
            # Update hover state
            button.is_hovered = button.rect.collidepoint(mouse_pos)
            
            # Update active state based on current mode
            if button.text == "SPAWN MODE":
                button.is_active = (self.mode == "SPAWN")
            elif button.text == "TASK MODE":
                button.is_active = (self.mode == "TASK")
            elif button.text == "CLEAR ALL":
                button.is_active = False  # Clear button is never active

    def draw_buttons(self):
        """Draw all GUI buttons."""
        for button in self.buttons:
            # Draw button background
            color = button.hover_color if button.is_hovered else button.color
            if button.is_active:
                color = tuple(min(c + 50, 255) for c in button.color)  # Brighter when active
            
            pygame.draw.rect(self.screen, color, button.rect)
            pygame.draw.rect(self.screen, self.COLORS['text'], button.rect, 2)  # Border
            
            # Draw button text
            text_surface = self.font.render(button.text, True, self.COLORS['text'])
            text_rect = text_surface.get_rect(center=button.rect.center)
            self.screen.blit(text_surface, text_rect)

    def handle_click(self, pos: Tuple[int, int]):
        """Handle mouse click events."""
        # Check button clicks first
        for button in self.buttons:
            if button.rect.collidepoint(pos):
                if button.text == "SPAWN MODE":
                    self.mode = "SPAWN"
                    self.task_queue.clear()
                    self.show_notification("Spawn Mode: Click vertices to spawn robots")
                    return
                elif button.text == "TASK MODE":
                    self.mode = "TASK"
                    self.task_queue = list(self.fleet_manager.robots.values())
                    self.task_queue.sort(key=lambda x: x.robot_id)  # Sort by robot ID
                    if self.task_queue:
                        self.show_notification(f"Task Mode: Assign destination for Robot {self.task_queue[0].robot_id}")
                    else:
                        self.show_notification("No robots to assign tasks to!", self.COLORS['warning'])
                    return
                elif button.text == "CLEAR ALL":
                    self.fleet_manager.clear_all_robots()
                    self.task_queue.clear()
                    self.mode = "SPAWN"
                    self.show_notification("All robots cleared")
                    return

        # Handle vertex clicks
        vertex_id = self.find_clicked_vertex(pos)
        if vertex_id is not None:
            if self.mode == "SPAWN":
                # Try to spawn a robot
                if self.fleet_manager.spawn_robot(vertex_id):
                    self.show_notification(f"Robot spawned at vertex {vertex_id}")
                else:
                    self.show_notification("Cannot spawn robot - vertex occupied", self.COLORS['warning'])
            elif self.mode == "TASK" and self.task_queue:
                # Handle task assignment
                self.handle_task_assignment(vertex_id)

    def find_clicked_vertex(self, screen_pos: Tuple[float, float]) -> Optional[int]:
        """Find vertex near clicked position."""
        # Convert screen coordinates to graph coordinates
        graph_pos = self.inverse_transform_point(screen_pos)
        
        # Check each vertex
        click_radius = 20 / self.scale  # Adjust click radius based on zoom level
        for vertex_id, pos in self.nav_graph.vertices.items():
            dx = graph_pos[0] - pos[0]
            dy = graph_pos[1] - pos[1]
            # Check if click is within radius of vertex
            if (dx * dx + dy * dy) <= click_radius * click_radius:
                return vertex_id
        
        return None

    def inverse_transform_point(self, screen_pos: Tuple[float, float]) -> Tuple[float, float]:
        """Convert screen coordinates to graph coordinates."""
        x = (screen_pos[0] - self.offset_x) / self.scale
        y = (screen_pos[1] - self.offset_y) / self.scale
        return (x, y)

    def draw_side_panel(self):
        """Draw enhanced side panel with box design."""
        # Panel background with border
        panel_rect = pygame.Rect(self.main_surface_width, 0, self.panel_width, self.screen.get_height())
        pygame.draw.rect(self.screen, self.COLORS['panel'], panel_rect)
        pygame.draw.rect(self.screen, self.COLORS['panel_border'], panel_rect, 2)

        # Header box
        header_box = pygame.Rect(self.main_surface_width + 10, 10, self.panel_width - 20, 40)
        pygame.draw.rect(self.screen, self.COLORS['box_bg'], header_box)
        pygame.draw.rect(self.screen, self.COLORS['box_border'], header_box, 2)
        
        header = self.large_font.render("Robot Status", True, self.COLORS['text'])
        header_rect = header.get_rect(center=header_box.center)
        self.screen.blit(header, header_rect)

        # Robot information boxes
        y_offset = 70
        for robot in sorted(self.fleet_manager.robots.values(), key=lambda r: r.robot_id):
            # Robot box
            box_height = 80
            box_rect = pygame.Rect(self.main_surface_width + 10, y_offset, 
                                 self.panel_width - 20, box_height)
            pygame.draw.rect(self.screen, self.COLORS['box_bg'], box_rect)
            pygame.draw.rect(self.screen, self.COLORS['box_border'], box_rect, 2)

            # Robot color indicator
            color = self.COLORS['robots'][robot.robot_id % len(self.COLORS['robots'])]
            if robot.is_dead:
                color = self.COLORS['critical']
            elif robot.is_charging:
                color = self.COLORS['charging']

            # Robot ID and state
            status_text = f"Robot {robot.robot_id}: {robot.state}"
            if robot.is_dead:
                status_text += " (DEAD)"
            status_surf = self.font.render(status_text, True, color)
            self.screen.blit(status_surf, (box_rect.x + 10, box_rect.y + 10))

            # Battery bar with gradient
            bar_width = box_rect.width - 20
            bar_height = 15
            bar_x = box_rect.x + 10
            bar_y = box_rect.y + 35

            # Background
            pygame.draw.rect(self.screen, self.COLORS['grid'], 
                           (bar_x, bar_y, bar_width, bar_height))

            # Battery level with gradient
            if robot.battery > 0:
                level_width = int(bar_width * robot.battery / 100)
                if robot.battery > 70:
                    color = (50, 255, 50)
                elif robot.battery > 30:
                    color = (255, 255, 50)
                else:
                    color = (255, 50, 50)
                pygame.draw.rect(self.screen, color,
                               (bar_x, bar_y, level_width, bar_height))

            # Battery percentage
            batt_text = f"{robot.battery:.1f}%"
            batt_surf = self.font.render(batt_text, True, self.COLORS['text'])
            batt_rect = batt_surf.get_rect(midright=(box_rect.right - 10, bar_y + bar_height/2))
            self.screen.blit(batt_surf, batt_rect)

            # Additional status info
            if robot.target_vertex is not None:
                dest_text = f"→ Node {robot.target_vertex}"
                dest_surf = self.font.render(dest_text, True, self.COLORS['text'])
                self.screen.blit(dest_surf, (box_rect.x + 10, box_rect.y + 55))

            y_offset += box_height + 10

    def handle_dead_robot(self, robot_id: int):
        """Handle dead robot with user prompt."""
        if self.fleet_manager.robots[robot_id].dead:
            # Create prompt box
            prompt_box = pygame.Rect(
                self.screen.get_width()//2 - 200,
                self.screen.get_height()//2 - 100,
                400,
                200
            )
            
            pygame.draw.rect(self.screen, self.COLORS['box_bg'], prompt_box)
            pygame.draw.rect(self.screen, self.COLORS['critical'], prompt_box, 2)
            
            # Show prompt text
            text = f"Robot {robot_id} has died! Battery depleted."
            text_surf = self.font.render(text, True, self.COLORS['text'])
            text_rect = text_surf.get_rect(center=(prompt_box.centerx, prompt_box.centery - 40))
            self.screen.blit(text_surf, text_rect)
            
            # Add replace/remove buttons
            replace_btn = Button(prompt_box.centerx - 100, prompt_box.bottom - 50, 
                               80, 30, "Replace", self.COLORS['charging'], self.replace_robot)
            remove_btn = Button(prompt_box.centerx + 20, prompt_box.bottom - 50,
                              80, 30, "Remove", self.COLORS['warning'], self.remove_robot)
            
            replace_btn.draw(self.screen)
            remove_btn.draw(self.screen)

    def show_path_conflict_notification(self, robot_id: int, conflict_point: int):
        """Show detailed conflict notification."""
        notification_box = pygame.Rect(
            self.screen.get_width() // 4,
            self.screen.get_height() // 4,
            400,
            150
        )
        
        pygame.draw.rect(self.screen, self.COLORS['box_bg'], notification_box)
        pygame.draw.rect(self.screen, self.COLORS['warning'], notification_box, 2)
        
        text_lines = [
            f"Path Conflict Detected for Robot {robot_id}",
            "Cannot take shortest path due to potential collision",
            "Attempting alternative route...",
            "Press SPACE to continue"
        ]
        
        y_offset = notification_box.y + 20
        for line in text_lines:
            text_surf = self.font.render(line, True, self.COLORS['text'])
            text_rect = text_surf.get_rect(centerx=notification_box.centerx, y=y_offset)
            self.screen.blit(text_surf, text_rect)
            y_offset += 30

    def get_blocked_vertices(self, current_robot_id: int) -> Set[int]:
        """Get set of vertices blocked by other robots."""
        blocked = set()
        
        for robot in self.fleet_manager.robots.values():
            if robot.robot_id != current_robot_id:
                # Add current position of other robots
                blocked.add(robot.current_vertex)
                
                # Add target positions of moving robots
                if robot.state == "moving" and robot.target_vertex is not None:
                    blocked.add(robot.target_vertex)
                    
                # Add vertices along other robots' paths
                if robot.robot_id in self.current_paths:
                    path = self.current_paths[robot.robot_id]
                    blocked.update(path[:-1])  # Add all but destination
        
        return blocked

    def toggle_mode(self):
        """Toggle between spawn mode and task mode"""
        self.mode = "SPAWN" if self.mode == "TASK" else "TASK"
        self.selected_robot = None
        
        # Update button colors based on mode
        self.buttons[0].color = self.COLORS['success'] if self.mode == "SPAWN" else self.COLORS['button']
        self.buttons[1].color = self.COLORS['success'] if self.mode == "TASK" else self.COLORS['button']
        
        mode_str = "SPAWN" if self.mode == "SPAWN" else "TASK"
        self.logger.info(f"Switched to {mode_str} mode")

def draw_dashed_line(surface, color, start_pos, end_pos, dash_length, dash_space, offset=0):
    """Draw a dashed line between two points."""
    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]
    distance = math.sqrt(dx * dx + dy * dy)
    
    if distance == 0:
        return
    
    # Normalize direction
    dx /= distance
    dy /= distance
    
    # Calculate number of segments
    dash_total = dash_length + dash_space
    num_dashes = int(distance / dash_total)
    
    # Draw dashes
    curr_pos = list(start_pos)
    for i in range(num_dashes):
        start_dash = i * dash_total + offset
        end_dash = start_dash + dash_length
        
        if end_dash > distance:
            end_dash = distance
        
        # Calculate dash endpoints
        dash_start = (
            start_pos[0] + dx * start_dash,
            start_pos[1] + dy * start_dash
        )
        dash_end = (
            start_pos[0] + dx * end_dash,
            start_pos[1] + dy * end_dash
        )
        
        # Draw dash segment
        pygame.draw.line(surface, color, dash_start, dash_end, 2) 