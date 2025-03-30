import json
import networkx as nx
from typing import Dict, List, Tuple, Optional, Set
import numpy as np
import logging
import math

class NavigationGraph:
    def __init__(self, graph_data: Dict):
        self.vertices = {}
        self.vertex_names = {}
        self.charging_stations = set()
        self.graph = nx.Graph()
        self._load_graph(graph_data)

    def _load_graph(self, graph_data: Dict):
        """Load graph from JSON data"""
        try:
            vertices = graph_data.get('vertices', [])
            lanes = graph_data.get('lanes', [])

            # Load vertices with their attributes
            for idx, vertex in enumerate(vertices):
                if len(vertex) >= 2:  # Must have at least x,y coordinates
                    x, y = float(vertex[0]), float(vertex[1])
                    self.vertices[idx] = (x, y)
                    self.graph.add_node(idx, pos=(x, y))
                    
                    # Handle vertex attributes if present
                    if len(vertex) > 2 and isinstance(vertex[2], dict):
                        attrs = vertex[2]
                        # Store vertex name if present
                        if 'name' in attrs:
                            self.vertex_names[idx] = attrs['name']
                        # Check for charging station
                        if attrs.get('is_charger', False):
                            self.charging_stations.add(idx)

            # Load lanes
            for lane in lanes:
                if len(lane) >= 2:  # Must have source and target vertices
                    v1, v2 = int(lane[0]), int(lane[1])
                    if v1 in self.vertices and v2 in self.vertices:
                        self.graph.add_edge(v1, v2, 
                                         weight=self._calculate_distance(v1, v2))

            if not self.vertices:
                raise ValueError("No valid vertices found in graph data")

        except (KeyError, IndexError, TypeError, ValueError) as e:
            raise ValueError(f"Invalid graph data format: {str(e)}")

    def _calculate_distance(self, v1: int, v2: int) -> float:
        """Calculate Euclidean distance between two vertices"""
        if v1 not in self.vertices or v2 not in self.vertices:
            return float('inf')
        x1, y1 = self.vertices[v1]
        x2, y2 = self.vertices[v2]
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def get_path_to_vertex(self, start: int, end: int, blocked_vertices: Set[int] = None) -> List[int]:
        """Get best available path considering obstacles and battery"""
        if blocked_vertices is None:
            blocked_vertices = set()

        try:
            # Try shortest path first
            shortest_path = nx.astar_path(
                self.graph,
                start,
                end,
                heuristic=self._manhattan_distance,
                weight=lambda u, v, d: float('inf') if u in blocked_vertices or v in blocked_vertices else d['weight']
            )
            if not any(v in blocked_vertices for v in shortest_path[1:-1]):
                return shortest_path
        except nx.NetworkXNoPath:
            pass

        # Try alternative paths
        all_paths = []
        for path in nx.all_simple_paths(self.graph, start, end, cutoff=15):  # Limit path length
            if not any(v in blocked_vertices for v in path[1:-1]):
                all_paths.append(path)

        if all_paths:
            # Return shortest valid alternative path
            return min(all_paths, key=len)
        return []

    def find_nearest_charger(self, vertex: int, blocked_vertices: Set[int] = None) -> Optional[int]:
        """Find nearest available charging station"""
        if blocked_vertices is None:
            blocked_vertices = set()

        min_distance = float('inf')
        nearest_charger = None

        for charger in self.charging_stations:
            if charger in blocked_vertices:
                continue
            try:
                path = self.get_path_to_vertex(vertex, charger, blocked_vertices)
                if path and len(path) < min_distance:
                    min_distance = len(path)
                    nearest_charger = charger
            except nx.NetworkXNoPath:
                continue

        return nearest_charger

    def get_all_possible_paths(self, start: int, end: int) -> List[List[int]]:
        """Get all possible paths between two vertices."""
        def dfs_paths(current, target, visited, path):
            path = path + [current]
            if current == target:
                paths.append(path)
            else:
                for neighbor in self.graph.neighbors(current):
                    if neighbor not in visited:
                        dfs_paths(neighbor, target, visited | {current}, path)
        
        paths = []
        dfs_paths(start, end, set(), [])
        return paths

    def get_longest_path(self, start: int, end: int, blocked_vertices: Set[int] = None) -> List[int]:
        """Find the longest safe path between two vertices."""
        if blocked_vertices is None:
            blocked_vertices = set()

        # Get all possible paths
        all_paths = self.get_all_possible_paths(start, end)
        
        # Filter valid paths (avoiding blocked vertices)
        valid_paths = [
            path for path in all_paths 
            if not any(v in blocked_vertices for v in path[1:-1])
        ]
        
        if not valid_paths:
            return []
        
        # Return the longest valid path
        return max(valid_paths, key=len)

    def get_shortest_path(self, start: int, end: int, blocked_vertices: Set[int] = None) -> List[int]:
        """Find shortest path avoiding blocked vertices."""
        if blocked_vertices is None:
            blocked_vertices = set()
        
        def cost_function(u, v, edge_data):
            if v in blocked_vertices:
                return float('inf')
            return 1.0

        try:
            return nx.astar_path(
                self.graph,
                start,
                end,
                heuristic=self._manhattan_distance,
                weight=cost_function
            )
        except nx.NetworkXNoPath:
            return []

    def _manhattan_distance(self, u: int, v: int) -> float:
        """Manhattan distance heuristic for A*."""
        pos_u = self.vertices[u]
        pos_v = self.vertices[v]
        return abs(pos_u[0] - pos_v[0]) + abs(pos_u[1] - pos_v[1])

    def get_vertex_name(self, vertex_id: int) -> str:
        """Get the name of a vertex if it exists"""
        return self.vertex_names.get(vertex_id, str(vertex_id))

    def is_charging_station(self, vertex_id: int) -> bool:
        """Check if a vertex is a charging station"""
        return vertex_id in self.charging_stations

    def get_nearest_charger(self, vertex_id: int) -> Optional[int]:
        """Find nearest charging station to given vertex."""
        if not self.charging_stations:
            return None
            
        min_dist = float('inf')
        nearest_charger = None
        
        for charger in self.charging_stations:
            try:
                path = nx.shortest_path_length(
                    self.graph, 
                    vertex_id, 
                    charger, 
                    weight="weight"
                )
                if path < min_dist:
                    min_dist = path
                    nearest_charger = charger
            except nx.NetworkXNoPath:
                continue
                
        return nearest_charger

    def get_vertex_position(self, vertex_id: int) -> Optional[Tuple[float, float]]:
        """Get vertex position with validation."""
        if not self.validate_vertex_id(vertex_id):
            logging.warning(f"Invalid vertex ID: {vertex_id}")
            return None
        return self.vertices[vertex_id]
    
    def get_vertex_attributes(self, vertex_id: int) -> Dict:
        """Get the attributes of a vertex by its ID."""
        if vertex_id < 0 or vertex_id >= len(self.vertices):
            raise ValueError(f"Invalid vertex ID: {vertex_id}")
        return self.vertices[vertex_id][2]
    
    def get_lane_speed_limit(self, start: int, end: int) -> float:
        """Get the speed limit of a lane."""
        if self.graph.has_edge(start, end):
            return self.graph[start][end].get('speed_limit', 0)
        return 0
    
    def get_vertex_neighbors(self, vertex_id: int) -> List[int]:
        """Get all neighbors of a vertex."""
        return list(self.graph.neighbors(vertex_id))
    
    def get_graph_bounds(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Get the minimum and maximum coordinates of the graph."""
        x_coords = [pos[0] for pos in self.vertices.values()]
        y_coords = [pos[1] for pos in self.vertices.values()]
        
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        return (min_x, min_y), (max_x, max_y)
    
    def get_vertex_count(self) -> int:
        """Get the total number of vertices in the graph."""
        return len(self.vertices)
    
    def get_lane_count(self) -> int:
        """Get the total number of lanes in the graph."""
        return len(self.lanes)

    def update_traffic_weight(self, start: int, end: int, weight: float):
        """Update traffic weight for path finding."""
        self.traffic_weights[(start, end)] = weight
        
    def get_alternative_paths(self, start: int, end: int, num_paths: int = 3) -> List[List[int]]:
        """Find multiple alternative paths for better traffic distribution."""
        paths = []
        temp_graph = self.graph.copy()
        
        for _ in range(num_paths):
            try:
                path = nx.astar_path(temp_graph, start, end,
                                   heuristic=self._manhattan_distance,
                                   weight='weight')
                paths.append(path)
                
                # Increase weights along this path to find alternatives
                for i in range(len(path) - 1):
                    temp_graph[path[i]][path[i+1]]['weight'] *= 1.5
                    
            except nx.NetworkXNoPath:
                break
                
        return paths 

    def validate_vertex_id(self, vertex_id: int) -> bool:
        """Validate vertex ID exists."""
        return vertex_id in self.vertices 

    def get_alternative_path(self, start: int, end: int, blocked_vertices: Set[int]) -> List[int]:
        """Find alternative path avoiding blocked vertices."""
        if start == end:
            return []
        
        def cost_function(u, v, edge_data):
            """Cost function that avoids blocked vertices."""
            if v in blocked_vertices:
                return float('inf')
            # Try to find longer but valid paths
            return 1.0 / (edge_data.get('weight', 1.0) + 0.1)

        try:
            # Try all possible paths
            paths = list(nx.all_simple_paths(self.graph, start, end))
            if not paths:
                return []
            
            # Filter paths that go through blocked vertices
            valid_paths = [
                path for path in paths 
                if not any(v in blocked_vertices for v in path[1:-1])
            ]
            
            if not valid_paths:
                return []
            
            # Return the longest valid path
            return max(valid_paths, key=len)
        
        except nx.NetworkXNoPath:
            return [] 

    def get_lanes(self) -> List[Tuple[int, int]]:
        """Get all lanes in the graph"""
        return list(self.graph.edges()) 