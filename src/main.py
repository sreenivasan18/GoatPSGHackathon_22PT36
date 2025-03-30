import sys
from pathlib import Path
import json
import logging

# Add src directory to Python path
src_dir = Path(__file__).parent.parent
sys.path.append(str(src_dir))

from src.models.nav_graph import NavigationGraph
from src.controllers.fleet_manager import FleetManager
from src.gui.fleet_gui import FleetGUI

def setup_logging():
    """Configure logging system."""
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)
    
    logging.basicConfig(
        filename=log_dir / "fleet_logs.txt",
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

def load_nav_graph(file_path: str) -> NavigationGraph:
    """Load navigation graph from JSON file"""
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
            
            vertices = []
            lanes = []
            
            # Get the first (and only) level from the levels dictionary
            level_data = next(iter(data['levels'].values()))
            
            # Extract vertices
            if 'vertices' in level_data:
                vertices = level_data['vertices']
            
            # Extract lanes - only take the vertex indices, ignore speed_limit
            if 'lanes' in level_data:
                for lane in level_data['lanes']:
                    if len(lane) >= 2:  # Lane has at least source and target vertices
                        lanes.append([lane[0], lane[1]])  # Only take the vertex indices
            
            print(f"Found {len(vertices)} vertices and {len(lanes)} lanes")
            
            if not vertices:
                raise ValueError("No valid vertices found in graph data")
                
            graph_data = {
                'vertices': vertices,
                'lanes': lanes
            }
            
            return NavigationGraph(graph_data)
                
    except FileNotFoundError:
        raise FileNotFoundError(f"Navigation graph file not found: {file_path}")
    except json.JSONDecodeError:
        raise ValueError(f"Invalid JSON format in file: {file_path}")
    except Exception as e:
        print(f"Error while processing data: {str(e)}")
        raise ValueError(f"Error processing navigation graph: {str(e)}")

def main():
    """Main application entry point."""
    setup_logging()
    
    # Load navigation graph
    nav_graph_path = Path("data/nav_graph.json")
    nav_graph = load_nav_graph(nav_graph_path)
    
    # Initialize managers and GUI
    fleet_manager = FleetManager(nav_graph)
    gui = FleetGUI(nav_graph, fleet_manager)
    
    # Start the application
    logging.info("Starting Fleet Management System")
    gui.run()

if __name__ == "__main__":
    main() 