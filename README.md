# 🚀 Fleet Management System with Traffic Negotiation  
*A Python-based Multi-Robot Navigation & Collision Avoidance System*  

<div align="center">
  <img src="https://img.shields.io/badge/Python-3.8+-blue?logo=python" alt="Python">
  <img src="https://img.shields.io/badge/Pygame-2.5.0-green?logo=pygame" alt="Pygame">
  <img src="https://img.shields.io/badge/NetworkX-3.2.1-red?logo=networkx" alt="NetworkX">
  <img src="https://img.shields.io/badge/Status-Active-brightgreen" alt="Status">
</div>

## 📌 Table of Contents  
- [✨ Features](#-features)  
- [🔧 How It Works](#-how-it-works)  
- [⚙️ Tech Stack](#-tech-stack)  
- [🛠 Setup](#-setup)  
- [🎮 GUI Interactions](#-gui-interactions)  
- [❌ What's Missing?](#-whats-missing)  
- [🚀 Future Enhancements](#-future-enhancements)  
- [📸 Screenshots](#-screenshots)  
- [🎯 Conclusion](#-conclusion)  

## ✨ Features  

<div align="center">
  <table>
    <tr>
      <td><strong>✅ Interactive Pygame GUI</strong></td>
      <td>Visualize robots, vertices, lanes, and charging stations with click-to-spawn functionality</td>
    </tr>
    <tr>
      <td><strong>✅ Real-Time Management</strong></td>
      <td>Track unique robots with color-coded IDs and battery status indicators</td>
    </tr>
    <tr>
      <td><strong>✅ Smart Navigation</strong></td>
      <td>A* pathfinding with collision avoidance and alternative routing</td>
    </tr>
    <tr>
      <td><strong>✅ Detailed Logging</strong></td>
      <td>Comprehensive activity tracking in fleet_logs.txt</td>
    </tr>
  </table>
</div>

## 🔧 How It Works  

### System Architecture
```mermaid
graph TD
    A[GUI] --> B[Fleet Manager]
    B --> C[Robot Controller]
    B --> D[Traffic Manager]
    C --> E[Individual Robots]
    D --> F[Navigation Graph]

Core Components

    Navigation Graph (nav_graph.py)

        Parses JSON environment definition

        Implements A* algorithm with Manhattan heuristic

        Identifies charging stations

    Robot Behavior (robot.py)
    python
    Copy

    class Robot:
        def __init__(self):
            self.id = uuid4()
            self.battery = 100.0
            self.state = "idle"  # States: idle/moving/charging/waiting

    Traffic Management (traffic_manager.py)

        Real-time occupancy tracking

        Deadlock detection system

    Fleet Manager (fleet_manager.py)

        Central coordination hub

        Task optimization algorithms

⚙️ Tech Stack
<div align="center"> <table> <tr> <th>Component</th> <th>Technology</th> <th>Version</th> </tr> <tr> <td>GUI Framework</td> <td>Pygame</td> <td>2.5.0</td> </tr> <tr> <td>Pathfinding</td> <td>NetworkX</td> <td>3.2.1</td> </tr> <tr> <td>Math Operations</td> <td>NumPy</td> <td>1.26.0</td> </tr> </table> </div>
🛠 Setup
Installation
bash
Copy

# Clone repository
git clone https://github.com/yourusername/fleet-management-system.git
cd fleet-management-system

# Install dependencies
pip install -r requirements.txt

Execution
bash
Copy

python src/main.py

🎮 GUI Interactions
<div align="center"> <table> <tr> <th>Interaction</th> <th>Effect</th> <th>Visual Feedback</th> </tr> <tr> <td>Left-Click Vertex</td> <td>Spawns new robot</td> <td>Color-coded robot appears</td> </tr> <tr> <td>Right-Click Sequence</td> <td>Assigns navigation task</td> <td>Path visualization appears</td> </tr> <tr> <td>Clear Button</td> <td>System reset</td> <td>All robots removed</td> </tr> </table> </div>
❌ What's Missing?
Issue	Impact	Priority
Dynamic Rerouting	Robots get stuck when paths become blocked	High
Visual Deadlocks	Difficult to diagnose traffic jams	Medium
Battery Alerts	Critical failures may go unnoticed	High
Occupancy Visuals	Hard to see blocked areas	Medium
🚀 Future Enhancements
ROS Integration Roadmap
mermaid
Copy

gantt
    title ROS Integration Timeline
    dateFormat  YYYY-MM-DD
    section Phase 1
    Environment Setup     :2023-11-01, 14d
    Basic Communication  :2023-11-15, 21d
    section Phase 2
    Hardware Integration :2023-12-06, 28d
    Testing              :2024-01-03, 14d

Why Not Implemented Now?

    Requires significant Linux/Python 2.7 adaptation

    Would fundamentally change architecture

    Time constraints for hackathon delivery

📸 Screenshots
<div align="center"> <img src="https://via.placeholder.com/600x400/2d3748/ffffff?text=Spawn+Mode" width="45%" alt="Spawn Mode"> <img src="https://via.placeholder.com/600x400/2d3748/ffffff?text=Task+Assignment" width="45%" alt="Task Mode"> </div>
🎯 Conclusion

This system successfully demonstrates:

    Multi-agent pathfinding

    Basic collision avoidance

    Interactive fleet management

Next Steps:

    Implement ROS for production deployment

    Add emergency stop protocols

    Develop web-based monitoring dashboard

<div align="center"> <p>🌟 <strong>Star this repository</strong> if you found it useful! 🌟</p> <p>🐛 <strong>Report issues</strong> to help improve the project</p> </div> ```
