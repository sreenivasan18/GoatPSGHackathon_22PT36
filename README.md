# 🚀 Fleet Management System with Traffic Negotiation

A **Python-based Multi-Robot Navigation & Collision Avoidance System**

---

## 📖 Table of Contents

- [✨ Features](#-features)
- [🔧 How It Works](#-how-it-works)
- [⚙️ Tech Stack](#%EF%B8%8F-tech-stack)
- [🛠 Setup](#-setup)
- [🎮 GUI Interactions](#-gui-interactions)
- [❌ What’s Missing?](#-whats-missing)
- [🚀 Future Enhancements](#-future-enhancements)
- [📸 Screenshots](#-screenshots)
- [🎯 Conclusion](#-conclusion)

---

## ✨ Features

✅ **Interactive Pygame GUI**
- Visualize **robots, vertices, lanes, and charging stations**.
- Click to **spawn robots** or **assign tasks**.

✅ **Real-Time Robot Management**
- Unique **robot colors & IDs**.
- Displays **battery levels & statuses** (moving, waiting, charging).

✅ **Traffic Negotiation**
- A* pathfinding with **collision checks (TrafficManager)**.
- **Alternative path calculation** if shortest path is blocked.

✅ **Logging**
- Every action **logged to fleet_logs.txt** (spawning, task completion).

---

## 🔧 How It Works

### 1️⃣ Navigation Graph (`nav_graph.py`)
- Parses `nav_graph.json` into a **weighted graph** (vertices & lanes).
- Uses **A*** for shortest-path calculations (**Manhattan heuristic**).
- Uses **Depth-First Search (DFS)** to find alternate paths when an existing robot occupies a line segment, preventing **path collisions**.
- Prioritizes **charging stations** for low-battery robots.

### 2️⃣ Robot Behavior (`robot.py`)
- Each robot has:
  - **Unique ID & color**
  - **Battery drain while moving**
  - **State machine** (idle, moving, charging, waiting)
- Smooth movement **interpolation between vertices**.

### 3️⃣ Traffic Manager (`traffic_manager.py`)
- **Tracks vertex/lane occupancy**.
- Detects **path conflicts & deadlocks** (*partial implementation*).

### 4️⃣ Fleet Manager (`fleet_manager.py`)
- Spawns **robots, assigns tasks, manages queues**.
- Optimizes **task assignment based on battery & distance**.

### 5️⃣ GUI (`fleet_gui.py`)
- **Left-Click**: Spawn robot (*Spawn Mode*).
- **Right-Click**: Assign task (*Task Mode*).
- **Side panel** shows robot status, battery, and system metrics.

---

## ⚙️ Tech Stack

| Package    | Use Case |
|------------|---------|
| Pygame     | Interactive GUI visualization |
| NetworkX   | Graph traversal (**A* algorithm**) |
| Logging    | Detailed activity logs |
| JSON       | Parse navigation graph |

---

## 🛠 Setup

### 1️⃣ Install Dependencies
```bash
# requirements.txt
pygame==2.5.0
networkx==3.2.1
numpy==1.26.0
```
Run:
```bash
pip install -r requirements.txt
```

### 2️⃣ Run the System
```bash
python src/main.py
```

---

## 🎮 GUI Interactions

| Action        | Effect  |
|--------------|---------|
| **Spawn Mode** | Click vertices to spawn robots |
| **Task Mode**  | Click robot → destination to assign task |
| **Clear All**  | Reset all robots and tasks |

---

## ❌ What’s Missing?

🔴 **No Dynamic Rerouting**: Robots don’t adjust paths mid-movement if blocked.

🔴 **Limited Deadlock Handling**: Deadlocks are detected but not resolved visually.

🔴 **Battery Alerts**: No pop-ups for critical battery levels.

🔴 **Occupancy Highlights**: Blocked lanes/vertices aren’t color-coded.


---

## 🚀 Future Enhancements

### 1️⃣ ROS (Robot Operating System)
**Why?** Enables real-time **multi-robot communication & hardware integration**.

✅ **Use Case:**
- Replace `TrafficManager` with **ROS nodes** for decentralized traffic control.
- Simulate robots in **Gazebo** for realistic physics.

❌ **Blocker:** ROS requires **Linux/Python 2.7 compatibility**; time-consuming to integrate.

### 2️⃣ Pybotics
**Why?** For **industrial robot kinematics** (e.g., precise arm movements).

✅ **Use Case:**
- Extend robots to **handle manipulators** (e.g., loading cargo).

❌ **Blocker:** Overkill for **2D navigation**; focus was on fleet logistics.

### 3️⃣ Robot Framework
**Why?** For **automated testing** of task assignments.

✅ **Use Case:**
- Validate **100+ robots navigating simultaneously**.

❌ **Blocker:** Needed more time to **write test suites**.

---

| Spawn Mode | Task Mode |
|------------|-----------|
| Spawn robots by clicking vertices. | Assign tasks by selecting **robot → destination**. |

---

## 🎯 Conclusion

This system successfully demonstrates **multi-robot pathfinding** but would scale better with **ROS/Pybotics**. The **GUI is intuitive**, but real-world deployments need **dynamic rerouting & hardware integration**.

⭐ **Star this repo if you loved it!** ⭐

