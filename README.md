# 🏭 NXP AIM India — Warehouse Robotics Autonomy Layer (ROS 2)

## Overview

This repository contains the **autonomy, perception, and decision-making layer** developed as part of the **NXP AIM India Warehouse Robotics Challenge**.

The code here is **not a full robot stack**.
It is designed to **plug into a pre-existing, proprietary robot platform** provided by the competition environment (Cognipilot / NXP).

> **Scope of this repository:**
> High-level autonomy logic built on top of an already running robot, SLAM system, and Nav2 stack.

---

## 🧠 System Architecture (Layered Design)

The overall system is intentionally divided into **three layers**, mirroring how real-world robotics systems are built in industry.

---

### 🧱 Layer 1 — Robot Platform (Provided / Proprietary)

**Not part of this repository**

This layer is supplied by the competition environment and includes:

* Robot URDF and physical model
* Sensor drivers (LiDAR, camera, IMU)
* TF tree and `robot_state_publisher`
* SLAM system publishing `/map`
* Nav2 stack exposing `/navigate_to_pose`
* Simulation or real robot bringup

Typical outputs from this layer:

* `/map` (`nav_msgs/OccupancyGrid`)
* `/amcl_pose`
* `/camera/image_raw/compressed`
* `/tf`
* Nav2 action server (`NavigateToPose`)

🚫 **This repository does NOT modify or launch this layer** due to proprietary constraints.

---

### 🧠 Layer 2 — Autonomy & Decision Logic (This Repository)

**This is the focus of the project.**

This layer consumes existing robot data and implements:

* Frontier-based exploration
* SLAM-map-based shelf detection using OpenCV
* Finite State Machine (FSM):
  * `EXPLORATION → NAVIGATION_TO_SHELVES`
* Shelf pose estimation and orientation handling
* Goal generation for object viewing and QR scanning
* Sequential Nav2 goal execution
* QR code detection logic
* Robust goal lifecycle management

This layer assumes:

* The robot already exists
* SLAM and Nav2 are already running
* Sensors are already publishing data

---

### 🧩 Layer 3 — Visualization & Debugging (Optional)

Used only for validation and debugging:

* RViz / Foxglove visualization
* Debug image publishing
* PoseArray visualization for detected shelves
* Logging and runtime introspection

This layer is optional but helpful for development and demonstrations.

---

## 📁 Repository Structure
```text
nxp_warehouse_robotics/
│
├── main_node.py                  # Thin orchestrator + FSM owner
│
├── navigation/
│   ├── frontier_exploration.py   # Frontier-based exploration logic
│   ├── navigation_controller.py # Thin Nav2 goal sender
│   └── shelf_goal_planner.py    # Shelf-relative goal generation
│
├── perception/
│   ├── shelf_detection_opencv.py # Shelf detection from SLAM map
│   └── qr_detection.py           # QR code detection (camera-based)
│
├── utils/
│   ├── map_utils.py              # Map ↔ world coordinate transforms
│   └── quaternion_utils.py       # Yaw → quaternion helpers
│
├── debug/
│   └── image_debug_publisher.py  # Debug image publishing utilities
```

---

## 🧭 Design Philosophy

### Thin `main_node.py`

The main node is intentionally minimal and only:

* Owns the FSM
* Handles ROS subscriptions and Nav2 callbacks
* Delegates all logic to modules

No perception, navigation, or geometry logic is embedded directly in callbacks.

---

### ROS-Independent Perception

Shelf detection is **completely ROS-independent**:

* Operates purely on NumPy arrays
* Uses OpenCV for morphology and geometry
* Allows easy offline testing and reuse

---

### Explicit Responsibility Separation

| Component             | Responsibility                       |
| --------------------- | ------------------------------------ |
| Frontier Explorer     | Decide *where* to explore            |
| Shelf Detector        | Detect shelves from SLAM map         |
| Goal Planner          | Decide *where to stand* near shelves |
| Navigation Controller | Send goals to Nav2                   |
| Main Node             | Orchestrate system state             |

This avoids hidden state and simplifies debugging.

---

## 🧪 Testing Strategy

The system was validated incrementally:

1. **Mapping & Frontier Logic**
   * Verified frontier detection using live SLAM maps
2. **Shelf Detection**
   * Tested independently on SLAM map images
   * Verified geometry and orientation filtering
3. **Goal Planning**
   * Validated goal offsets and orientations
4. **Navigation Integration**
   * Sequential Nav2 goal execution
   * Success-based goal advancement
5. **Full Pipeline**
   * Exploration → shelf detection → navigation

Testing was intentionally **non-monolithic**, reflecting real robotics workflows.

---

## 🚀 How This Node Is Run

This repository **does not launch a robot**.

Correct usage:
```bash
# Terminal 1: Start the robot platform (provided)
ros2 launch <provided_bringup> warehouse_sim.launch.py

# Terminal 2: Run autonomy layer
source install/setup.bash
ros2 run nxp_warehouse_robotics main_node.py
```

The autonomy node attaches to the already running system.

---

## 🎯 Project Status

* ✅ Architecture complete
* ✅ Exploration stable
* ✅ Shelf detection working
* ✅ Goal sequencing implemented
* ⚠️ Further Nav2 recovery tuning possible
* ⚠️ Extensive field testing pending

Overall completion: **~85–90%**

---

## 🧠 Key Learnings

* Designing autonomy separately from robot hardware
* SLAM map semantics and grid ↔ world transforms
* Map-based perception using classical CV
* FSM-driven robotics pipelines
* Nav2 action lifecycle handling
* Debugging distributed ROS 2 systems

---

## ⚠️ Notes on Proprietary Constraints

Due to competition rules:

* Robot URDF, bringup, and drivers are not included
* This repository focuses exclusively on autonomy logic
* Code is written to integrate cleanly without platform modification

---

## 📌 Intended Audience

* Robotics engineers
* Interviewers reviewing autonomy/system design
* Students learning ROS 2 architecture beyond tutorials

---

## 🔀 Finite State Machine (FSM)

The autonomy logic is governed by a **simple, explicit FSM** owned entirely by `main_node.py`.
```text
┌──────────────────────┐
│      EXPLORATION     │
│                      │
│ - Receive /map       │
│ - Detect frontiers   │
│ - Send Nav2 goals    │
│   to explore space   │
│                      │
└─────────┬────────────┘
          │
          │  No frontiers detected
          │  for N seconds
          ▼
┌────────────────────────────┐
│  NAVIGATION_TO_SHELVES     │
│                            │
│ - Run shelf detection ONCE │
│ - Generate shelf goals     │
│ - Sequential Nav2 goals    │
│   (object → QR)            │
│                            │
└────────────────────────────┘
```

### FSM Design Notes

* FSM transitions are **time-guarded**, not instantaneous
* Exploration ends only after **persistent frontier exhaustion**
* Shelf navigation is **finite** (no looping back to exploration)
* FSM state is stored **only in `main_node.py`**

---

## 🧠 High-Level Data Flow Diagram

This diagram shows **how data flows between layers**, not ROS nodes.
```text
           ┌──────────────────────────┐
           │   Robot Platform Layer   │
           │  (Provided / Proprietary)│
           │                          │
           │  - SLAM                  │
           │  - Sensors               │
           │  - Nav2                  │
           └──────────┬───────────────┘
                      │
        /map, /amcl_pose, camera
                      │
                      ▼
┌────────────────────────────────────────┐
│        Autonomy & Decision Layer        │
│          (This Repository)              │
│                                        │
│  ┌──────────────┐   ┌──────────────┐  │
│  │ Frontier     │   │ Shelf        │  │
│  │ Exploration  │   │ Detection    │  │
│  └──────┬───────┘   └──────┬───────┘  │
│         │                  │          │
│         ▼                  ▼          │
│      Nav2 Goals        Shelf Poses     │
│         │                  │          │
│         └──────────┬───────┘          │
│                    ▼                  │
│              Goal Planner              │
│                    │                  │
│                    ▼                  │
│           Navigation Controller        │
└────────────────────────────────────────┘
```

---

## 🧭 Navigation Goal Lifecycle

This diagram explains **exactly how goals are sent and completed** — interviewers love this.
```text
[ ShelfGoalPlanner ]
        │
        │  generate PoseStamped goals
        ▼
[ main_node.py ]
        │
        │  send_next_shelf_goal()
        ▼
[ NavigationController ]
        │
        │  send_goal_async()
        ▼
[ Nav2 Action Server ]
        │
        │  SUCCESS / FAILURE
        ▼
[ main_node.py ]
        │
        │  increment index ONLY on success
        ▼
[ Next Goal or Stop ]
```

### Key Guarantees

* Goals are **sequential**, never parallel
* Index increments **only on success**
* Failure does **not silently advance state**
* Nav2 remains the sole motion authority

---

## 🧩 Responsibility Boundary Diagram

This prevents *"why didn't you put X in Y?"* questions.
```text
+----------------------+------------------------------+
| Component            | Responsibility               |
+----------------------+------------------------------+
| main_node.py         | FSM, orchestration, callbacks|
| FrontierExplorer     | Where to explore             |
| ShelfDetector        | What is a shelf              |
| ShelfGoalPlanner     | Where robot should stand     |
| NavigationController | How to send Nav2 goals       |
| Nav2                 | Path planning & execution    |
+----------------------+------------------------------+
```

---

## 🎤 How to Explain This Diagram in an Interview (Say This)

> "I intentionally kept the FSM small and explicit.
> Exploration runs until the map is exhausted, then the system switches once to shelf navigation.
> Each module owns a single responsibility, and Nav2 remains the only component allowed to move the robot."

That statement alone puts you **above 90% of student projects**.

---