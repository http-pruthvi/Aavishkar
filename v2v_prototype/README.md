# V2V Collision Avoidance Prototype

This project demonstrates a Vehicle-to-Vehicle (V2V) collision avoidance system using ROS2.

## Structure
- `src/`: Source code for BSM publisher and listener nodes.
- `msg/`: Custom Basic Safety Message (BSM) definition.
- `launch/`: Launch files to run the demo.
- `scripts/`: Helper scripts for building and running.

## Prerequisites
- ROS2 Humble (or compatible)
- `colcon` build tool

## How to Run
1. Generate keys (optional for this demo but good practice):
   ```bash
   ./scripts/generate_keys.sh
   ```
2. Build and run the demo:
   ```bash
   ./scripts/run_demo.sh
   ```

## What to Expect
- **Vehicle 1** (Publisher) will start at (0,0) and move diagonally at constant speed.
- **Vehicle 2** (Listener) is simulated starting at (100,100) and moving towards Vehicle 1.
- The listener will calculate Time-to-Collision (TTC) and print warnings to the console when a collision is imminent.
