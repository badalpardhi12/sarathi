# Sarathi Robotics Simulation Framework

A framework for simulating and controlling the Meca500 robotic arm using RGB-D data and text-based motion commands.

## Project Overview
This project provides a complete pipeline for:
- Simulating the Meca500 robotic arm in virtual environments
- Processing RGB-D sensor data for object perception
- Converting natural language commands to robot motions
- Executing tasks in both simulation and real-world environments

## Directory Structure
```
assets/               # Robot models and meshes
├── meca500/          # Meca500 URDF and mesh files
configs/              # Configuration files (e.g., robot.yaml)
control/              # Motion execution logic
data/                 # RGB-D datasets (sim_rgbd_v1/)
init_tests/           # Hardware initialization tests
perception/           # Object resolution modules
robot/                # Robot interface implementations
scripts/              # Main entry points:
│   ├── capture_rgbd_dataset.py  # Dataset collection
│   ├── run_real_from_text.py    # Real robot execution
│   └── text_to_motion_sim.py    # Simulation execution
sim/                  # Simulation environment setup
skills/               # High-level skill implementations
```

## Setup Instructions
1. Install required dependencies:
```bash
pip install numpy pybullet opencv-python torch
```

2. Verify GPU acceleration (for simulation):
```bash
python init_tests/test_gpu.py
```

3. Test simulation environment:
```bash
python init_tests/test_sim.py
```

## Usage Examples

### Run text-to-motion simulation
```bash
python scripts/text_to_motion_sim.py "Pick up the red block and place it in the bin"
```

### Capture RGB-D dataset
```bash
python scripts/capture_rgbd_dataset.py --output data/custom_dataset
```

### Execute on real robot
```bash
python scripts/run_real_from_text.py "Move to home position"
```

## Key Components
- **Robot Interface**: [`robot/meca_iface.py`](robot/meca_iface.py)
- **Motion Execution**: [`control/execute_sim.py`](control/execute_sim.py)
- **Object Perception**: [`perception/resolve_object.py`](perception/resolve_object.py)
- **Text-to-Pose Conversion**: [`skills/text_to_pose.py`](skills/text_to_pose.py)

## Configuration
Modify robot parameters in [`configs/robot.yaml`](configs/robot.yaml)

## License
[Specify license here - MIT/Apache 2.0 recommended for robotics projects]