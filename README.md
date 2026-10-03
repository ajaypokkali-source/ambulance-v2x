# Ambulance V2X Simulation

A simulation-based project using Eclipse SUMO, Python, and TraCI to develop and evaluate Vehicle-to-Everything (V2X) communication for emergency ambulance movement through urban traffic.

## Project Objectives

- Implement Vehicle-to-Vehicle (V2V) emergency communication.
- Implement Vehicle-to-Infrastructure (V2I) traffic-light preemption.
- Implement Infrastructure-to-Infrastructure (I2I) coordination.
- Integrate V2V, V2I, and I2I into a coordinated V2X system.
- Evaluate ambulance movement and surrounding traffic response.

## Technologies Used

- Eclipse SUMO – Traffic simulation.
- Python – Simulation logic and control.
- TraCI – SUMO control interface.
- XML – Network, route, and configuration files.
- Git and GitHub – Version control and collaboration.

## Project Structure

```text
ambulance_v2x/
    python/
    results/
    simulation/
    tls_simulation/
    v2v_stress_test/
    v2x_integrated/
    .gitignore
    README.md
```

## Implemented Modules

### 1. V2V – Vehicle-to-Vehicle Communication

- Detects an approaching ambulance and surrounding vehicles.
- Identifies vehicles blocking the ambulance in the same lane.
- Sends emergency messages to nearby vehicles.
- Slows blocking vehicles and checks adjacent-lane safety.
- Commands safe lane changes to clear the ambulance's path.
- Allows the ambulance to overtake.
- Supports returning vehicles to their original lanes after the ambulance passes.
- Includes intersection-level vehicle yielding behavior.

### 2. V2I – Vehicle-to-Infrastructure Communication

- Detects an approaching ambulance near a traffic signal.
- Calculates its distance from the target intersection.
- Sends an emergency preemption request.
- Activates and maintains the emergency signal phase.
- Restores normal signal operation after the ambulance clears the intersection.

### 3. I2I – Infrastructure-to-Infrastructure Coordination

- Coordinates traffic-light infrastructure across multiple intersections.
- Supports emergency corridor signal coordination.
- Enables infrastructure to respond to the ambulance's progress along its route.

### 4. Integrated V2X System

- Combines vehicle-level and infrastructure-level emergency responses.
- Integrates V2V vehicle yielding and lane-clearing behavior.
- Integrates V2I traffic-light preemption.
- Integrates I2I multi-intersection coordination.
- Supports coordinated ambulance movement through the simulated road corridor.

## Road Network

The main corridor consists of five intersections:

J1 ---- J2 ---- J3 ---- J4 ---- J5

The network includes:

- Multiple intersections and connecting side roads.
- Forward and reverse traffic.
- Two lanes in each direction on the main corridor.
- Traffic-light-controlled junctions.
- Normal traffic and emergency-vehicle routes.

## Running the Simulation

Install Eclipse SUMO and Python.

Install TraCI:

```bash
pip install traci
```

Navigate to the required experiment directory and run its Python controller.

Example:

```bat
cd /d C:\ambulance_v2x\v2x_integrated
python your_controller_script.py
```

Replace `your_controller_script.py` with the actual controller filename.

## Results and Evaluation

The results directory is reserved for simulation outputs and experiment data.

Evaluation metrics include:

- Ambulance travel time.
- Ambulance delay at intersections.
- Emergency-message detection time.
- Vehicle response time.
- Lane-change completion time.
- Number of responding vehicles.
- Traffic-light preemption response time.
- Intersection clearance time.
- Comparison between normal traffic and V2X-assisted traffic.

## Future Work

- Expand traffic-density and route scenarios.
- Perform additional integrated V2X stress tests.
- Analyze and compare simulation performance.
- Improve result visualization and reporting.
- Explore real-world deployment considerations.

## Cloud Integration

AWS cloud integration may be explored in a future phase. It is currently on hold and is not required to run the simulations.

## Project Purpose

This project develops a SUMO-based V2X framework to study how communication between emergency vehicles, surrounding vehicles, and road infrastructure can support coordinated ambulance movement through urban traffic.