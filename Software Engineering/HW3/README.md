# Mars Rover System Model - Software Engineering HW3

A fully functional object-oriented Python implementation of the Mars Rover system. The architecture is mapped directly from the team's conceptual UML class diagram, and its execution flow faithfully recreates the transitions of the state diagram and the interaction steps of the sequence diagrams.

## Project Structure

The project is structured as a clean, modular Python package located in the `rover` subdirectory:

```
HW3/
├── main.py               # Main simulation entry point
├── README.md             # This documentation file
├── FEEDBACK.md           # My individual course & team feedback
└── rover/                # Operational core package
    ├── __init__.py       # Package initialization
    ├── state.py          # StateMachine and RoverState Enum (State diagram)
    ├── control.py        # Central Control Unit coordination
    ├── computer.py       # Onboard computer for diagnostics & command parsing
    ├── pmu.py            # Power Management Unit (battery & solar charging)
    ├── lab.py            # Scientific Analysis Laboratory (geological testing)
    ├── actuators.py      # Actuator coordination (Drill, Wheels motors)
    └── sensors.py        # Sensor array coordination (Camera, Environmental probes)
```

## System Requirements

- **Runtime**: Python 3.8 or higher.
- **Dependencies**: None. Model developed using only Python's standard library.

## Execution Instructions

To execute the Mars Rover model and run through the complete multi-phase operational simulation, run the following command in your terminal:

```bash
python main.py
```

## UML Diagram Mapping Details

- **Bidirectional Associations**: All links between systems (e.g., `Control` to `Actuators`, `Sensors` to `Lab`) are fully represented in Python. References are established at runtime inside `main.py` constructor bindings to maintain architectural fidelity.
- **Inheritance & Hierarchy**:
  - `Actuators` coordinates `Drill`, `Wheels`, and shares rotation mapping with `Camera`.
  - `Sensors` coordinates `TemperatureSensor`, `WindSensor`, `PressureSensor`, and receives visual data recording feeds from `Camera`.
- **State Machine (`State_diagram.jpg`)**:
  - Encapsulated within `state.py`.
  - Handled dynamically by `Control` which triggers state updates on events like `Drive command`, `Find and collect`, `ask for update`, and `low battery`.
- **Self-Diagnose Sequence (`Sequence Diagram for Self Diagnose.png`)**:
  - Coordinated by the `Computer` unit under `perform_diagnose_coordination()`.
  - Step-by-step connection checks and queries are sent sequentially to `Actuators`, `PMU`, `Sensors`, and `Communication` before returning diagnostic clearance back to `Control`.
- **Communication Flow (`Seq_Diagram_Communication_unit.jpg`)**:
  - Recreates both Scenario A (Outgoing telemetry packets: `Control -> Comm Unit -> Antenna -> Earth`) and Scenario B (Incoming Earth commands: `Earth -> Antenna -> Comm Unit -> Control`) with full asynchronous acknowledge signaling and data forwarding structures.
