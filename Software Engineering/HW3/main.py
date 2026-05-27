from rover.state import StateMachine
from rover.control import Control
from rover.computer import Computer
from rover.communication import Communication, TxRx
from rover.pmu import PMU
from rover.lab import Lab
from rover.actuators import Actuators
from rover.sensors import Sensors

def main():
    print("=================================================================")
    print("                MARS ROVER AUTONOMOUS MODEL                      ")
    print("         Software Engineering HW3 Model Reitinger                 ")
    print("=================================================================")
    
    
    state_machine = StateMachine()
    
    antenna = TxRx()
    comm_unit = Communication(antenna)
    computer = Computer()
    pmu = PMU()
    lab = Lab()
    sensors = Sensors()
    actuators = Actuators(camera=sensors.camera) # Link camera actuator
    
    control = Control(state_machine)
    
    # Establishes bi-directional references as specified in UML associations
    control.set_references(
        communication=comm_unit,
        computer=computer,
        pmu=pmu,
        actuators=actuators,
        sensors=sensors,
        lab=lab
    )
    
    comm_unit.set_references(control=control, computer=computer)
    computer.set_references(control=control, communication=comm_unit, lab=lab)
    pmu.set_control(control=control)
    actuators.set_control(control=control)
    sensors.set_references(control=control, lab=lab)
    lab.set_references(computer=computer, control=control)

    # -------------------------------------------------------------
    # PHASE 1: Boot Up and Self Diagnose
    # Matches 'Sequence Diagram of Self Diagnose' and 'State_diagram.jpg'
    # -------------------------------------------------------------
    print("\n[PHASE 1: System Boot & Diagnosis]")
    diagnose_result = control.diagnose()
    if not diagnose_result:
        print("[Fatal] Diagnostic sequence failed. Shutting down system.")
        return

    # -------------------------------------------------------------
    # PHASE 2: Navigating the Mars Surface
    # Matches 'Drive command' trigger in State diagram
    # -------------------------------------------------------------
    print("\n[PHASE 2: Navigation Command]")
    # Steering left by 90 degrees (1.57 rad) and moving 15 meters forward
    control.performMove(["left", "15.0"])

    # -------------------------------------------------------------
    # PHASE 3: Scientific Drilling & Sample Analysis
    # Matches 'Find and collect' trigger in State diagram
    # -------------------------------------------------------------
    print("\n[PHASE 3: Scientific Exploration Command]")
    control.perform_find_and_collect("Hematite Layer 04")
    
    # Report conditions check
    print("\n[Lab/Sensor reporting check]")
    sensors.report_to_lab()
    sensors.report_to_control()

    # -------------------------------------------------------------
    # PHASE 4: Telemetry Transmission to Earth
    # Matches 'ask for update' trigger in State diagram
    # Matches Scenario A (Control -> Comm -> Antenna -> Earth) in Communication Sequence Diagram
    # -------------------------------------------------------------
    print("\n[PHASE 4: Uplink Telemetry Command]")
    control.trigger_communication("BATTERY=85%_VIB=OK_RAD=SAFE")

    # -------------------------------------------------------------
    # PHASE 5: Earth Command & Reply Interaction (Incoming flow)
    # Matches Scenario B (Earth -> Antenna -> Comm -> Control) in Communication Sequence Diagram
    # -------------------------------------------------------------
    print("\n[PHASE 5: Downlink Earth Command Flow]")
    # Earth sends a command 'drive forward 25' which antenna receives and forwards
    antenna.receive_from_earth("drive right 25.0")

    # -------------------------------------------------------------
    # PHASE 6: Power Management & Solar Recharging
    # Matches 'low battery' trigger in State diagram
    # -------------------------------------------------------------
    print("\n[PHASE 6: Low Battery & Charging Cycle]")
    # Simulate battery dropping below threshold
    pmu._battery_level = 8.0 # Simulate low battery charge
    control.trigger_low_battery()

    # -------------------------------------------------------------
    # PHASE 7: Power Off Shutdown
    # Matches 'Power off' trigger in State diagram
    # -------------------------------------------------------------
    print("\n[PHASE 7: Shutdown Sequence]")
    control.power_off()

    print("\n=================================================================")
    print("                SIMULATION FINISHED SUCCESSFULLY                 ")
    print("=================================================================")

if __name__ == "__main__":
    main()
