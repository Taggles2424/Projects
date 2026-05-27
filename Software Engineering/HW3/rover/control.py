from .state import RoverState, StateMachine

class Control:
    """
    Central Controller for the Mars Rover.
    Corresponds to the Control block in the UML class diagram and the first lifeline in sequence diagrams.
    Tracks state transitions and coordinates actuators, sensors, communication, and PMU.
    """
    def __init__(self, state_machine: StateMachine):
        self.state_machine = state_machine
        self.communication = None
        self.computer = None
        self.pmu = None
        self.actuators = None
        self.sensors = None
        self.lab = None

    def set_references(self, communication, computer, pmu, actuators, sensors, lab):
        self.communication = communication
        self.computer = computer
        self.pmu = pmu
        self.actuators = actuators
        self.sensors = sensors
        self.lab = lab

    def diagnose(self) -> bool:
        """
        Executes diagnostic routine.
        Matches 'Sequence Diagram of Self Diagnose' and transitions state.
        """
        print("[Control] Starting Mars Rover diagnose sequence...")
        # Self Diagnose is already our initial state
        if self.state_machine.current_state != RoverState.SELF_DIAGNOSE:
            self.state_machine.transition_to(RoverState.SELF_DIAGNOSE, "run check")

        # Delegate diagnostic coordination to Computer
        result = self.computer.perform_diagnose_coordination(
            self.actuators, self.pmu, self.sensors
        )

        if result == "diagnose_ok":
            print("[Control] Self-Diagnose: OK!")
            self.state_machine.transition_to(RoverState.IDLE, "OK")
            return True
        else:
            print(f"[Control] Self-Diagnose failed. Reason: {result}")
            self.state_machine.transition_to(RoverState.FINAL_STATE, "Issue found")
            return False

    def performMove(self, command_args: list):
        """
        UML Class Method. Executes move command.
        Transitions state, drives actuators, and transitions back to IDLE.
        """
        if self.state_machine.current_state != RoverState.IDLE:
            print("[Control] Error: Rover must be in IDLE state to move.")
            return

        self.state_machine.transition_to(RoverState.MOVING, "Drive command")
        
        # Determine movement parameters
        direction = command_args[0] if command_args else "forward"
        distance = float(command_args[1]) if len(command_args) > 1 else 10.0
        
        print(f"[Control] Command decoded. Moving '{direction}' by {distance} meters...")
        
        # Delegate to Actuators
        # Wheels are steered and moved
        if direction == "left":
            self.actuators.wheels.turnLeft(1.57) # turn left by pi/2 radians
        elif direction == "right":
            self.actuators.wheels.turnRight(1.57)
            
        self.actuators.move()
        self.actuators.wheels.move(distance)
        
        print("[Control] Movement complete.")
        self.state_machine.transition_to(RoverState.IDLE, "Moved")

    def controlPower(self, mode: str):
        """
        UML Class Method. Adjusts power parameters.
        """
        print(f"[Control] Adjusting Power management: switching to '{mode}' mode")
        self.pmu.adjust_power_profile(mode)

    def trigger_low_battery(self):
        """
        Simulates low battery safety transition.
        """
        if self.state_machine.current_state != RoverState.IDLE:
            print("[Control] Safety Alert: Battery low! Interrupting active tasks.")
        
        self.state_machine.transition_to(RoverState.CHARGING, "low battery")
        print("[Control] Charging activated. PMU tracking battery levels...")
        self.pmu.charge_battery()

    def notify_battery_charged(self):
        """
        Called by PMU when battery levels are restored.
        """
        print("[Control] PMU reports full charge.")
        self.state_machine.transition_to(RoverState.IDLE, "battery charged")

    def perform_find_and_collect(self, target: str):
        """
        Simulates sample collection.
        Transitions to COLLECTING_SAMPLE, drills, triggers sensors, performs lab analysis, returns to IDLE.
        """
        if self.state_machine.current_state != RoverState.IDLE:
            print("[Control] Error: Rover must be in IDLE state to collect sample.")
            return

        self.state_machine.transition_to(RoverState.COLLECTING_SAMPLE, "Find and collect")
        
        print(f"[Control] Sampling target identified: '{target}'")
        
        # 1. Activate Camera to record visual data before drilling
        self.sensors.activateSensor("camera")
        self.actuators.camera.record()
        
        # 2. Activate Temperature and Pressure sensors to record environment conditions
        self.sensors.activateSensor("temperature")
        self.sensors.activateSensor("pressure")
        
        # 3. Use Actuator Drill to collect sample
        self.actuators.grip()
        self.actuators.drill.drill()
        
        # 4. Perform Lab analysis
        print("[Control] Delivering sample to Lab for geological tests...")
        vib_level = self.lab.getVib(2.4)
        tvac_temp = self.lab.getTVAC(120.0)
        rad_level = self.lab.getrad(0.015)
        
        print(f"[Control] Lab analysis results: Vib={vib_level}, TVAC={tvac_temp}, Rad={rad_level}")
        print("[Control] Sample parsed and logged into research database.")
        
        self.state_machine.transition_to(RoverState.IDLE, "Collected sample")

    def trigger_communication(self, telemetry_data: str):
        """
        Simulates communication task with Earth.
        Transitions state, sends message, waits for acknowledge, transitions back.
        Matches Scenario A (Control -> Comm Unit -> Antenna -> Earth) in sequence diagram.
        """
        if self.state_machine.current_state != RoverState.IDLE:
            print("[Control] Error: Rover must be in IDLE state to communicate.")
            return

        self.state_machine.transition_to(RoverState.COMMUNICATING, "ask for update")
        print("\n--- [Sequence: Communication Outgoing Flow Started] ---")
        print(f"[Control] Sending telemetry packet to Communication Unit...")
        
        ack = self.communication.send(telemetry_data)
        print(f"[Control] Received Transmission status: '{ack}'")
        print("--- [Sequence: Communication Outgoing Flow Completed] ---\n")
        
        self.state_machine.transition_to(RoverState.IDLE, "Send Data (finished task)")

    def forward_from_comm(self, reply_message: str, sender_comm):
        """
        Receives forwarded message from Communication Unit.
        Matches Scenario B (Antenna -> Comm Unit -> Control) in sequence diagram.
        """
        print(f"[Control] Received forwarded signal message: '{reply_message}'")
        print("[Control] Executing command embedded in reply...")
        
        # Process and parse command
        parsed = self.computer.parse_command(reply_message)
        
        # Sending Ack back to Communication Unit
        print("[Control] Sending 'ack' back to Communication Unit")
        
        # Return gate (exit sequence)
        print("[Control] Complete: Outgoing 'gate' trigger activated.")

    def power_off(self):
        """
        Powers off the rover, transitioning it to the final state.
        """
        print("[Control] Command received: Initiate Power Off.")
        self.state_machine.transition_to(RoverState.FINAL_STATE, "Power off")
        print("[Control] Operational shut down completed.")
