class Computer:
    """
    Onboard Computer.
    Manages coordination, calculation, parsing, and diagnostic tasks.
    Maps to the Computer block in the UML diagram and the second lifeline/bar in the Self Diagnose sequence diagram.
    """
    def __init__(self):
        self.control = None
        self.communication = None
        self.lab = None

    def set_references(self, control, communication, lab):
        self.control = control
        self.communication = communication
        self.lab = lab

    def perform_calculations(self, expression: str) -> float:
        """
        Calculates mathematical solutions for autonomous navigation and scientific parsing.
        """
        print(f" [Computer] Performing onboard calculations for: '{expression}'")
        # Simulating a calculation
        result = len(expression) * 1.5
        print(f" [Computer] Calculation result: {result}")
        return result

    def parse_command(self, raw_command: str) -> dict:
        """
        Parses commands received from Earth.
        """
        print(f" [Computer] Parsing raw signal command: '{raw_command}'")
        parts = raw_command.lower().split()
        cmd_type = parts[0] if parts else "unknown"
        args = parts[1:] if len(parts) > 1 else []
        parsed = {"action": cmd_type, "arguments": args}
        print(f" [Computer] Parsed command structure: {parsed}")
        return parsed

    def perform_diagnose_coordination(self, actuators, pmu, sensors) -> str:
        """
        Coordinates self-diagnostic routine.
        Matches the 'Sequence Diagram of Self Diagnose' step-by-step:
        1. connect Move() and ask Move() -> Actuators
        2. connect Charge() and ask Charge() -> PMU
        3. connect collect Data() and ask collect Data() -> Sensors
        4. connect Communication() and ask Communication() -> Communication
        """
        print("\n--- [Sequence: Self-Diagnose Coordination Started] ---")
        
        # 1. Connect and ask Move to Actuators
        print(" [Computer] Executing sub-check 1/4: Connecting to Actuators...")
        actuators.connect_move()
        move_ok = actuators.ask_move()
        print(f" [Computer] Actuators check complete. Status: {'OK' if move_ok else 'FAILED'}")
        if not move_ok:
            return "failed: actuators issue"

        # 2. Connect and ask Charge to PMU
        print(" [Computer] Executing sub-check 2/4: Connecting to PMU...")
        pmu.connect_charge()
        charge_ok = pmu.ask_charge()
        print(f" [Computer] PMU check complete. Status: {'OK' if charge_ok else 'FAILED'}")
        if not charge_ok:
            return "failed: low power/charge issue"

        # 3. Connect and ask collect Data to Sensors
        print(" [Computer] Executing sub-check 3/4: Connecting to Sensors...")
        sensors.connect_collect_data()
        sensors_ok = sensors.ask_collect_data()
        print(f" [Computer] Sensors check complete. Status: {'OK' if sensors_ok else 'FAILED'}")
        if not sensors_ok:
            return "failed: sensors malfunctioning"

        # 4. Connect and ask Communication to Communication Unit
        print(" [Computer] Executing sub-check 4/4: Connecting to Communication Unit...")
        # Since 'connect_communication' and 'ask_communication' are performed on Communication:
        # In our class diagram, Communication has send/receive/report methods.
        # We can implement connect_communication/ask_communication on the Communication class.
        if hasattr(self.communication, 'connect_communication'):
            self.communication.connect_communication()
        else:
            print(" [Computer] (Simulated connection interface on Communication Unit)")
        
        if hasattr(self.communication, 'ask_communication'):
            comm_ok = self.communication.ask_communication()
        else:
            print(" [Computer] (Simulated diagnostic ping on Communication Unit)")
            comm_ok = True

        print(f" [Computer] Communication Unit check complete. Status: {'OK' if comm_ok else 'FAILED'}")
        if not comm_ok:
            return "failed: communication link offline"

        print("--- [Sequence: Self-Diagnose Coordination Completed] ---\n")
        return "diagnose_ok"
