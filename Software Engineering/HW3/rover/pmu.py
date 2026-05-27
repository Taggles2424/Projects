class PMU:
    """
    Power Management Unit.
    Tracks battery levels, power modes, and manages battery safety.
    Corresponds to the PMU block in the UML class diagram.
    """
    def __init__(self):
        self.control = None
        self._battery_level = 85.0 # percentage
        self._power_mode = "balanced"

    def set_control(self, control):
        self.control = control

    def connect_charge(self):
        """
        Diagnostic connection step. Matches 'connect Charge()' in sequence diagram.
        """
        print("  [PMU] Connecting diagnostic probes to charger circuits...")

    def ask_charge(self) -> bool:
        """
        Diagnostic check step. Matches 'ask Charge()' in sequence diagram.
        Returns True if battery is operational.
        """
        print("  [PMU] Querying battery status, capacity, and current power state...")
        operational = self._battery_level > 10.0
        print(f"  [PMU] Battery report: level={self._battery_level}%, Health=99%, Status={'OPERATIONAL' if operational else 'LOW_POWER'}")
        return operational

    def reportPower(self) -> float:
        """
        UML Class Method (written with exact casing/naming).
        Returns the current battery level.
        """
        # UML diagram shows '-reportPower()' in the PMU class
        print(f"  [PMU] Private check: Current battery charge is {self._battery_level}%")
        return self._battery_level

    def adjust_power_profile(self, mode: str):
        """
        Adjusts the power consumption profile of the Rover.
        """
        self._power_mode = mode
        print(f"  [PMU] Power mode changed to '{self._power_mode}'. Adjusting voltages and sleep states...")

    def charge_battery(self):
        """
        Simulates charging the battery to 100%.
        """
        print(f"  [PMU] Initializing solar panels... Current Battery: {self._battery_level}%")
        while self._battery_level < 100.0:
            self._battery_level += 5.0
            if self._battery_level > 100.0:
                self._battery_level = 100.0
            print(f"  [PMU] Charging: {self._battery_level}%")
        
        print("  [PMU] Charge complete! 100% capacity restored.")
        # Notify central controller
        if self.control:
            self.control.notify_battery_charged()
