class Drill:
    """
    Geological Core Drill actuator.
    Maps to the drill block in the UML diagram.
    """
    def __init__(self, parent_actuators):
        self.parent = parent_actuators

    def drill(self):
        """
        UML Class Method. Runs the core drill to extract soil samples.
        """
        print("  [Drill] Engaging core drill... Rotating at 400 RPM.")
        print("  [Drill] Advancing core barrel into substrate...")
        print("  [Drill] Sample core extracted successfully!")
        self.parent.report_actuator_status("Drill", "Successful extraction")


class Wheels:
    """
    Rover Drive Wheels actuator.
    Maps to the wheels block in the UML diagram.
    """
    def __init__(self, parent_actuators):
        self.parent = parent_actuators

    def turnRight(self, rad: float):
        """
        UML Class Method. Steers the wheels to the right.
        """
        print(f"  [Wheels] Steering right by {rad:.2f} radians...")
        self.parent.report_actuator_status("Wheels", f"Steered right {rad} rad")

    def turnLeft(self, rad: float):
        """
        UML Class Method. Steers the wheels to the left.
        """
        print(f"  [Wheels] Steering left by {rad:.2f} radians...")
        self.parent.report_actuator_status("Wheels", f"Steered left {rad} rad")

    def move(self, pull: float):
        """
        UML Class Method. Rotates motors to drive forward or backward.
        """
        direction = "forward" if pull >= 0 else "reverse"
        print(f"  [Wheels] Driving wheels {direction} with force pull={abs(pull)}...")
        self.parent.report_actuator_status("Wheels", f"Driven {direction} distance={abs(pull)}")


class Actuators:
    """
    Actuator Coordination unit.
    Coordinates all physical rover movements.
    Maps to the Actuators block in the UML class diagram.
    """
    def __init__(self, camera=None):
        self.control = None
        self.drill = Drill(self)
        self.wheels = Wheels(self)
        # Camera is shared with actuators (rotate/report) and sensors (record/report)
        self.camera = camera
        if self.camera:
            self.camera.set_actuators(self)

    def set_control(self, control):
        self.control = control

    def set_camera(self, camera):
        self.camera = camera
        self.camera.set_actuators(self)

    def grip(self) -> bool:
        """
        UML Class Method. Actuates holding grips or locks tool assemblies.
        """
        print(" [Actuators] Engaging structural grip clamps for stabilization...")
        return True

    def move(self) -> bool:
        """
        UML Class Method. Actuates movement systems.
        """
        print(" [Actuators] Initiating physical motor movements...")
        return True

    def connect_move(self):
        """
        Diagnostic connection step. Matches 'connect Move()' in sequence diagram.
        """
        print(" [Actuators] Connecting diagnostic path to steering, drive, and drill motors...")

    def ask_move(self) -> bool:
        """
        Diagnostic check step. Matches 'ask Move()' in sequence diagram.
        Returns True if mechanical hardware is safe.
        """
        print(" [Actuators] Performing diagnostic load tests on drive systems and drill...")
        print(" [Actuators] Drive actuators feedback: 100% operational.")
        return True

    def report_actuator_status(self, device_name: str, status_msg: str):
        """
        Coordinates 'report' arrows pointing back from subclasses to Actuators.
        """
        print(f" [Actuators] Received report from {device_name}: '{status_msg}'")
