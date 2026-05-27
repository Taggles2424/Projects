class Lab:
    """
    Onboard Science Laboratory.
    Performs geological, vacuum, thermal, and radiation testing.
    Corresponds to the Lab block in the UML class diagram.
    """
    def __init__(self):
        self.computer = None
        self.control = None

    def set_references(self, computer, control):
        self.computer = computer
        self.control = control

    def getVib(self, x: float) -> str:
        """
        UML Class Method. Performs vibrational testing on the collected sample.
        """
        print(f"  [Lab] Running vibration test on sample at amplitude x={x} mm...")
        # Simulating a vibration analysis
        density = x * 0.42
        result = f"Density estimate: {density:.2f} g/cm3 - Moderate cohesion"
        print(f"  [Lab] Vibration result: {result}")
        return result

    def getTVAC(self, t: float) -> str:
        """
        UML Class Method. Performs Thermal Vacuum Chamber (TVAC) simulated testing.
        """
        print(f"  [Lab] Simulating vacuum chamber thermal stress test at temp t={t} Kelvin...")
        # Simulating a thermal stress analysis
        sublimation_point = t * 1.05
        result = f"Sublimation point: {sublimation_point:.2f} K - No degassing observed"
        print(f"  [Lab] TVAC result: {result}")
        return result

    def getrad(self, r: float) -> str:
        """
        UML Class Method. Measures radiation levels of the collected geological sample.
        """
        print(f"  [Lab] Running radiation count on sample at dose rate r={r} mSv/h...")
        # Simulating radiation analysis
        radioactive = "HIGH" if r > 0.05 else "SAFE/NEGLIGIBLE"
        result = f"Activity: {radioactive} ({r} mSv/h) - High silicate presence"
        print(f"  [Lab] Radiation result: {result}")
        return result

    def report_conditions(self, sensor_data: dict) -> str:
        """
        Processes reports from Sensors and creates a combined geological environment analysis.
        Matches 'Report conditions' link in UML diagram.
        """
        print("  [Lab] Aggregating environmental conditions reported by sensors...")
        report = f"Geological Report: Temp={sensor_data.get('temperature')}C, Wind={sensor_data.get('wind')}m/s, Pres={sensor_data.get('pressure')}hPa"
        print(f"  [Lab] Consolidated conditions: {report}")
        return report
