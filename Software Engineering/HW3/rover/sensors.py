class Camera:
    """
    Onboard high-resolution camera.
    Associated with both Actuators (for mechanical rotation) and Sensors (for recording data).
    Maps to the Camera block in the UML class diagram.
    """
    def __init__(self):
        self.actuators = None
        self.sensors = None

    def set_actuators(self, actuators):
        self.actuators = actuators

    def set_sensors(self, sensors):
        self.sensors = sensors

    def record(self) -> str:
        """
        UML Class Method. Captures panoramic raw image data.
        """
        print("  [Camera] Powering up CCD sensors...")
        print("  [Camera] Calibrating focus and aperture...")
        image_metadata = "IMG_MARS_PANORAMA_SOL34.RAW - 4096x4096px - Haze reduction ON"
        print(f"  [Camera] Recording raw frame: {image_metadata}")
        
        # Report status back to Sensors and Actuators
        if self.sensors:
            self.sensors.report_sensor_status("Camera", "Image captured successfully")
        if self.actuators:
            self.actuators.report_actuator_status("Camera", "Rotation completed, frame recorded")
            
        return image_metadata


class TemperatureSensor:
    """
    Thermal probe reporting to Sensors.
    Maps to the Temperature block in the UML diagram.
    """
    def __init__(self, parent_sensors):
        self.parent = parent_sensors
        self.value = -55.4 # Default temperature in degrees C

    def read_temperature(self) -> float:
        """
        Returns temperature in degrees Celsius.
        """
        self.parent.report_sensor_status("Temperature", f"{self.value} C")
        return self.value


class WindSensor:
    """
    Anemometer reporting to Sensors.
    Maps to the Wind block in the UML diagram.
    """
    def __init__(self, parent_sensors):
        self.parent = parent_sensors
        self.value = 7.2 # Default wind speed in m/s

    def read_wind_speed(self) -> float:
        """
        Returns wind speed in meters per second.
        """
        self.parent.report_sensor_status("Wind", f"{self.value} m/s")
        return self.value


class PressureSensor:
    """
    Atmospheric pressure transducer reporting to Sensors.
    Maps to the Pressure block in the UML diagram.
    """
    def __init__(self, parent_sensors):
        self.parent = parent_sensors
        self.value = 6.1 # Default pressure in hPa

    def read_pressure(self) -> float:
        """
        Returns atmospheric pressure in hPa.
        """
        self.parent.report_sensor_status("Pressure", f"{self.value} hPa")
        return self.value


class Sensors:
    """
    Sensor array coordinator.
    Collects raw physical measurements and reports telemetry data.
    Maps to the Sensors block in the UML class diagram.
    """
    def __init__(self):
        self.control = None
        self.lab = None
        
        # Instantiate sub-sensors
        self.camera = Camera()
        self.camera.set_sensors(self)
        self.temperature = TemperatureSensor(self)
        self.wind = WindSensor(self)
        self.pressure = PressureSensor(self)
        
        self._sensor_statuses = {}

    def set_references(self, control, lab):
        self.control = control
        self.lab = lab

    def activateSensor(self, sensor_name: str) -> bool:
        """
        UML Class Method. Powers up and calibrates a specific sensor.
        """
        print(f" [Sensors] Activating and calibrating: '{sensor_name}'...")
        self._sensor_statuses[sensor_name.lower()] = "ACTIVE"
        return True

    def connect_collect_data(self):
        """
        Diagnostic connection step. Matches 'connect collect Data()' in sequence diagram.
        """
        print(" [Sensors] Connecting digital telemetry buses to temperature, pressure, wind, and camera...")

    def ask_collect_data(self) -> bool:
        """
        Diagnostic check step. Matches 'ask collect Data()' in sequence diagram.
        Returns True if sensors are responding.
        """
        print(" [Sensors] Simulating data collection test across all sensor buses...")
        # Check components
        temp_val = self.temperature.read_temperature()
        wind_val = self.wind.read_wind_speed()
        pres_val = self.pressure.read_pressure()
        
        operational = temp_val is not None and wind_val is not None and pres_val is not None
        print(f" [Sensors] Sensors diagnostic complete. Status: {'OPERATIONAL' if operational else 'ERROR'}")
        return operational

    def report_sensor_status(self, sensor_type: str, status_value: str):
        """
        Coordinates the 'report' arrows from subclasses to Sensors.
        """
        print(f" [Sensors] Received telemetry report from {sensor_type}: '{status_value}'")

    def report_to_control(self) -> dict:
        """
        Reports sensor data to Control. Matches 'report failure/data' arrow.
        """
        data = {
            "temperature": self.temperature.read_temperature(),
            "wind": self.wind.read_wind_speed(),
            "pressure": self.pressure.read_pressure()
        }
        print(f" [Sensors] Reporting telemetry dataset to Control Unit: {data}")
        return data

    def report_to_lab(self):
        """
        Reports conditions to the Lab. Matches 'Report conditions' arrow.
        """
        print(" [Sensors] Initiating condition report transfer to Science Lab...")
        data = {
            "temperature": self.temperature.read_temperature(),
            "wind": self.wind.read_wind_speed(),
            "pressure": self.pressure.read_pressure()
        }
        self.lab.report_conditions(data)
