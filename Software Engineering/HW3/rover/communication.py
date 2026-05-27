class TxRx:
    """
    Antenna / Transceiver unit of the Mars Rover.
    It corresponds to the Tx/Rx block in the UML diagram and the :Antenna lifeline in the sequence diagram.
    """
    def __init__(self):
        self.comm_unit = None

    def set_comm_unit(self, comm_unit):
        self.comm_unit = comm_unit

    def send(self, message: str) -> str:
        """
        Sends message to Earth.
        """
        print(f"  [Antenna] Transmitting payload to Earth: '{message}'")
        # Simulating transmission latency
        print("  [Antenna] Signal sent successfully to Earth. Waiting for Ack...")
        return "ack"

    def receive_from_earth(self, reply_message: str):
        """
        Simulates receiving an incoming signal/reply from Earth.
        Matches the second half of the Communication Unit sequence diagram.
        """
        print(f"\n--- [Sequence: Earth Reply Flow Started] ---")
        print(f"  [Earth] Sending reply to Rover Antenna: '{reply_message}'")
        print(f"  [Antenna] Received reply signal from Earth: '{reply_message}'")
        
        # Forwarding the reply to the Communication Unit
        if self.comm_unit:
            self.comm_unit.receive_reply(reply_message, self)
        else:
            print("  [Antenna] Error: Communication Unit reference not set.")


class Communication:
    """
    Communication Unit class.
    Matches the Communication block in the UML class diagram and the :Communic. unit lifeline in the sequence diagram.
    """
    def __init__(self, antenna: TxRx):
        self.antenna = antenna
        self.antenna.set_comm_unit(self)
        self.control = None
        self.computer = None

    def set_references(self, control, computer):
        self.control = control
        self.computer = computer

    def send(self, message: str) -> str:
        """
        Sends data. Corresponds to +send() in the UML and 'send message' in the sequence diagram.
        """
        print(f" [Comm Unit] Preparing to send message: '{message}'")
        print(" [Comm Unit] Forwarding transmission request to Antenna...")
        ack_status = self.antenna.send(message)
        print(f" [Comm Unit] Received transmission ack from Antenna: '{ack_status}'")
        return ack_status

    def receive(self) -> str:
        """
        Corresponds to +receive() in the UML.
        """
        print(" [Comm Unit] Checking buffer for incoming transmissions...")
        return "No unread messages"

    def receive_reply(self, reply_message: str, sender_antenna: TxRx):
        """
        Processes reply received from Antenna.
        Matches the Communication Unit sequence diagram.
        """
        print(" [Comm Unit] Processing reply received from Antenna...")
        # Sending Ack back to Antenna
        print(" [Comm Unit] Sending 'ack' to Antenna")
        
        # Forwarding the message to the Control Unit
        if self.control:
            print(" [Comm Unit] Forwarding the reply message to the Control Unit...")
            self.control.forward_from_comm(reply_message, self)
        else:
            print(" [Comm Unit] Error: Control unit reference not set.")

    def report(self) -> str:
        """
        Corresponds to +report() in the UML.
        """
        status = "Communication Unit is ONLINE. Link Quality: 98%."
        print(f" [Comm Unit] Reporting status: {status}")
        return status
