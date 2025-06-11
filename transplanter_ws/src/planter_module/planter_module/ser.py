import serial
import select
import time

class Serial:
    def __init__(self):
        self.ser = None
        self.buffer = ""

    def set(self, commPort): #sets the baudrate at 9600 and connects to the desired port
        self.ser = serial.Serial(commPort, 9600, timeout = 3)
        self.ser.reset_input_buffer()  # Clears old data
        print("port set")

    def reset(self): # doesnt do much
        self.ser = None

    def getOutput(self, timeout=2):
        """
        Reads from serial, ensuring only complete messages from START to END are processed.
        Handles cases where messages get split across reads.
        """
        while True:
            try:
                raw_data = self.ser.readline().decode('utf-8', errors='replace').strip()

                print(f"Raw Data: {raw_data}")
                
                if not raw_data:
                    continue  # Skip empty reads

                self.buffer += " " + raw_data  # Append new data to buffer

                print(f"buffer: {self.buffer}")

                # Remove any unexpected characters (e.g., "�")
                self.buffer = self.buffer.replace("�", "")

                # Check for a full message
                start_index = self.buffer.find("START")
                end_index = self.buffer.find("END", start_index)

                if start_index != -1 and end_index != -1:
                    # Extract the clean message
                    message = self.buffer[start_index:end_index + len("END")]
                    self.buffer = self.buffer[end_index + len("END"):]  # Remove processed message

                    self.ser.reset_input_buffer()  # Clears old data
                    print(f"Processed Message: {repr(message)}")  # Debugging output

                    # Parse message
                    return message
                
            except Exception as e:
                print(f"Error reading serial: {e}")
                self.ser.flush()
                self.buffer = ""
                continue  # Keep trying
    

    def drillMotor(self, direction): #sends a command to the drill motor to control it
        if self.ser:
            if direction == "forward":
                self.ser.write(b'DF')
            else:
                self.ser.write(b'DO')
            
            self.ser.flush()

