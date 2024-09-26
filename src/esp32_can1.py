import can

def receive_can_messages():
    # Set up the CAN interface (can0 in this case)
    bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)

    print("Listening for CAN messages on can0...")

    try:
        while True:
            # Receive a message (this will block until a message is received)
            msg = bus.recv()

            if msg:
                # Distinguish between CAN 2.0 and CAN FD based on the DLC and FD flag
                if msg.is_fd:
                    print(f"CAN FD message: ID={msg.arbitration_id}, Data Length={msg.dlc}, Data={msg.data.hex()}")
                else:
                    print(f"CAN 2.0 message: ID={msg.arbitration_id}, Data Length={msg.dlc}, Data={msg.data.hex()}")

    except KeyboardInterrupt:
        print("\nStopping CAN message receiver...")

if __name__ == "__main__":
    receive_can_messages()
