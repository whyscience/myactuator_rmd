import threading
import time

import can


# Setup the CAN interface
def setup_can_interface(interface='can0', bitrate=1000000):
    try:
        bus = can.interface.Bus(channel=interface, bustype='socketcan', bitrate=bitrate)
        print("CAN Receiver initialized")
        return bus
    except OSError as e:
        print(f"Error setting up CAN interface: {e}")
        exit(1)


# Function to receive and process CAN message
def receive_can_messages(bus):
    idx = 0
    while True:
        message = bus.recv(1.0)  # Timeout of 1 second
        if message is not None:
            print(f"Received packet with id 0x{message.arbitration_id:X}, idx={idx}")
            idx += 1
            # Check if it's an extended frame
            if message.is_extended_id:
                print("Extended frame")

            # Check if it's a remote frame (RTR)
            if message.is_remote_frame:
                print("RTR frame and requested length", message.dlc)
            else:
                # print(f"Data length: {message.dlc}")
                print("Data: ", end="")
                for byte in message.data:
                    # print data in Hex
                    print(f"{byte:X} ", end="")
                print()
        else:
            # if idx % 10 == 0:
            #     print("No message received {}".format(idx))
            pass



def send_can_messages(bus):
    message = can.Message(arbitration_id=0x141, data=[0, 1, 2, 3, 4, 5, 6, 7], is_extended_id=False)
    count = 0
    while True:
        try:
            bus.send(message)
            print("{} Message sent on {}".format(count, bus.channel_info))
            count += 1
        except can.CanError:
            print("Message NOT sent")
        time.sleep(5)


# Main function to initialize and receive messages
def main():
    bus = setup_can_interface()
    # add a thread to send messages
    send_thread = threading.Thread(target=send_can_messages, args=(bus,))
    send_thread.start()

    try:
        while True:
            receive_can_messages(bus)
            # send_can_messages(bus)
    except KeyboardInterrupt:
        print("\nCAN Receiver stopped")


if __name__ == "__main__":
    main()
