import socket
import time  # For adding delay between commands

HOST = "10.181.205.209"  # robot IP
PORT = 30002  # port for sending commands

# Initialize the socket connection
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# Command to move to 100 degrees
move_to_100 = "movej([d2r(100), 0, 0, 0, 0, 0])\n"
# Command to move to 0 degrees
move_to_0 = "movej([d2r(0), 0, 0, 0, 0, 0])\n"

try:
    while True:
        # Move to 100 degrees
        s.send(move_to_100.encode('ascii'))
        time.sleep(2)  # Wait for the robot to finish the move (adjust as needed)

        # Move to 0 degrees
        s.send(move_to_0.encode('ascii'))
        time.sleep(2)  # Wait for the robot to finish the move (adjust as needed)

except KeyboardInterrupt:
    print("Program interrupted. Stopping...")
finally:
    s.close()  # Close the connection when done

