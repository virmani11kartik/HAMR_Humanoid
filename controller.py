import pygame
import socket
import serial
import time
import select
import csv
import datetime
import re
import matplotlib.pyplot as plt

# # Change COM to your ESP32 port
# ser = serial.Serial('COM4', 115200, timeout=0.1)
# time.sleep(2)  # wait for ESP to reset

# === Update with the ESP32's IP printed on Serial Monitor ===
ESP32_IP = "192.168.4.1"  # Replace with your ESP32 IP
ESP32_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)  # Non-blocking mode

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to controller: {joystick.get_name()}")

csv_file = open("controller_and_pose_log.csv", mode="w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "timestamp",
    "LX", "LY", "RX", "RY", "LT", "RT", "A", "B",
    "pose_x", "pose_x_std", "pose_y", "pose_y_std", "pose_theta", "pose_theta_std"
])

plt.ion()
fig, ax = plt.subplots()
trajectory_x = []
trajectory_y = []
traj_plot, = ax.plot([], [], 'b.-', label='Trajectory')
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Real-Time Robot Trajectory")
ax.grid(True)
ax.legend()
plt.show()

try:
    while True:
        updated = False 
        pygame.event.pump()  # Updates internal state

        # Get left stick X and Y
        left_x = joystick.get_axis(0)  # Left stick horizontal
        left_y = -joystick.get_axis(1)  # Left stick vertical

        # Get right stick X and Y
        right_x = joystick.get_axis(2)
        right_y = -joystick.get_axis(3)

        # Get trigger values
        left_trigger = joystick.get_axis(4)
        right_trigger = joystick.get_axis(5)

        # Get button states (example: A, B, X, Y)
        a = joystick.get_button(0)
        b = joystick.get_button(1)
        x = joystick.get_button(2)
        y = joystick.get_button(3)

        msg = (
            f"LX:{left_x:.2f} LY:{left_y:.2f} "
            f"RX:{right_x:.2f} RY:{right_y:.2f} "
            f"LT:{left_trigger:.2f} RT:{right_trigger:.2f} "
            f"A:{a} B:{b} X:{x} Y:{y}"
        )

        # Send controller data to ESP32
        # ser.write((msg + '\n').encode())
        sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))

        timestamp = datetime.datetime.now().isoformat(timespec='milliseconds')
        pose_x = pose_x_std = pose_y = pose_y_std = pose_theta = pose_theta_std = None

        # Print what you send
        # print("Sent:", msg)

        # Read from ESP32 
        # while ser.in_waiting > 0:
        #     response = ser.readline().decode(errors='ignore').strip()
        #     if response:
        #         print(response)

        # Non-blocking receive from ESP32
        ready = select.select([sock], [], [], 0)
        if ready[0]:
            data, addr = sock.recvfrom(1024)
            msg_received = data.decode().strip()
            print("Received from HAMR:", msg_received)

            match = re.search(
                r"X:\s*([-\d\.]+)\s*±\s*([-\d\.]+)\s*m,\s*Y:\s*([-\d\.]+)\s*±\s*([-\d\.]+)\s*m,\s*Theta:\s*([-\d\.]+)\s*±\s*([-\d\.]+)", msg_received)
            if match:
                pose_x, pose_x_std, pose_y, pose_y_std, pose_theta, pose_theta_std = map(float, match.groups())
                if pose_x is not None and pose_y is not None:
                    trajectory_x.append(pose_x)
                    trajectory_y.append(pose_y)
                    updated = True

        if updated:
            traj_plot.set_data(trajectory_x, trajectory_y)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)  # Refresh Rate
            
        csv_writer.writerow([
        timestamp,
        left_x, left_y, right_x, right_y, left_trigger, right_trigger, a, b,
        pose_x, pose_x_std, pose_y, pose_y_std, pose_theta, pose_theta_std
        ])
        csv_file.flush()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting...")
    # ser.close()
    pygame.quit()
    sock.close()
    csv_file.close()
    plt.close(fig)


