#!/usr/bin/env python3
import serial
import matplotlib.pyplot as plt
import re
import time
import glob
import sys
import os
from collections import deque

# =========================
# Config
# =========================
BAUD = 115200
PREFERRED_PORT = '/dev/ttyUSB0'  # change if you like; auto-detect will be used if not found
DEBUG = False                    # set True to see why lines were skipped
MAX_POINTS = 3000                # rolling window for plotting; set None for unlimited

# =========================
# Serial port auto-detect
# =========================
def find_serial_port(preferred=None):
    if preferred and os.path.exists(preferred):
        return preferred
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return candidates[0] if candidates else None

port = find_serial_port(PREFERRED_PORT)
if not port:
    print("ERROR: No serial device under /dev/ttyUSB* or /dev/ttyACM*. Plug in your board.")
    sys.exit(1)

print(f"Using serial port: {port}")
try:
    ser = serial.Serial(port, BAUD, timeout=1)
except serial.SerialException as e:
    print(f"ERROR opening {port}: {e}")
    sys.exit(1)

time.sleep(2)  # let the board reset

# =========================
# Robust parsers (regex)
# =========================
NUM = r"[+\-]?\d+(?:\.\d+)?"
INT = r"[+\-]?\d+"

rpm_L_re = re.compile(rf"\bL:\s*({NUM})\s*RPM\b", re.I)
rpm_R_re = re.compile(rf"\bR:\s*({NUM})\s*RPM\b", re.I)
errT_re  = re.compile(rf"\bError[_ ]?Turret:\s*({NUM})\b", re.I)

# Tolerant to order/spacing inside section
pwm_L_re = re.compile(rf"\bPWM\b.*?\bL\s*=\s*({INT})", re.I | re.S)
pwm_R_re = re.compile(rf"\bPWM\b.*?\bR\s*=\s*({INT})", re.I | re.S)
pwm_T_re = re.compile(rf"\bPWM\b.*?\bT\s*=\s*({INT})", re.I | re.S)

rot_L_re = re.compile(rf"\bRot\b.*?\bL\s*=\s*({NUM})", re.I | re.S)
rot_R_re = re.compile(rf"\bRot\b.*?\bR\s*=\s*({NUM})", re.I | re.S)
t_ang_re = re.compile(rf"\b(T_angle|T|theta)\s*=\s*({NUM})", re.I)

def parse_line(line: str):
    # Normalize Unicode minus (U+2212) to ASCII hyphen-minus
    line = line.replace("\u2212", "-")

    mL  = rpm_L_re.search(line)
    mR  = rpm_R_re.search(line)
    mPL = pwm_L_re.search(line)
    mPR = pwm_R_re.search(line)

    if not all([mL, mR, mPL, mPR]):
        reason = "Missing: " + ", ".join(
            name for name, m in [
                ("L_rpm", mL), ("R_rpm", mR), ("PWM_L", mPL), ("PWM_R", mPR)
            ] if m is None
        )
        return None, reason

    data = {
        "rpmL": float(mL.group(1)),
        "rpmR": float(mR.group(1)),
        "pwmL": int(mPL.group(1)),
        "pwmR": int(mPR.group(1)),
    }

    # Optional fields
    mRL = rot_L_re.search(line);  mRR = rot_R_re.search(line)
    mPT = pwm_T_re.search(line);  mET = errT_re.search(line); mTA = t_ang_re.search(line)

    if mRL: data["rotL"] = float(mRL.group(1))
    if mRR: data["rotR"] = float(mRR.group(1))
    if mPT: data["pwmT"] = int(mPT.group(1))
    if mET: data["errorT"] = float(mET.group(1))
    if mTA: data["t_angle"] = float(mTA.group(2))

    return data, None

# =========================
# Plot selection
# =========================
print("Enter which plots to show separated by space (options: RPM PWM ERROR PID), or 'all' to show all:")
user_input = input().strip().lower()
if user_input == 'all':
    selected_plots = ['rpm', 'pwm', 'error', 'pid']
else:
    selected_plots = [s for s in user_input.split() if s in ('rpm', 'pwm', 'error', 'pid')]
    if not selected_plots:
        selected_plots = ['rpm', 'pwm', 'error', 'pid']

num_plots = len(selected_plots)
if num_plots == 1:
    nrows, ncols = 1, 1
elif num_plots == 2:
    nrows, ncols = 1, 2
else:
    nrows, ncols = 2, 2

# =========================
# Data buffers (rolling)
# =========================
def dq(): return deque(maxlen=MAX_POINTS) if MAX_POINTS else []
times   = dq()
rpmL_d  = dq()
rpmR_d  = dq()
pwmL_d  = dq()
pwmR_d  = dq()
rotL_d  = dq()
rotR_d  = dq()

start_time = time.time()

# =========================
# Plot setup
# =========================
plt.ion()
fig, axs = plt.subplots(nrows, ncols, figsize=(4*ncols, 3*nrows))
fig.suptitle('Motor PID Telemetry')
fig.text(0.5, 0.04, 'Time (s)', ha='center', va='center', fontsize=12)

if isinstance(axs, plt.Axes):
    axs = [axs]
else:
    axs = axs.flatten()

plot_lines = []
for i, plot_name in enumerate(selected_plots):
    ax = axs[i]
    if plot_name == 'rpm':
        line, = ax.plot([], [], label='RPM Error (L-R)')
        ax.set_title('RPM Difference')
        ax.set_ylabel('RPM')
        ax.grid(True); ax.legend()
        plot_lines.append(('rpm', ax, line))
    elif plot_name == 'pwm':
        lineL, = ax.plot([], [], label='PWM Left')
        lineR, = ax.plot([], [], label='PWM Right')
        ax.set_title('PWM Values')
        ax.set_ylabel('PWM (signed)')
        ax.grid(True); ax.legend()
        plot_lines.append(('pwm', ax, (lineL, lineR)))
    elif plot_name == 'error':
        line, = ax.plot([], [], label='Rotation Error (L-R)')
        ax.set_title('Rotation Error Over Time')
        ax.set_ylabel('Rotation Error (rev)')
        ax.grid(True); ax.legend()
        plot_lines.append(('error', ax, line))
    elif plot_name == 'pid':
        lineL, = ax.plot([], [], label='RPM Left (Slave)')
        lineR, = ax.plot([], [], label='RPM Right (Master)', linestyle='dashed')
        ax.set_title('PID Response (Master vs Slave)')
        ax.set_ylabel('RPM')
        ax.grid(True); ax.legend()
        plot_lines.append(('pid', ax, (lineL, lineR)))
    else:
        ax.set_visible(False)

for j in range(len(selected_plots), len(axs)):
    axs[j].set_visible(False)

# =========================
# Main loop
# =========================
try:
    while True:
        line_bytes = ser.readline()
        if not line_bytes:
            continue

        line_str = line_bytes.decode(errors='ignore').strip()
        if not line_str:
            continue

        # Skip obvious non-telemetry chatter quickly
        if "L:" not in line_str or "RPM" not in line_str or "PWM" not in line_str:
            if DEBUG:
                print("SKIP →", repr(line_str))
            continue

        data, why = parse_line(line_str)
        if data is None:
            if DEBUG:
                print("NO MATCH →", repr(line_str), "| reason:", why)
            continue

        # --- Parsed values ---
        t = time.time() - start_time
        rpmL = data["rpmL"]; rpmR = data["rpmR"]
        pwmL = data["pwmL"]; pwmR = data["pwmR"]
        rotL = data.get("rotL"); rotR = data.get("rotR")

        # ✅ Print parsed telemetry every time we plot
        print(
            f"t={t:.2f}s | "
            f"rpmL={rpmL:.2f}, rpmR={rpmR:.2f} | "
            f"pwmL={pwmL}, pwmR={pwmR} | "
            f"rotL={rotL if rotL is not None else 'NA'}, "
            f"rotR={rotR if rotR is not None else 'NA'}"
        )

        # Append buffers
        (times.append(t) if isinstance(times, deque) else times.append(t))
        (rpmL_d.append(rpmL) if isinstance(rpmL_d, deque) else rpmL_d.append(rpmL))
        (rpmR_d.append(rpmR) if isinstance(rpmR_d, deque) else rpmR_d.append(rpmR))
        (pwmL_d.append(pwmL) if isinstance(pwmL_d, deque) else pwmL_d.append(pwmL))
        (pwmR_d.append(pwmR) if isinstance(pwmR_d, deque) else pwmR_d.append(pwmR))
        if rotL is not None: (rotL_d.append(rotL) if isinstance(rotL_d, deque) else rotL_d.append(rotL))
        if rotR is not None: (rotR_d.append(rotR) if isinstance(rotR_d, deque) else rotR_d.append(rotR))

        # Update plots
        for name, ax, lines in plot_lines:
            if name == 'rpm':
                rpm_error = [a - b for a, b in zip(rpmL_d, rpmR_d)]
                lines.set_data(times, rpm_error)
            elif name == 'pwm':
                lineL, lineR = lines
                lineL.set_data(times, pwmL_d)
                lineR.set_data(times, pwmR_d)
            elif name == 'error':
                if rotL_d and rotR_d:
                    rot_error = [a - b for a, b in zip(rotL_d, rotR_d)]
                    lines.set_data(times, rot_error)
            elif name == 'pid':
                lineL, lineR = lines
                lineL.set_data(times, rpmL_d)
                lineR.set_data(times, rpmR_d)

            ax.relim()
            ax.autoscale_view()

        plt.pause(0.01)

except KeyboardInterrupt:
    print("\nPlotting stopped by user.")
finally:
    try:
        ser.close()
    except Exception:
        pass
    plt.ioff()
    plt.show()


# import serial
# import matplotlib.pyplot as plt
# import re
# import time
# import math
# import socket

# # Prompt user for which plots to display
# print("Enter which plots to show separated by space (options: RPM PWM ERROR PID), or 'all' to show all:")
# user_input = input().strip().lower()
# if user_input == 'all':
#     selected_plots = ['rpm', 'pwm', 'error', 'pid']
# else:
#     selected_plots = user_input.split()

# num_plots = len(selected_plots)

# # Determine subplot grid shape dynamically
# if num_plots == 1:
#     nrows, ncols = 1, 1
# elif num_plots == 2:
#     nrows, ncols = 1, 2
# elif num_plots == 3:
#     nrows, ncols = 2, 2
# elif num_plots == 4:
#     nrows, ncols = 2, 2
# else:
#     nrows = (num_plots + 1) // 2
#     ncols = 2

# # Open serial port - update COM port as needed
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
# time.sleep(2)
# line = ser.readline()

# # Data buffers
# times = []
# rpmL_data = []
# rpmR_data = []
# pwmL_data = []
# pwmR_data = []
# rotL_data = []
# rotR_data = []

# start_time = time.time()

# plt.ion()
# fig, axs = plt.subplots(nrows, ncols, figsize=(4*ncols, 3*nrows))
# fig.suptitle('Motor PID Telemetry')

# # Add single x-axis label for entire figure, centered below all subplots
# fig.text(0.5, 0.04, 'Time (s)', ha='center', va='center', fontsize=12)

# # Flatten axs to 1D list for easier indexing even if 1D or 2D
# if isinstance(axs, plt.Axes):
#     axs = [axs]
# else:
#     axs = axs.flatten()

# plot_lines = []

# # Prepare plots as per selection
# for i, plot_name in enumerate(selected_plots):
#     ax = axs[i]
#     if plot_name == 'rpm':
#         line, = ax.plot([], [], label='RPM Error (L-R)', color='blue')
#         ax.set_title('RPM Difference')
#         ax.set_ylabel('RPM')
#         ax.grid(True)
#         ax.legend()
#         plot_lines.append(('rpm', ax, line))
#     elif plot_name == 'pwm':
#         lineL, = ax.plot([], [], label='PWM Left', color='red')
#         lineR, = ax.plot([], [], label='PWM Right', color='green')
#         ax.set_title('PWM Values')
#         ax.set_ylabel('PWM (0-4095)')
#         ax.grid(True)
#         ax.legend()
#         plot_lines.append(('pwm', ax, (lineL, lineR)))
#     elif plot_name == 'error':
#         line, = ax.plot([], [], label='Rotation Error (L-R)', color='magenta')
#         ax.set_title('Rotation Error Over Time')
#         ax.set_ylabel('Rotation Error (rev)')
#         ax.grid(True)
#         ax.legend()
#         plot_lines.append(('error', ax, line))
#     elif plot_name == 'pid':
#         lineL, = ax.plot([], [], label='RPM Left (Slave)', color='blue')
#         lineR, = ax.plot([], [], label='RPM Right (Master)', color='black', linestyle='dashed')
#         ax.set_title('PID Response (Master vs Slave)')
#         ax.set_ylabel('RPM')
#         ax.grid(True)
#         ax.legend()
#         plot_lines.append(('pid', ax, (lineL, lineR)))
#     else:
#         ax.set_visible(False)  # Hide any unexpected plots

# # Hide any extra unused subplots
# for j in range(len(selected_plots), len(axs)):
#     axs[j].set_visible(False)

# # Regex pattern for serial input parsing
# # pattern = re.compile(
# #     r"L:\s*(-?\d+\.\d+)\s*RPM\s*\|\s*R:\s*(-?\d+\.\d+)\s*RPM\s*\|"
# #     r"\s*Error_Turret:\s*(-?\d+\.\d+)\s*\|"
# #     r"\s*PWM:\s*L=(\d+),\s*R=(\d+),\s*T=(\d+)\s*\|"
# #     r"\s*Rot:\s*L=(-?\d+\.\d+),\s*R=(-?\d+\.\d+),\s*T_angle=(-?\d+\.\d+)"
# # )

# number = r"-?\d+(?:\.\d+)?"
# pattern = re.compile(
#     rf"L:\s*({number})\s*RPM\s*\|\s*R:\s*({number})\s*RPM\s*\|"
#     rf"\s*Error_Turret:\s*({number})\s*\|"
#     rf"\s*PWM:\s*L=(\d+),\s*R=(\d+),\s*T=(\d+)\s*\|"
#     rf"\s*Rot:\s*L=({number}),\s*R=({number}),\s*T_angle=({number})"
# )

# try:
#     while True:
#         line_bytes = ser.readline()
#         line_str = line_bytes.decode(errors='ignore').strip()

#         match = pattern.search(line_str)
#         if match:
#             rpmL = float(match.group(1))
#             rpmR = float(match.group(2))
#             errorT = float(match.group(3))
#             pwmL = int(match.group(4))
#             pwmR = int(match.group(5))
#             pwmT = int(match.group(6))
#             rotL = float(match.group(7))
#             rotR = float(match.group(8))
#             t_angle = float(match.group(9))


#             t = time.time() - start_time

#             times.append(t)
#             rpmL_data.append(rpmL)
#             rpmR_data.append(rpmR)
#             pwmL_data.append(pwmL)
#             pwmR_data.append(pwmR)
#             rotL_data.append(rotL)
#             rotR_data.append(rotR)

#             for name, ax, lines in plot_lines:
#                 if name == 'rpm':
#                     rpm_error = [a - b for a, b in zip(rpmL_data, rpmR_data)]
#                     lines.set_data(times, rpm_error)
#                 elif name == 'pwm':
#                     lineL, lineR = lines
#                     lineL.set_data(times, pwmL_data)
#                     lineR.set_data(times, pwmR_data)
#                 elif name == 'error':
#                     rot_error = [a - b for a, b in zip(rotL_data, rotR_data)]
#                     lines.set_data(times, rot_error)
#                 elif name == 'pid':
#                     lineL, lineR = lines
#                     lineL.set_data(times, rpmL_data)
#                     lineR.set_data(times, rpmR_data)

#                 ax.relim()
#                 ax.autoscale_view()

#             plt.pause(0.01)
#             print(f"Data: {line_str}")
        
#         else:
#             print("No match")

# except KeyboardInterrupt:
#     print("Plotting stopped by user")

# import serial
# import time
# import matplotlib.pyplot as plt
# import re

# # Initialize serial port
# ser = serial.Serial('COM7', 115200, timeout=1)
# time.sleep(2)

# # Data buffers
# pwms = []
# rpms = []

# # Regex pattern to extract PWM and RPM
# pattern = re.compile(r"PWM:\s*(\d+)\s*\|\s*RPM:\s*(-?\d+\.?\d*)")

# # Setup interactive plot
# plt.ion()
# fig, ax = plt.subplots()
# ax.set_title('Real-Time PWM vs RPM')
# ax.set_xlabel('PWM (0–4095)')
# ax.set_ylabel('RPM')
# ax.grid(True)

# # Keep track of line segments
# line_segments = []

# try:
#     print("Reading and plotting live... Press Ctrl+C to stop.\n")

#     while True:
#         line = ser.readline().decode(errors='ignore').strip()
#         match = pattern.search(line)

#         if match:
#             pwm = int(match.group(1))
#             rpm = float(match.group(2))

#             if len(pwms) > 0:
#                 color = 'blue' if pwm > pwms[-1] else 'red'
#                 ax.plot([pwms[-1], pwm], [rpms[-1], rpm], color=color)
#                 plt.pause(0.01)

#             pwms.append(pwm)
#             rpms.append(rpm)

# except KeyboardInterrupt:
#     print("\nFinished. Saving plot...")
#     plt.ioff()
#     plt.savefig('pwm_vs_rpm_live.png')
#     plt.show()




# import serial
# import matplotlib.pyplot as plt
# import re
# import time

# # Open serial port - update COM port as needed
# ser = serial.Serial('COM7', 115200, timeout=1)
# time.sleep(2)

# # Data buffers
# times = []
# rpmL_data = []
# rpmR_data = []
# pwmL_data = []
# pwmR_data = []
# rotL_data = []
# rotR_data = []

# start_time = time.time()

# # Initialize 2x2 subplot
# plt.ion()
# fig, axs = plt.subplots(2, 2, figsize=(12, 8))
# fig.suptitle('Motor PID Telemetry')

# # Lines for plots
# line_rpm_error, = axs[0, 0].plot([], [], label='RPM Error (L-R)', color='blue')
# axs[0, 0].set_title('RPM Difference')
# axs[0, 0].set_xlabel('Time (s)')
# axs[0, 0].set_ylabel('RPM')
# axs[0, 0].grid(True)
# axs[0, 0].legend()

# line_pwmL, = axs[0, 1].plot([], [], label='PWM Left', color='red')
# line_pwmR, = axs[0, 1].plot([], [], label='PWM Right', color='green')
# axs[0, 1].set_title('PWM Values')
# axs[0, 1].set_xlabel('Time (s)')
# axs[0, 1].set_ylabel('PWM (0-4095)')
# axs[0, 1].grid(True)
# axs[0, 1].legend()

# line_rotL, = axs[1, 0].plot([], [], label='Rotation Left', color='purple')
# line_rotR, = axs[1, 0].plot([], [], label='Rotation Right', color='orange')
# axs[1, 0].set_title('Rotations Over Time')
# axs[1, 0].set_xlabel('Time (s)')
# axs[1, 0].set_ylabel('Rotations (rev)')
# axs[1, 0].grid(True)
# axs[1, 0].legend()

# line_rpmL, = axs[1, 1].plot([], [], label='RPM Left (Slave)', color='blue')
# line_rpmR, = axs[1, 1].plot([], [], label='RPM Right (Master)', color='black', linestyle='dashed')
# axs[1, 1].set_title('PID Response (Master vs Slave)')
# axs[1, 1].set_xlabel('Time (s)')
# axs[1, 1].set_ylabel('RPM')
# axs[1, 1].grid(True)
# axs[1, 1].legend()

# # Regex to extract all needed values from your serial print line
# pattern = re.compile(
#     r"L:\s*(-?\d+\.\d+)\s*RPM\s*\|\s*R:\s*(-?\d+\.\d+)\s*RPM\s*\|\s*PWM:\s*L=(\d+),\s*R=(\d+)\s*\|\s*Rot:\s*L=([0-9\.]+),\s*R=([0-9\.]+)"
# )

# try:
#     while True:
#         line_bytes = ser.readline()
#         line_str = line_bytes.decode(errors='ignore').strip()

#         match = pattern.search(line_str)
#         if match:
#             rpmL = float(match.group(1))
#             rpmR = float(match.group(2))
#             pwmL = int(match.group(3))
#             pwmR = int(match.group(4))
#             rotL = float(match.group(5))
#             rotR = float(match.group(6))

#             t = time.time() - start_time

#             # Append data
#             times.append(t)
#             rpmL_data.append(rpmL)
#             rpmR_data.append(rpmR)
#             pwmL_data.append(pwmL)
#             pwmR_data.append(pwmR)
#             rotL_data.append(rotL)
#             rotR_data.append(rotR)

#             # Update plots
#             # 1. RPM Error
#             rpm_error = [a - b for a, b in zip(rpmL_data, rpmR_data)]
#             line_rpm_error.set_data(times, rpm_error)
#             axs[0, 0].relim()
#             axs[0, 0].autoscale_view()

#             # 2. PWM
#             line_pwmL.set_data(times, pwmL_data)
#             line_pwmR.set_data(times, pwmR_data)
#             axs[0, 1].relim()
#             axs[0, 1].autoscale_view()

#             # 3. Rotations
#             line_rotL.set_data(times, rotL_data)
#             line_rotR.set_data(times, rotR_data)
#             axs[1, 0].relim()
#             axs[1, 0].autoscale_view()

#             # 4. PID Response
#             line_rpmL.set_data(times, rpmL_data)
#             line_rpmR.set_data(times, rpmR_data)
#             axs[1, 1].relim()
#             axs[1, 1].autoscale_view()

#             plt.pause(0.01)

# except KeyboardInterrupt:
#     print("Plotting stopped by user")


# import serial
# import matplotlib.pyplot as plt
# import re
# import time

# # Adjust COM port and baud rate to match your board
# ser = serial.Serial('COM7', 115200, timeout=1)
# time.sleep(2)

# # Initialize plot
# plt.ion()
# fig, ax = plt.subplots()
# times, errors = [], []
# line, = ax.plot([], [], label='RPM Error (L - R)', color='b')
# ax.set_title("Motor Synchronization PID")
# ax.set_xlabel("Time (s)")
# ax.set_ylabel("RPM Difference")
# ax.grid(True)
# ax.legend()

# start = time.time()

# while True:
#     try:
#         line_bytes = ser.readline()
#         line_str = line_bytes.decode(errors='ignore').strip()

#         # Match this line: L: 25.4 RPM | R: 22.1 RPM | ...
#         match = re.search(r"L:\s*(-?\d+\.\d+)\s*RPM\s*\|\s*R:\s*(-?\d+\.\d+)", line_str)
#         if match:
#             rpmL = float(match.group(1))
#             rpmR = float(match.group(2))
#             err = rpmL - rpmR
#             t = time.time() - start

#             times.append(t)
#             errors.append(err)

#             # Update plot
#             line.set_xdata(times)
#             line.set_ydata(errors)
#             ax.relim()
#             ax.autoscale_view()
#             plt.draw()
#             plt.pause(0.01)

#     except KeyboardInterrupt:
#         print("Stopped by user.")
#         break
