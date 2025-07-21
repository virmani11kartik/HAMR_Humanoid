import pygame
import serial
import time

# Change COM to your ESP32 port
ser = serial.Serial('COM4', 115200, timeout=0.1)
time.sleep(2)  # wait for ESP to reset

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to controller: {joystick.get_name()}")

try:
    while True:
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
        ser.write((msg + '\n').encode())

        # Print what you send
        # print("Sent:", msg)

        # Read from ESP32 
        while ser.in_waiting > 0:
            response = ser.readline().decode(errors='ignore').strip()
            if response:
                print(response)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
    pygame.quit()
