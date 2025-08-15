#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial, struct, time, zlib

MAGIC = 0xCAFE
VER = 1

# Packet: uint16 magic, uint16 ver, uint32 seq, uint64 t_tx_ns, float left, float right, uint16 crc16
PKT_FMT_NOCRC = '<HHIQff'
PKT_FMT       = '<HHIQffH'
PKT_SIZE      = struct.calcsize(PKT_FMT)

def crc16_surrogate(b: bytes) -> int:
    # Matches ESP’s surrogate: CRC32 folded to 16 bits
    return zlib.crc32(b) & 0xFFFF

class SerialRelayNode(Node):
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        super().__init__('serial_relay_node')
        self.ser = serial.Serial(port, baud, timeout=0.02, write_timeout=0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.left_val = 0.0
        self.right_val = 0.0
        self.seq = 0

        self.create_subscription(Float64, '/left_wheel/cmd_vel', self.left_cb, 10)
        self.create_subscription(Float64, '/right_wheel/cmd_vel', self.right_cb, 10)

        # Send at 100 Hz for smoother latency stats (OK even if publishers are slower)
        self.timer = self.create_timer(0.01, self.tick)

    def left_cb(self, msg):  self.left_val  = msg.data
    def right_cb(self, msg): self.right_val = msg.data

    def tick(self):
        # Build packet
        self.seq += 1
        t_tx_ns = time.monotonic_ns()
        header = struct.pack(PKT_FMT_NOCRC, MAGIC, VER, self.seq, t_tx_ns, self.left_val, self.right_val)
        crc = crc16_surrogate(header)
        pkt = struct.pack(PKT_FMT, MAGIC, VER, self.seq, t_tx_ns, self.left_val, self.right_val, crc)

        # Write & wait for echo
        try:
            self.ser.write(pkt)
            echo = self.ser.read(PKT_SIZE)
            if len(echo) == PKT_SIZE:
                # Check it’s the same packet back (simple verify seq & crc)
                m, v, seq, t_ns, l, r, c = struct.unpack(PKT_FMT, echo)
                if m == MAGIC and v == VER and c == crc and seq == self.seq:
                    rtt_ms = (time.monotonic_ns() - t_tx_ns) / 1e6
                    self.get_logger().info(
                        f"SEQ={seq}  L={self.left_val:.6f}  R={self.right_val:.6f}  "
                        f"RTT={rtt_ms:.3f} ms  (one-way≈{rtt_ms/2:.3f} ms)"
                    )
                else:
                    self.get_logger().warn("Echo mismatch (bad hdr/seq/crc)")
            else:
                # No echo this cycle—OK at times, just note
                self.get_logger().debug("No echo")
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timeout")

def main():
    rclpy.init()
    node = SerialRelayNode(port='/dev/ttyUSB0', baud=115200)  # ⬅ change port if needed (e.g., /dev/ttyACM0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
