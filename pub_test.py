#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64
# import serial, struct, time, zlib, math

# # ----------------- Config -----------------
# SERIAL_PORT = '/dev/ttyUSB0'   # change to your port (e.g., /dev/ttyACM0)
# BAUD        = 2000000

# # Packet: uint16 magic, uint16 ver, uint32 seq, uint64 t_tx_ns, float left, float right, uint16 crc16
# MAGIC = 0xCAFE
# VER   = 1
# PKT_FMT_NOCRC = '<HHIQff'
# PKT_FMT       = '<HHIQffH'
# PKT_SIZE      = struct.calcsize(PKT_FMT)

# def crc16_surrogate(b: bytes) -> int:
#     # CRC32 folded to 16 bits — matches ESP side implementation
#     return zlib.crc32(b) & 0xFFFF

# class PubRelayNode(Node):
#     def __init__(self):
#         super().__init__('pub_relay_node')

#         # ---- Serial link ----
#         self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.02, write_timeout=0.05)
#         self.ser.reset_input_buffer()
#         self.ser.reset_output_buffer()

#         # ---- ROS pubs/subs ----
#         self.left_pub  = self.create_publisher(Float64, '/left_wheel/cmd_vel',  10)
#         self.right_pub = self.create_publisher(Float64, '/right_wheel/cmd_vel', 10)
#         self.create_subscription(Float64, '/left_wheel/cmd_vel',  self.left_cb,  10)
#         self.create_subscription(Float64, '/right_wheel/cmd_vel', self.right_cb, 10)

#         # ---- State ----
#         self.t = 0.0
#         self.left_val  = 0.0
#         self.right_val = 0.0
#         self.seq = 0

#         # ---- Timers ----
#         self.create_timer(0.05, self.gen_tick)   # 20 Hz signal generator
#         self.create_timer(0.01, self.relay_tick) # 100 Hz serial relay w/ RTT
#         self.create_timer(2.0,  self.report_subs)

#         self.get_logger().info(f"Serial on {SERIAL_PORT} @ {BAUD} | merged publisher+relay")

#     # ---------- Publishers ----------
#     def gen_tick(self):
#         # Generate test commands (sin/cos)
#         self.t += 0.1
#         self.left_val  = math.sin(self.t)
#         self.right_val = math.cos(self.t)

#         lm = Float64(); lm.data = self.left_val
#         rm = Float64(); rm.data = self.right_val
#         self.left_pub.publish(lm)
#         self.right_pub.publish(rm)
#         # Optional: print(f"GEN  L={lm.data:.6f}  R={rm.data:.6f}")

#     # ---------- Subscribers ----------
#     def left_cb(self, msg):  self.left_val  = msg.data
#     def right_cb(self, msg): self.right_val = msg.data

#     # ---------- Relay w/ RTT ----------
#     def relay_tick(self):
#         self.seq += 1
#         t_tx_ns = time.monotonic_ns()

#         header = struct.pack(PKT_FMT_NOCRC, MAGIC, VER, self.seq, t_tx_ns, self.left_val, self.right_val)
#         crc    = crc16_surrogate(header)
#         pkt    = struct.pack(PKT_FMT, MAGIC, VER, self.seq, t_tx_ns, self.left_val, self.right_val, crc)

#         try:
#             self.ser.write(pkt)
#             echo = self.ser.read(PKT_SIZE)
#             if len(echo) == PKT_SIZE:
#                 m, v, seq, ts_ns, l, r, c = struct.unpack(PKT_FMT, echo)
#                 if m == MAGIC and v == VER and seq == self.seq and c == crc:
#                     rtt_ms = (time.monotonic_ns() - t_tx_ns) / 1e6
#                     self.get_logger().info(
#                         f"SEQ={seq}  L={self.left_val:.6f}  R={self.right_val:.6f}  "
#                         f"RTT={rtt_ms:.3f} ms  (one-way≈{rtt_ms/2:.3f} ms)"
#                     )
#                 else:
#                     self.get_logger().warn("Echo mismatch (hdr/seq/crc)")
#             # else: no echo this cycle — OK occasionally
#         except serial.SerialTimeoutException:
#             self.get_logger().warn("Serial write timeout")

#     def report_subs(self):
#         self.get_logger().info(
#             f"Subs — Left:{self.left_pub.get_subscription_count()} Right:{self.right_pub.get_subscription_count()} "
#             f"| Current L={self.left_val:.4f} R={self.right_val:.4f}"
#         )

# def main():
#     rclpy.init()
#     node = PubRelayNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.ser.close()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
import serial, struct, time, zlib, math, threading

# ----------------- Serial config -----------------
SERIAL_PORT = '/dev/ttyUSB0'   # change if needed, e.g. /dev/ttyACM0
BAUD        = 460800

# ----------------- Protocol -----------------
MAGIC = 0xCAFE
VER   = 1

TYPE_CMD = 0x01  # PC -> ESP  : wheel commands
TYPE_ENC = 0x02  # ESP -> PC  : encoder ticks

# Packets
# CMD: uint16 magic, uint16 ver, uint16 type, uint32 seq, uint64 t_tx_ns, float left, float right, uint16 crc
CMD_FMT_NOCRC = '<HHHIQff'
CMD_FMT       = '<HHHIQffH'
CMD_SIZE      = struct.calcsize(CMD_FMT)

# ENC: uint16 magic, uint16 ver, uint16 type, uint32 seq, uint64 t_tx_ns, int32 ticksL, int32 ticksR, uint16 crc
ENC_FMT_NOCRC = '<HHHIQii'
ENC_FMT       = '<HHHIQiiH'
ENC_SIZE      = struct.calcsize(ENC_FMT)

def crc16_surrogate(b: bytes) -> int:
    return zlib.crc32(b) & 0xFFFF

class PubRelayEncNode(Node):
    def __init__(self):
        super().__init__('pub_relay_enc_node')

        # ---- Serial ----
        self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.005, write_timeout=0.02)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # ---- ROS pubs/subs ----
        self.left_pub  = self.create_publisher(Float64, '/left_wheel/cmd_vel',  10)
        self.right_pub = self.create_publisher(Float64, '/right_wheel/cmd_vel', 10)

        self.left_enc_pub  = self.create_publisher(Int32, '/left_wheel/encoder_ticks',  10)
        self.right_enc_pub = self.create_publisher(Int32, '/right_wheel/encoder_ticks', 10)

        self.create_subscription(Float64, '/left_wheel/cmd_vel',  self.left_cb,  10)
        self.create_subscription(Float64, '/right_wheel/cmd_vel', self.right_cb, 10)

        # ---- State ----
        self.t = 0.0
        self.left_val  = 0.0
        self.right_val = 0.0
        self.seq = 0
        self.running = True

        # ---- Timers ----
        self.create_timer(0.05, self.gen_tick)   # 20 Hz generator (remove if you have an external controller)
        self.create_timer(0.01, self.tx_tick)    # 100 Hz: send latest commands

        # ---- RX thread for encoder packets ----
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"Serial on {SERIAL_PORT} @ {BAUD} | CMD->ESP, ENC<-ESP")

    # ---------- Command generation (optional) ----------
    def gen_tick(self):
        self.t += 0.1
        self.left_val  = math.sin(self.t)
        self.right_val = math.cos(self.t)
        self.left_pub.publish(Float64(data=self.left_val))
        self.right_pub.publish(Float64(data=self.right_val))

    # ---------- Subscribers (if others publish commands) ----------
    def left_cb(self, msg):  self.left_val  = msg.data
    def right_cb(self, msg): self.right_val = msg.data

    # ---------- TX: send wheel commands ----------
    def tx_tick(self):
        self.seq += 1
        t_tx_ns = time.monotonic_ns()
        header = struct.pack(CMD_FMT_NOCRC, MAGIC, VER, TYPE_CMD, self.seq, t_tx_ns, self.left_val, self.right_val)
        crc    = crc16_surrogate(header)
        pkt    = struct.pack(CMD_FMT, MAGIC, VER, TYPE_CMD, self.seq, t_tx_ns, self.left_val, self.right_val, crc)
        try:
            self.ser.write(pkt)
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timeout")

    # ---------- RX: read encoder packets ----------
    def rx_loop(self):
        # Simple framing: read in fixed chunks and validate; if mismatch, resync by shifting one byte
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(64)  # small-ish grab
                if not chunk:
                    continue
                buf += chunk

                # Try parsing while we have at least ENC_SIZE bytes
                while len(buf) >= ENC_SIZE:
                    # Quick MAGIC check at pos 0
                    m, = struct.unpack_from('<H', buf, 0)
                    if m != MAGIC:
                        # resync: drop 1 byte and continue
                        buf.pop(0)
                        continue

                    # Peek type
                    try:
                        # magic(2) ver(2) type(2)
                        _, _, typ = struct.unpack_from('<HHH', buf, 0)
                    except struct.error:
                        break  # need more data

                    if typ == TYPE_ENC and len(buf) >= ENC_SIZE:
                        pkt_bytes = bytes(buf[:ENC_SIZE])
                        # Validate CRC
                        crc_calc = crc16_surrogate(pkt_bytes[:-2])
                        m, v, t, seq, t_ns, ticksL, ticksR, crc = struct.unpack(ENC_FMT, pkt_bytes)
                        if crc_calc == crc and v == VER and m == MAGIC:
                            # Good packet: publish ticks
                            self.left_enc_pub.publish(Int32(data=int(ticksL)))
                            self.right_enc_pub.publish(Int32(data=int(ticksR)))
                            # consume
                            del buf[:ENC_SIZE]
                        else:
                            # bad frame; resync one byte
                            buf.pop(0)
                    elif typ == TYPE_CMD:
                        # If ESP ever mirrors CMD back (shouldn't), skip size
                        if len(buf) >= CMD_SIZE:
                            del buf[:CMD_SIZE]
                        else:
                            break
                    else:
                        # Unknown type: drop 1 byte
                        buf.pop(0)
            except Exception as e:
                # avoid tight error loops
                time.sleep(0.01)

    def destroy_node(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = PubRelayEncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
