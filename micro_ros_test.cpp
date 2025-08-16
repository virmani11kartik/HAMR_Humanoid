#include <Arduino.h>
#include <WiFi.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <sys/time.h>
#include <cstring>

#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>

// Forward declare to avoid pulling uXRCE headers
struct uxrCustomTransport;

// --- Wi-Fi AP CONFIG ---
static const char* AP_SSID = "ESP32S2_microROS_AP";
static const char* AP_PASS = "esp32s2pass";
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GW(192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);

// --- Agent CONFIG ---
static const char* AGENT_IP   = "192.168.4.2";
static const uint16_t AGENT_PORT = 8888;

// --- micro-ROS objects ---
rcl_subscription_t left_sub, right_sub;
std_msgs__msg__Float64 left_msg, right_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// --- UDP transport state ---
struct UdpTransport {
  int sock = -1;
  struct sockaddr_in agent_addr;
  struct timeval rx_timeout;
} udp;

// --- Custom transport callbacks ---
extern "C" bool custom_transport_open(struct uxrCustomTransport*) {
  udp.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udp.sock < 0) return false;

  udp.rx_timeout.tv_sec = 0;
  udp.rx_timeout.tv_usec = 20000;
  setsockopt(udp.sock, SOL_SOCKET, SO_RCVTIMEO, &udp.rx_timeout, sizeof(udp.rx_timeout));

  memset(&udp.agent_addr, 0, sizeof(udp.agent_addr));
  udp.agent_addr.sin_family = AF_INET;
  udp.agent_addr.sin_port = htons(AGENT_PORT);
  udp.agent_addr.sin_addr.s_addr = inet_addr(AGENT_IP);
  return true;
}

extern "C" bool custom_transport_close(struct uxrCustomTransport*) {
  if (udp.sock >= 0) {
    close(udp.sock);
    udp.sock = -1;
  }
  return true;
}

extern "C" size_t custom_transport_write(struct uxrCustomTransport*, const uint8_t* buf, size_t len, uint8_t* err) {
  ssize_t sent = sendto(udp.sock, buf, len, 0, (struct sockaddr*)&udp.agent_addr, sizeof(udp.agent_addr));
  if (sent < 0) { if (err) *err = 1; return 0; }
  if (err) *err = 0;
  return (size_t)sent;
}

extern "C" size_t custom_transport_read(struct uxrCustomTransport*, uint8_t* buf, size_t len, int, uint8_t* err) {
  struct sockaddr_in from; socklen_t fromlen = sizeof(from);
  ssize_t recvd = recvfrom(udp.sock, buf, len, 0, (struct sockaddr*)&from, &fromlen);
  if (recvd < 0) { if (err) *err = 1; return 0; }
  if (err) *err = 0;
  return (size_t)recvd;
}

// --- Callbacks for subscribers ---
void left_cb(const void* msgin) {
  auto* msg = (const std_msgs__msg__Float64*)msgin;
  Serial0.print("[left] "); Serial0.println(msg->data, 6);
}

void right_cb(const void* msgin) {
  auto* msg = (const std_msgs__msg__Float64*)msgin;
  Serial0.print("[right] "); Serial0.println(msg->data, 6);
}

// --- Helper for RCL errors ---
static inline void CHECK_RCL(rcl_ret_t rc, const char* what) {
  if (rc != RCL_RET_OK) {
    Serial0.print("ERR "); Serial0.println(what);
    while (true) { delay(1000); }
  }
}

void setup() {
  delay(2000); // Give time after upload
  Serial0.begin(115200);
  Serial0.println("Booting...");

  // Wi-Fi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial0.println("AP start FAILED");
    while (true) { delay(1000); }
  }
  Serial0.print("AP IP: "); Serial0.println(WiFi.softAPIP());

  // Register transport
  rmw_uros_set_custom_transport(false, nullptr,
      custom_transport_open, custom_transport_close,
      custom_transport_write, custom_transport_read);

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  CHECK_RCL(rclc_support_init(&support, 0, NULL, &allocator), "support_init");
  CHECK_RCL(rclc_node_init_default(&node, "esp32s2_sub_node", "", &support), "node_init");

  std_msgs__msg__Float64__init(&left_msg);
  std_msgs__msg__Float64__init(&right_msg);

  CHECK_RCL(rclc_subscription_init_default(
      &left_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "/left_wheel/cmd_vel"), "left_sub");

  CHECK_RCL(rclc_subscription_init_default(
      &right_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "/right_wheel/cmd_vel"), "right_sub");

  CHECK_RCL(rclc_executor_init(&executor, &support.context, 2, &allocator), "exec_init");
  CHECK_RCL(rclc_executor_add_subscription(&executor, &left_sub, &left_msg, &left_cb, ON_NEW_DATA), "exec_add_left");
  CHECK_RCL(rclc_executor_add_subscription(&executor, &right_sub, &right_msg, &right_cb, ON_NEW_DATA), "exec_add_right");

  Serial0.println("Ready to receive /left_wheel/cmd_vel and /right_wheel/cmd_vel");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  delay(1);
}



#include <Arduino.h>

struct __attribute__((packed)) Packet {
  uint16_t magic;   // 0xCAFE
  uint16_t ver;     // 1
  uint32_t seq;
  uint64_t t_tx_ns;
  float left;
  float right;
  uint16_t crc16;   // CRC32-folded-to-16 surrogate
};

static constexpr uint16_t MAGIC = 0xCAFE;
static constexpr uint16_t VER   = 1;
static constexpr size_t   PKT_SIZE = sizeof(Packet);

// CRC32 -> fold to 16 bits (matches Python)
uint16_t crc16_surrogate(const uint8_t* data, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i=0;i<n;i++) {
    c ^= data[i];
    for (int k=0;k<8;k++) {
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
  }
  c ^= 0xFFFFFFFFu;
  return (uint16_t)(c & 0xFFFF);
}

void setup() {
  Serial.begin(2000000);   // USB CDC logs
  Serial0.begin(2000000);  // Data on UART0 from Python (pins vary by board)
  Serial.println("ESP32 UART RTT relay ready.");
}

void loop() {
  static uint8_t buf[PKT_SIZE];
  static size_t  idx = 0;
  static uint32_t last_seq = 0;
  static bool have_last = false;

  while (Serial0.available()) {
    buf[idx++] = (uint8_t)Serial0.read();
    if (idx < PKT_SIZE) continue;

    // Full packet received
    Packet pkt;
    memcpy(&pkt, buf, PKT_SIZE);
    idx = 0; // reset buffer (simple 1-packet framing; good at low error rate)

    // Validate header/CRC
    if (pkt.magic != MAGIC || pkt.ver != VER) {
      Serial.println("Bad header; dropping");
      continue;
    }
    uint16_t calc = crc16_surrogate((uint8_t*)&pkt, PKT_SIZE - 2);
    if (calc != pkt.crc16) {
      Serial.println("CRC mismatch; dropping");
      continue;
    }

    // Drop detection
    if (have_last && pkt.seq != last_seq + 1) {
      uint32_t drops = (pkt.seq - last_seq - 1);
      Serial.printf("⚠ Drop(s): %u (last=%u, now=%u)\n", drops, last_seq, pkt.seq);
    }
    last_seq = pkt.seq;
    have_last = true;

    // Log values
    Serial.printf("SEQ=%u  L=%.6f  R=%.6f\n", pkt.seq, pkt.left, pkt.right);

    // Echo back immediately for RTT measurement on Python side
    Serial0.write((uint8_t*)&pkt, PKT_SIZE);
  }
}



#include <Arduino.h>

// ----------------- Protocol -----------------
static const uint16_t MAGIC = 0xCAFE;
static const uint16_t VER   = 1;
static const uint16_t TYPE_CMD  = 0x0001; // PC->ESP : left,right
static const uint16_t TYPE_CMD3 = 0x0011; // PC->ESP : left,right,turret
static const uint16_t TYPE_ENC  = 0x0002; // ESP->PC : encoders

#pragma pack(push,1)
struct CmdPacket {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  float left, right;
  uint16_t crc16;
};
struct Cmd3Packet {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  float left, right, turret;
  uint16_t crc16;
};
struct EncPacket {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  int32_t ticksL, ticksR;
  uint16_t crc16;
};
#pragma pack(pop)

static const size_t CMD_SIZE  = sizeof(CmdPacket);   // 2-float
static const size_t CMD3_SIZE = sizeof(Cmd3Packet);  // 3-float
static const size_t ENC_SIZE  = sizeof(EncPacket);

// CRC32->16 surrogate (must match PC side)
uint16_t crc16_surrogate(const uint8_t* data, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i=0;i<n;i++) {
    c ^= data[i];
    for (int k=0;k<8;k++) {
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
  }
  c ^= 0xFFFFFFFFu;
  return (uint16_t)(c & 0xFFFF);
}

// ---- Your encoder sources (replace with ISR counters) ----
volatile int32_t ticksL = 0;
volatile int32_t ticksR = 0;

// ---- Motor command hooks ----
void setMotorCommands(float left, float right) {
  // TODO: map to PWM/driver
  Serial.printf("CMD L=%.3f R=%.3f\n", left, right);
}
void setTurret(float turret) {
  // TODO: map to turret motor
  Serial.printf("TUR=%.3f\n", turret);
}

void setup() {
  Serial.begin(115200);   // USB CDC logs
  Serial0.begin(460800);  // UART to Pi/PC
  Serial.println("ESP32 bidirectional UART ready.");
}

void loop() {
  // ---- RX: parse commands (robust to CMD or CMD3) ----
  static uint8_t buf[64];      // big enough for CMD3 (48 bytes) + slack
  static size_t  have = 0;

  // accumulate
  while (Serial0.available() && have < sizeof(buf)) {
    buf[have++] = (uint8_t)Serial0.read();
  }

  // try to parse while we have at least a header
  while (have >= 6) { // magic(2)+ver(2)+type(2)
    uint16_t magic = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    if (magic != MAGIC) {
      // resync: drop 1 byte
      memmove(buf, buf+1, --have);
      continue;
    }
    uint16_t ver  = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    uint16_t type = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

    size_t need = 0;
    if (ver != VER) {
      memmove(buf, buf+1, --have);
      continue;
    }
    if (type == TYPE_CMD)      need = CMD_SIZE;
    else if (type == TYPE_CMD3)need = CMD3_SIZE;
    else {
      // unknown type; drop 1 byte
      memmove(buf, buf+1, --have);
      continue;
    }

    if (have < need) break; // need more bytes

    // we have a full frame
    if (type == TYPE_CMD && need == CMD_SIZE) {
      CmdPacket cmd;
      memcpy(&cmd, buf, CMD_SIZE);
      uint16_t calc = crc16_surrogate((uint8_t*)&cmd, CMD_SIZE - 2);
      if (calc == cmd.crc16) {
        setMotorCommands(cmd.left, cmd.right);
      }
      else {
        Serial.println("CRC fail (CMD)"); // optional
      }
      memmove(buf, buf + CMD_SIZE, have - CMD_SIZE);
      have -= CMD_SIZE;
    } else if (type == TYPE_CMD3 && need == CMD3_SIZE) {
      Cmd3Packet cmd3;
      memcpy(&cmd3, buf, CMD3_SIZE);
      uint16_t calc = crc16_surrogate((uint8_t*)&cmd3, CMD3_SIZE - 2);
      if (calc == cmd3.crc16) {
        setMotorCommands(cmd3.left, cmd3.right);
        setTurret(cmd3.turret);
      }
      else {
        Serial.println("CRC fail (CMD3)");
      }
      memmove(buf, buf + CMD3_SIZE, have - CMD3_SIZE);
      have -= CMD3_SIZE;
    } else {
      // shouldn’t happen; resync
      memmove(buf, buf+1, --have);
    }
  }

  // ---- TX: send encoder ticks @ 100 Hz ----
  static uint32_t last_tx_ms = 0;
  if (millis() - last_tx_ms >= 10) {
    last_tx_ms = millis();

    EncPacket enc;
    enc.magic   = MAGIC;
    enc.ver     = VER;
    enc.type    = TYPE_ENC;
    static uint32_t enc_seq = 0;
    enc.seq     = ++enc_seq;
    enc.t_tx_ns = (uint64_t)micros() * 1000ull;

    noInterrupts();
    enc.ticksL = ticksL;
    enc.ticksR = ticksR;
    interrupts();

    enc.crc16 = crc16_surrogate((uint8_t*)&enc, ENC_SIZE - 2);

    Serial0.write((uint8_t*)&enc, ENC_SIZE);
    // Optional log:
    // Serial.printf("ENC seq=%u L=%ld R=%ld\n", enc.seq, (long)enc.ticksL, (long)enc.ticksR);
  }
}
