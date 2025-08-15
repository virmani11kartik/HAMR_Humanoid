#include <Arduino.h>

// ----------------- Protocol -----------------
static const uint16_t MAGIC = 0xCAFE;
static const uint16_t VER   = 1;
static const uint16_t TYPE_CMD = 0x01; // PC->ESP
static const uint16_t TYPE_ENC = 0x02; // ESP->PC

#pragma pack(push,1)
struct CmdPacket {
  uint16_t magic;
  uint16_t ver;
  uint16_t type;     // 0x01
  uint32_t seq;
  uint64_t t_tx_ns;
  float left;        // wheel cmd
  float right;       // wheel cmd
  uint16_t crc16;    // crc32 folded to 16
};

struct EncPacket {
  uint16_t magic;
  uint16_t ver;
  uint16_t type;     // 0x02
  uint32_t seq;
  uint64_t t_tx_ns;  // ESP send time
  int32_t ticksL;
  int32_t ticksR;
  uint16_t crc16;    // crc32 folded to 16
};
#pragma pack(pop)

static const size_t CMD_SIZE = sizeof(CmdPacket);
static const size_t ENC_SIZE = sizeof(EncPacket);

// CRC32->16 surrogate
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

// ---- Your encoder sources (replace with your ISR counters) ----
volatile int32_t ticksL = 0;
volatile int32_t ticksR = 0;
// Example: call these from your attachInterrupt handlers to update ticksL/ticksR

// ---- Motor command hook ----
void setMotorCommands(float left, float right) {
  // TODO: map to PWM/driver as needed
  // For now we just log
  Serial.printf("CMD L=%.3f R=%.3f\n", left, right);
}

void setup() {
  Serial.begin(115200);   // USB CDC logs
  Serial0.begin(460800);  // Data UART to PC

  Serial.println("ESP32 bidirectional UART ready.");
}

void loop() {
  // ---- RX: parse commands ----
  static uint8_t rbuf[CMD_SIZE];
  static size_t ridx = 0;

  while (Serial0.available()) {
    rbuf[ridx++] = (uint8_t)Serial0.read();
    if (ridx < CMD_SIZE) continue;

    CmdPacket cmd;
    memcpy(&cmd, rbuf, CMD_SIZE);
    ridx = 0;

    if (cmd.magic != MAGIC || cmd.ver != VER || cmd.type != TYPE_CMD) {
      // Bad header/type; try to resync by shifting 1 byte next loop
      // Simple resync: move window by one byte if the first two don't match MAGIC
      // (Keeping it minimal; at this baud and framing, errors are rare)
      continue;
    }
    uint16_t calc = crc16_surrogate((uint8_t*)&cmd, CMD_SIZE - 2);
    if (calc != cmd.crc16) {
      // CRC mismatch; drop
      continue;
    }

    // Valid command
    setMotorCommands(cmd.left, cmd.right);
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

    // read atomically
    noInterrupts();
    enc.ticksL = ticksL;
    enc.ticksR = ticksR;
    interrupts();

    enc.crc16 = crc16_surrogate((uint8_t*)&enc, ENC_SIZE - 2);

    // Write first (keep TX path fast), then log
    Serial0.write((uint8_t*)&enc, ENC_SIZE);

    // Optional: lightweight log (comment out if you want even lower jitter)
    // Serial.printf("ENC seq=%u L=%ld R=%ld\n", enc.seq, (long)enc.ticksL, (long)enc.ticksR);
  }
}
