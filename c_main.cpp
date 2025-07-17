// #include <Arduino.h>

// // === Pin assignment =========================================================
// constexpr uint8_t PIN_MOTOR_FWD = 5;     // H‑bridge IN₁
// constexpr uint8_t PIN_MOTOR_BWD = 6;     // H‑bridge IN₂
// constexpr uint8_t PIN_PWM       = 1;    // H‑bridge EN / PWM  

// // === PWM setup ==============================================================
// constexpr uint8_t  LEDC_CH      = 0;     // LEDC channel 0..7 on S2
// constexpr uint32_t PWM_FREQ_HZ  = 30;    // mechanical, not audible
// constexpr uint8_t  PWM_RES_BITS = 11;    // 0‑2047 counts

// // === Wi‑Fi AP credentials ===================================================
// constexpr char     WIFI_SSID[]  = "ESP32_Cedric_AP";
// constexpr char     WIFI_PASS[]  = "easy";
// // constexpr IPAddress WIFI_IP     (192,168,1,134);   // static AP IP

// IPAddress WIFI_IP(192, 168, 1, 134); // IP address for Cedric Hollande

// // === HTML server ============================================================
// #include <WiFi.h>
// #include <body.h>
// #include <html510.h>
// HTML510Server web(80);

// // Current state --------------------------------------------------------------
// int duty_percent   = 50;            // 0‑100 %
// int direction_cmd  = 0;             // -1, 0, +1

// // ---------- HTTP handlers ---------------------------------------------------
// void pageRoot()              { web.sendhtml(body); }

// void onDirection()           // slider −1/0/+1
// {
//     direction_cmd = web.getVal();
//     digitalWrite(PIN_MOTOR_FWD, direction_cmd ==  1);
//     digitalWrite(PIN_MOTOR_BWD, direction_cmd == -1);
// }

// void onDuty()                // slider 0‑100 %
// {
//     duty_percent = web.getVal();
//     int pwm_counts = map(duty_percent, 0, 100, 0, (1<<PWM_RES_BITS)-1);
//     ledcWrite(LEDC_CH, pwm_counts);
// }

// // ---------- Arduino life‑cycle ---------------------------------------------
// void setup()
// {
//     Serial.begin(115200);

//     // --- Wi‑Fi AP -----------------------------------------------------------
//     WiFi.mode(WIFI_MODE_AP);
//     WiFi.softAP(WIFI_SSID, WIFI_PASS);
//     WiFi.softAPConfig(WIFI_IP, IPAddress(192,168,1,2), IPAddress(255,255,255,0));
//     Serial.print("AP on "); Serial.println(WIFI_IP);

//     // --- Web server ---------------------------------------------------------
//     web.begin();
//     web.attachHandler("/",                          pageRoot);
//     web.attachHandler("/set?type=direction&value=", onDirection);
//     web.attachHandler("/set?type=duty&value=",      onDuty);

//     // --- Motor pins ---------------------------------------------------------
//     pinMode(PIN_MOTOR_FWD, OUTPUT);
//     pinMode(PIN_MOTOR_BWD, OUTPUT);
//     pinMode(PIN_PWM,       OUTPUT);

//     // --- LEDC PWM -----------------------------------------------------------
//     ledcSetup   (LEDC_CH, PWM_FREQ_HZ, PWM_RES_BITS);
//     ledcAttachPin(PIN_PWM, LEDC_CH);
//     onDuty();                                     // apply default 50 %
// }

// void loop()
// {
//     web.serve();          // non‑blocking one‑shot
//     delay(10);            // light cooperative yield
// }
