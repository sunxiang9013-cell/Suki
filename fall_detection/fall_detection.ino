/*
 * ESP32 + MPU6050 拐杖摔倒监测(含WiFi配网 + 微信报警)
 * ------------------------------------------------
 * 工作流程：
 *   1. 开机校准10秒(呼吸灯)，期间保持拐杖静止竖直，记录Roll/Pitch基准角度
 *   2. 校准完成后联网：有保存凭证→直连；无凭证/连不上→AP配网(1分钟超时)
 *   3. 监测中：任一轴角度偏差>60° → 开始5秒倒计时(LED快闪)
 *   4. 5秒内角度恢复 → 重置计时
 *   5. 5秒内未恢复 → 判定拐杖倒了，LED常亮 + 微信报警(巴法云)
 *   6. 拐杖被扶起(角度恢复持续1秒) → 解除报警，回到监测
 *   7. WiFi离线时自动重连(无需复位)
 *
 * 报警策略：判定倒了立即发一次；发送失败/没网则联网后自动补发；
 *           发送成功即停止；扶起后重置(下次再倒再发)。
 *
 * 角度算法：互补滤波(加速度计+陀螺仪融合)，Roll/Pitch不漂移
 * 依赖库：Wire, WiFi, WebServer, Preferences, HTTPClient(均ESP32内置)
 */

#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <HTTPClient.h>

// ============ 引脚与硬件配置 ============
#define MPU6050_ADDR   0x68      // MPU6050 I2C地址(AD0接GND时0x68，接VCC时0x69)
#define SDA_PIN        19        // ESP32 I2C数据引脚
#define SCL_PIN        22        // ESP32 I2C时钟引脚
#define LED_PIN        4         // 状态指示LED(板载LED，可改)

// ============ 摔倒监测参数(可调) ============
#define CALIB_TIME_MS       10000  // 开机校准时间(ms)
#define FALL_CONFIRM_MS     5000   // 倒下确认时间(ms)：偏差持续超阈值多久判定为倒了
#define RECOVER_CONFIRM_MS  1000   // 报警解除确认时间(ms)：角度恢复正常持续多久解除报警
#define ANGLE_THRESHOLD     60.0   // 角度偏差阈值(度)：任一轴偏差超过此值视为异常
#define SAMPLE_PERIOD_MS    10     // 采样周期(ms)，100Hz
#define COMP_ALPHA          0.98   // 互补滤波系数(越大越信任陀螺仪，越小越信任加速度计)

// ============ LED指示参数(可调) ============
#define LED_PWM_FREQ         5000  // LED PWM频率(Hz)
#define LED_PWM_BITS         8     // PWM分辨率(8位=0~255)
#define BREATH_PERIOD_MS     1500  // 校准呼吸灯完整周期(ms，从暗→亮→暗)，值越小呼吸越快
#define FAST_BLINK_PERIOD_MS 300   // 超阈值快闪完整周期(ms，亮+灭各半)，值越小闪得越快

// ============ WiFi配网参数(可调) ============
#define AP_SSID                   "ESP32_Cane_Config"  // 配网AP名称
#define CONFIG_PORTAL_TIMEOUT_MS  60000   // 配网等待超时(ms)，1分钟内无人访问则退出
#define WIFI_CONNECT_TIMEOUT_MS   15000   // 连接WiFi超时(ms)
#define RECONNECT_INTERVAL_MS     30000   // 主程序中WiFi离线后自动重连间隔(ms)

// ============ 微信报警参数(巴法云) ============
#define BEMFA_UID                 "8dbc02daeecaddd942aa25f963629d9a"  // 巴法云私钥uid
#define BEMFA_DEVICE              "防摔设备"                          // 设备名
#define BEMFA_MSG                 "请注意，老人摔倒了！"              // 报警消息
#define ALARM_RETRY_INTERVAL_MS   60000   // 报警发送失败后重试间隔(ms)

// ============ 运行状态 ============
enum SysState {
  STATE_CALIBRATING,  // 校准中
  STATE_MONITORING,   // 正常监测
  STATE_FALLEN        // 已判定倒下，报警中
};

// ============ 全局对象 ============
Preferences prefs;
WebServer server(80);
WiFiClient client_bemfa;
HTTPClient http_bemfa;

// ============ 全局变量：摔倒监测 ============
float baseRoll = 0, basePitch = 0;      // 校准记录的基准角度
float roll = 0, pitch = 0;              // 当前实时角度
unsigned long lastSampleMs = 0;         // 上次采样时间戳
unsigned long fallTimerStartMs = 0;     // 倒下计时起点
unsigned long recoverTimerStartMs = 0;  // 报警解除计时起点
bool inFallTimer = false;               // 是否正在倒下计时中
SysState state = STATE_CALIBRATING;

// ============ 全局变量：WiFi配网 ============
bool configStarted = false;        // 是否有用户访问了配网页面(访问后取消超时)
unsigned long apStartMs = 0;       // AP开启时间戳
String savedSSID = "";             // 从NVS读取的已保存WiFi名(供主程序重连用)
String savedPass = "";             // 从NVS读取的已保存WiFi密码

// ============ 全局变量：微信报警 ============
bool alarmPending = false;             // 待发送报警标志(进入FALLEN置true，发送成功置false)
unsigned long lastAlarmSendMs = 0;     // 上次报警发送时间戳

// 配网页面HTML头部
const char CONFIG_PAGE_HEAD[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>拐杖配网</title>
<style>
body{font-family:Arial;max-width:420px;margin:20px auto;padding:15px;color:#333;}
h2{margin-top:0;}
input,button{width:100%;padding:11px;margin:6px 0;box-sizing:border-box;font-size:15px;border:1px solid #ccc;border-radius:5px;}
button{background:#1976D2;color:#fff;border:none;}
.item{padding:9px;border-bottom:1px solid #eee;cursor:pointer;}
.item:hover{background:#f0f6ff;}
.note{color:#888;font-size:13px;}
</style>
</head>
<body>
<h2>拐杖 WiFi 配网</h2>
<div class="note">点击WiFi名自动填入，输入密码后提交</div>
<form action="/save" method="post">
<input name="ssid" placeholder="WiFi名称">
<input name="pass" type="password" placeholder="WiFi密码">
<button type="submit">保存并连接</button>
</form>
<hr>
<h3>附近WiFi</h3>
)=====";

// ============ 初始化MPU6050 ============
void initMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz快速模式

  // 唤醒(退出睡眠模式)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);

  // 加速度计量程±2g(灵敏度16384 LSB/g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); Wire.write(0x00);
  Wire.endTransmission(true);

  // 陀螺仪量程±250°/s(灵敏度131 LSB/(°/s))
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
  Wire.endTransmission(true);

  // 低通滤波器(约43Hz，减小噪声)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); Wire.write(0x03);
  Wire.endTransmission(true);

  delay(100);  // 等寄存器稳定
}

// ============ 读取MPU6050原始数据(6轴) ============
void readMPU6050(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // 从ACCEL_XOUT_H开始读
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);  // 连读14字节

  ax = Wire.read() << 8 | Wire.read();  // 加速度X
  ay = Wire.read() << 8 | Wire.read();  // 加速度Y
  az = Wire.read() << 8 | Wire.read();  // 加速度Z
  Wire.read(); Wire.read();             // 温度(跳过)
  gx = Wire.read() << 8 | Wire.read();  // 陀螺仪X
  gy = Wire.read() << 8 | Wire.read();  // 陀螺仪Y
  gz = Wire.read() << 8 | Wire.read();  // 陀螺仪Z
}

// ============ 互补滤波更新Roll/Pitch ============
// 原理：陀螺仪短期准但长期漂移，加速度计长期准但短期受运动干扰
//       两者按比例融合，取长补短，得到稳定不漂移的角度
void updateAngles(float dt, int16_t ax, int16_t ay, int16_t az,
                  int16_t gx, int16_t gy) {
  // --- 加速度计计算角度(以重力方向为绝对参考) ---
  float accRoll  = atan2((float)ay, (float)az) * 180.0 / PI;
  float accPitch = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI;

  // --- 陀螺仪角速度转换为°/s ---
  // 量程±250°/s对应灵敏度131 LSB/(°/s)
  float gyroRollRate  = gx / 131.0;  // Roll角速度(绕X轴)
  float gyroPitchRate = gy / 131.0;  // Pitch角速度(绕Y轴)

  // --- 互补滤波融合 ---
  // 陀螺仪积分预测 + 加速度计修正
  roll  = COMP_ALPHA * (roll  + gyroRollRate  * dt) + (1.0 - COMP_ALPHA) * accRoll;
  pitch = COMP_ALPHA * (pitch + gyroPitchRate * dt) + (1.0 - COMP_ALPHA) * accPitch;
}

// ============ 校准：记录基准角度 ============
void doCalibration() {
  Serial.println("[校准] 请保持拐杖静止竖直，10秒后完成...");
  unsigned long startMs = millis();
  float sumRoll = 0, sumPitch = 0;
  int count = 0;

  // 先读一次给角度一个初值，避免首帧异常
  int16_t ax, ay, az, gx, gy, gz;
  readMPU6050(ax, ay, az, gx, gy, gz);
  updateAngles(0.01, ax, ay, az, gx, gy);

  // 持续采样取平均值作为基准
  while (millis() - startMs < CALIB_TIME_MS) {
    unsigned long now = millis();
    if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
      float dt = (now - lastSampleMs) / 1000.0;
      lastSampleMs = now;
      readMPU6050(ax, ay, az, gx, gy, gz);
      updateAngles(dt, ax, ay, az, gx, gy);
      sumRoll  += roll;
      sumPitch += pitch;
      count++;
    }
    // LED呼吸灯提示校准中(从暗→亮→暗，正弦曲线)
    float phase = 2.0 * PI * (millis() % BREATH_PERIOD_MS) / (float)BREATH_PERIOD_MS;
    int duty = (1.0 - cos(phase)) * 127.5;  // 0~255
    ledcWrite(LED_PIN, duty);
  }

  baseRoll  = sumRoll / count;
  basePitch = sumPitch / count;
  ledcWrite(LED_PIN, 0);  // 校准完成熄灯

  Serial.printf("[校准] 完成  基准Roll=%.1f°  Pitch=%.1f°\n", baseRoll, basePitch);
}

// ============ 判断当前角度是否超出阈值 ============
// 决策：任一轴偏差>阈值即判异常(非两轴都超)
// 原因：拐杖倒下方向不确定，倒向不同方向时Roll/Pitch变化量不同，
//       要求两轴都超阈值会漏报某些方向的倒下；配合5秒计时防误报。
bool isAngleOutOfRange() {
  float dRoll  = fabs(roll  - baseRoll);
  float dPitch = fabs(pitch - basePitch);
  return (dRoll > ANGLE_THRESHOLD) || (dPitch > ANGLE_THRESHOLD);
}

// ============ 尝试连接WiFi ============
bool tryConnect(const String &ssid, const String &pass) {
  Serial.printf("[WiFi] 连接 \"%s\" ...\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start >= WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("[WiFi] 连接超时");
      WiFi.disconnect(true);
      return false;
    }
    delay(300);
  }
  Serial.printf("[WiFi] 已连接  IP=%s\n", WiFi.localIP().toString().c_str());
  return true;
}

// ============ 配网页面 ============
void handleRoot() {
  configStarted = true;  // 标记用户已进入配网，取消超时

  // 扫描周围WiFi
  int n = WiFi.scanNetworks();
  String list = "";
  if (n == 0) {
    list = "<div class='note'>未发现WiFi，请确认路由器开启</div>";
  } else {
    for (int i = 0; i < n; i++) {
      String s = WiFi.SSID(i);
      int rssi = WiFi.RSSI(i);
      // 转义SSID中的单引号，避免破坏onclick
      String esc = s;
      esc.replace("'", "\\'");
      list += "<div class='item' onclick=\"document.forms[0].ssid.value='" + esc + "'\">";
      list += s + " <span class='note'>(" + String(rssi) + "dBm)</span></div>";
    }
  }
  server.send(200, "text/html", String(CONFIG_PAGE_HEAD) + list + "</body></html>");
}

// ============ 保存配网信息 ============
void handleSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  if (ssid.length() == 0) {
    server.send(400, "text/html",
      "<meta charset='utf-8'>SSID不能为空，<a href='/'>返回</a>");
    return;
  }
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);

  String html = "<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<h3>保存成功</h3><p>WiFi: " + ssid + "</p><p>设备正在重启并连接...</p>";
  server.send(200, "text/html", html);

  delay(1500);  // 等客户端接收响应
  ESP.restart();
}

// ============ 启动配网模式 ============
void startConfigPortal() {
  Serial.println("[配网] 启动AP配网模式");
  WiFi.mode(WIFI_AP_STA);  // AP供手机连接，STA用于扫描周围WiFi列表
  WiFi.softAP(AP_SSID);
  delay(100);

  Serial.printf("[配网] 请用手机连接WiFi \"%s\"\n", AP_SSID);
  Serial.printf("[配网] 然后浏览器访问 %s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("[配网] 等待配网(1分钟内无人访问则超时退出)...");

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302);
  });
  server.begin();

  apStartMs = millis();
  configStarted = false;

  // 配网主循环
  while (true) {
    server.handleClient();

    // 仅在用户未进入配网时计时；一旦访问页面则取消超时，持续等待提交
    if (!configStarted && millis() - apStartMs >= CONFIG_PORTAL_TIMEOUT_MS) {
      Serial.println("[配网] 1分钟内无人进入配网，超时退出");
      server.stop();
      WiFi.softAPdisconnect(true);
      return;
    }
    delay(2);
  }
}

// ============ 发送微信报警(巴法云) ============
// 返回true表示发送成功(会清除alarmPending)；false表示失败(保留alarmPending待重试)
bool sendWechatAlarm() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[报警] WiFi未连接，无法发送微信通知");
    return false;
  }
  String url = "http://api.bemfa.com/api/wechat/v1/weget.php?type=2&uid="
               BEMFA_UID "&device=" BEMFA_DEVICE "&msg=" BEMFA_MSG;
  http_bemfa.begin(client_bemfa, url);
  int code = http_bemfa.GET();
  if (code > 0) {
    Serial.printf("[报警] 微信通知已发送(code=%d)\n", code);
    http_bemfa.end();
    return true;
  } else {
    Serial.printf("[报警] 微信通知发送失败(code=%d)\n", code);
    http_bemfa.end();
    return false;
  }
}

// ============ setup ============
void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(0, OUTPUT);
  digitalWrite(0,LOW);

  ledcAttach(LED_PIN, LED_PWM_FREQ, LED_PWM_BITS);  // LED PWM初始化(呼吸灯用)
  ledcWrite(LED_PIN, 0);  // 初始熄灭

  initMPU6050();
  Serial.println("[系统] ESP32拐杖摔倒监测启动");

  // ===== 1. 先校准陀螺仪 =====
  lastSampleMs = millis();
  doCalibration();
ledcWrite(LED_PIN, 0);  // 初始熄灭
  // ===== 2. 校准完成后联网 =====
  Serial.println("[系统] 开始联网...");
  prefs.begin("wifi_creds", false);  // NVS命名空间，可读写
  savedSSID = prefs.getString("ssid", "");
  savedPass = prefs.getString("pass", "");

  bool connected = false;
  if (savedSSID.length() > 0) {
    // 有保存凭证，尝试连接
    if (tryConnect(savedSSID, savedPass)) {
      connected = true;
    } else {
      Serial.println("[WiFi] 原凭证连接失败，进入配网");
    }
  } else {
    Serial.println("[WiFi] 无保存凭证，进入配网");
  }
  if (!connected) {
    startConfigPortal();
    // 到这里说明是超时退出(配网成功会restart)
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setAutoReconnect(true);  // 开启断线自动重连
    Serial.println("[系统] 联网完成(已联网)");
  } else {
    Serial.println("[系统] 进入主程序(未联网，将自动重连)");
  }

  // ===== 3. 进入监测 =====
  state = STATE_MONITORING;
  lastSampleMs = millis();
  Serial.println("[监测] 开始监测");
}

// ============ loop ============
void loop() {
  unsigned long now = millis();

  // ===== 1. 摔倒监测采样(100Hz节流) =====
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    float dt = (now - lastSampleMs) / 1000.0;
    lastSampleMs = now;

    // 采样并更新角度
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050(ax, ay, az, gx, gy, gz);
    updateAngles(dt, ax, ay, az, gx, gy);

    if (state == STATE_MONITORING) {
      if (isAngleOutOfRange()) {
        if (!inFallTimer) {
          // 首次超阈值，启动倒下计时
          inFallTimer = true;
          fallTimerStartMs = now;
          Serial.printf("[监测] 角度超阈值 Roll=%.1f° Pitch=%.1f° 开始计时\n", roll, pitch);
        } else if (now - fallTimerStartMs >= FALL_CONFIRM_MS) {
          // 持续超阈值满5秒，判定倒下
          state = STATE_FALLEN;
          inFallTimer = false;
          alarmPending = true;       // 标记需要发送报警
          lastAlarmSendMs = 0;       // 立即允许首次发送
          Serial.println("[报警] 拐杖倒了！");
        }
      } else {
        // 角度恢复正常，重置倒下计时
        if (inFallTimer) {
          inFallTimer = false;
          Serial.println("[监测] 角度恢复，计时重置");
        }
      }

      // LED指示：超阈值计时中快闪，否则熄灭
      if (inFallTimer) {
        ledcWrite(LED_PIN, ((now / (FAST_BLINK_PERIOD_MS / 2)) % 2) ? 255 : 0);
      } else {
        ledcWrite(LED_PIN, 0);
      }
    }
    else if (state == STATE_FALLEN) {
      // --- 报警中：LED常亮 ---
      ledcWrite(LED_PIN, 255);

      // 串口周期性重复报警(2秒一次)
      static unsigned long lastAlarmPrint = 0;
      if (now - lastAlarmPrint > 2000) {
        lastAlarmPrint = now;
        Serial.println("[报警] 拐杖倒了！");
      }

      // 角度恢复正常持续1秒 → 解除报警，回到监测
      if (!isAngleOutOfRange()) {
        if (recoverTimerStartMs == 0) {
          recoverTimerStartMs = now;
        } else if (now - recoverTimerStartMs >= RECOVER_CONFIRM_MS) {
          recoverTimerStartMs = 0;
          state = STATE_MONITORING;
          alarmPending = false;  // 解除报警，重置标志(下次再倒再发)
          Serial.println("[监测] 拐杖扶起，恢复监测");
        }
      } else {
        recoverTimerStartMs = 0;  // 又超阈值，重置恢复计时
      }
    }
  }

  // ===== 2. 微信报警发送(每次loop检查，发送有间隔限制) =====
  // alarmPending为true时尝试发送；联网则发，成功则清除标志，失败保留待重试
  if (alarmPending && WiFi.status() == WL_CONNECTED) {
    // 首次(lastAlarmSendMs==0)立即发；否则按重试间隔发送
    if (lastAlarmSendMs == 0 || (now - lastAlarmSendMs >= ALARM_RETRY_INTERVAL_MS)) {
      lastAlarmSendMs = now;
      if (sendWechatAlarm()) {
        alarmPending = false;  // 发送成功，停止重试
      }
    }
  }

  // ===== 3. WiFi离线自动重连(有保存凭证时，定期重连，无需复位) =====
  static unsigned long lastReconnect = 0;
  static bool autoReconnectSet = false;

  if (WiFi.status() == WL_CONNECTED) {
    // 连上后开启断线自动重连(只设一次)
    if (!autoReconnectSet) {
      autoReconnectSet = true;
      WiFi.setAutoReconnect(true);
    }
  } else if (savedSSID.length() > 0) {
    // 离线且有保存凭证，定期尝试重连
    if (now - lastReconnect >= RECONNECT_INTERVAL_MS) {
      lastReconnect = now;
      Serial.println("[WiFi] 离线，尝试自动重连...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(savedSSID.c_str(), savedPass.c_str());
    }
  }
}
