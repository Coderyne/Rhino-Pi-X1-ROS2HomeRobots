#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("58:d0:05:0e:85:8d");

String xbox_string()
{
  String str = String(xboxController.xboxNotif.btnY) + "," +
               String(xboxController.xboxNotif.btnX) + "," +
               String(xboxController.xboxNotif.btnB) + "," +
               String(xboxController.xboxNotif.btnA) + "," +
               String(xboxController.xboxNotif.btnLB) + "," +
               String(xboxController.xboxNotif.btnRB) + "," +
               String(xboxController.xboxNotif.btnSelect) + "," +
               String(xboxController.xboxNotif.btnStart) + "," +
               String(xboxController.xboxNotif.btnXbox) + "," +
               String(xboxController.xboxNotif.btnShare) + "," +
               String(xboxController.xboxNotif.btnLS) + "," +
               String(xboxController.xboxNotif.btnRS) + "," +
               String(xboxController.xboxNotif.btnDirUp) + "," +
               String(xboxController.xboxNotif.btnDirRight) + "," +
               String(xboxController.xboxNotif.btnDirDown) + "," +
               String(xboxController.xboxNotif.btnDirLeft) + "," +
               String(xboxController.xboxNotif.joyLHori) + "," +
               String(xboxController.xboxNotif.joyLVert) + "," +
               String(xboxController.xboxNotif.joyRHori) + "," +
               String(xboxController.xboxNotif.joyRVert) + "," +
               String(xboxController.xboxNotif.trigLT) + "," +
               String(xboxController.xboxNotif.trigRT) + "\n";
  return str;
};

void xbox_string_format(uint8_t *data)
{
  bool btnA = xboxController.xboxNotif.btnA;
  bool btnB = xboxController.xboxNotif.btnB;
  bool btnX = xboxController.xboxNotif.btnX;
  bool btnY = xboxController.xboxNotif.btnY;
  bool btnLB = xboxController.xboxNotif.btnLB;
  bool btnRB = xboxController.xboxNotif.btnRB;
  bool btnXbox = xboxController.xboxNotif.btnXbox;

  // 获取摇杆和扳机的值，中心为0
  // -100 - 100范围内 向上为正 向右为正
  int joyL_X = (int)(((float)xboxController.xboxNotif.joyLHori - 32768.0) / 32768.0 * 100.0);
  int joyL_Y = -(int)(((float)xboxController.xboxNotif.joyLVert - 32768.0) / 32768.0 * 100.0);
  int joyR_X = (int)(((float)xboxController.xboxNotif.joyRHori - 32768.0) / 32768.0 * 100.0);
  int joyR_Y = -(int)(((float)xboxController.xboxNotif.joyRVert - 32768.0) / 32768.0 * 100.0);
  int trig_L = (int)(((float)xboxController.xboxNotif.trigLT) / 1023.0 * 100.0);
  int trig_R = (int)(((float)xboxController.xboxNotif.trigRT) / 1023.0 * 100.0);

  //包头
  uint8_t header[2] = {0x07, 0x21};

  data[0] = header[0];
  data[1] = header[1];
  data[2] = (uint8_t)(joyL_X) + 128;
  data[3] = (uint8_t)(joyL_Y) + 128;
  data[4] = (uint8_t)(joyR_X) + 128;
  data[5] = (uint8_t)(joyR_Y) + 128;
  data[6] = (uint8_t)(trig_L);
  data[7] = (uint8_t)(trig_R);
  data[8] = (btnA << 0) | (btnLB << 1) | (btnRB << 2) | (btnXbox << 3);
  data[9] = (btnB << 0) | (btnX << 1) | (btnY << 2);
  // 校验和：对前10个字节求和后取低8位
  uint16_t checksum = 0;
  for (int i = 0; i < 10; ++i)
  {
    checksum += data[i];
  }
  data[10] = (uint8_t)(checksum & 0xFF);
}

void uart_feedback()
{
  uint8_t data[11];
  xbox_string_format(data);
  Serial1.write(data, sizeof(data));
}

// Serial1 接收帧解析：检测包头 0x07 0x21 时触发振动
static uint8_t rxState = 0;
static unsigned long vibrationTriggeredAt = 0;
static const unsigned long vibrationCooldownMs = 300;

void triggerVibration()
{
  unsigned long now = millis();
  if (now - vibrationTriggeredAt < vibrationCooldownMs) return;
  vibrationTriggeredAt = now;

  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  repo.setAllOff();
  repo.v.select.shake = true;
  repo.v.select.center = true;
  repo.v.power.shake = 50;
  repo.v.power.center = 50;
  repo.v.timeActive = 25;   // 250ms
  repo.v.timeSilent = 0;
  repo.v.countRepeat = 0;
  xboxController.writeHIDReport(repo);
}

void serial1_onReceive()
{
  while (Serial1.available()) {
    uint8_t byte = Serial1.read();
    if (rxState == 0 && byte == 0x07) {
      rxState = 1;
    } else if (rxState == 1 && byte == 0x21) {
      rxState = 0;
      triggerVibration();
    } else {
      rxState = 0;
    }
  }
}

/*
支持四种振动模式
left：上左电机动
right：上右电机动
center：下左电机和下右电机一起动，频率高力量小
shake：下左电机和下右电机一起动，频率低力量大

测试结果：
四种模式都可以调振动力度
下左电机和下右电机是绑定的，只能一起动，但是提供了两种振动模式，个人猜测是两种模式的原理是给电机不同的电压
可以随意搭配使用，但center和shake一起用的话执行的应该是shake
*/

// 配置参考
// repo.v.select.center = 0;
// repo.v.select.left = 0;
// repo.v.select.right = 0;
// repo.v.select.shake = 0;
// repo.v.power.center = 0; // x% power
// repo.v.power.left = 0;
// repo.v.power.right = 30;
// repo.v.power.shake = 0;
// repo.v.timeActive = 0; // 振动 x/100 秒，最大2.56秒(uint8_t)
// repo.v.timeSilent = 0;   // 静止 x/100 秒
// repo.v.countRepeat = 0;  // 循环次数 x+1

// 官方例程
void demoVibration()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  Serial.println("full power for 1 sec");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.center = true;
  repo.v.select.left = false;
  repo.v.select.right = false;
  repo.v.select.shake = false;
  repo.v.power.center = 30; // 30% power
  Serial.println("run center 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.center = false;
  repo.v.select.left = true;
  repo.v.power.left = 30;
  Serial.println("run left 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.left = false;
  repo.v.select.right = true;
  repo.v.power.right = 30;
  Serial.println("run right 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.right = false;
  repo.v.select.shake = true;
  repo.v.power.shake = 30;
  Serial.println("run shake 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.shake = false;
  repo.v.select.center = true;
  repo.v.power.center = 50;
  repo.v.timeActive = 20;
  repo.v.timeSilent = 20;
  repo.v.countRepeat = 2;
  Serial.println("run center 50\% power in 0.2 sec 3 times");
  xboxController.writeHIDReport(repo);
  delay(2000);
}

// 振动反馈，根据扳机按压力度调整振动力度
void demoVibration_2()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  static uint16_t TrigMax = XboxControllerNotificationParser::maxTrig;
  String str_1;
  repo.setAllOff();
  repo.v.select.left = true;
  repo.v.select.right = true;
  repo.v.power.left = (uint8_t)((float)xboxController.xboxNotif.trigLT / (float)TrigMax * 100);
  repo.v.power.right = (uint8_t)((float)xboxController.xboxNotif.trigRT / (float)TrigMax * 100);
  repo.v.timeActive = 50;
  xboxController.writeHIDReport(repo);
  str_1 = String(repo.v.power.left) + "," + String(repo.v.power.right) + "\n";

  Serial.print(str_1);
  delay(50);
}

//检测是否有按键被按下，或者摇杆被移动，或扳机被按下
bool isPress()
{
  if (xboxController.xboxNotif.btnY || xboxController.xboxNotif.btnX || xboxController.xboxNotif.btnB || xboxController.xboxNotif.btnA ||
      xboxController.xboxNotif.btnLB || xboxController.xboxNotif.btnRB || xboxController.xboxNotif.btnSelect || xboxController.xboxNotif.btnStart ||
      xboxController.xboxNotif.btnXbox || xboxController.xboxNotif.btnShare || xboxController.xboxNotif.btnLS || xboxController.xboxNotif.btnRS ||
      xboxController.xboxNotif.btnDirUp || xboxController.xboxNotif.btnDirRight || xboxController.xboxNotif.btnDirDown || xboxController.xboxNotif.btnDirLeft ||
      xboxController.xboxNotif.joyLHori != 32768 || xboxController.xboxNotif.joyLVert != 32767 ||
      xboxController.xboxNotif.joyRHori != 32768 || xboxController.xboxNotif.joyRVert != 32767 ||
      xboxController.xboxNotif.trigLT != 0 || xboxController.xboxNotif.trigRT != 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void setup()
{
  //Serial.begin(115200); //DEBUG
  Serial1.begin(115200, SERIAL_8N1, 20, 21); // RX, TX
  Serial.println("Starting NimBLE Client");
  xboxController.begin();
}

void loop()
{
  delay(10);
  xboxController.onLoop();
  if (xboxController.isConnected())
  {
    if (xboxController.isWaitingForFirstNotification())
    {
      Serial.println("waiting for first notification");
    }
    else
    {
      uart_feedback();
      serial1_onReceive();
      if (isPress())
      {
        // uart_feedback();
      }
      
      // demoVibration();
      // demoVibration_2();
    }
  }
  else
  {
    Serial.println("not connected");
    if (xboxController.getCountFailedConnection() > 2)
    {
      ESP.restart();
    }
  }
}
