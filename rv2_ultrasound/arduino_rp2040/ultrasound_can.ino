/**
 * The ultrasound module AJ-SR04M can measure the distance range from 0.2m to 8m.
 * A single module spend about 46.2 ms to complete the measurement (trigger + measure).
 * If using sequential trigger to trigger 3 modules, the time cost of one cycle will be about 138.6 ms.
 * If using parallel trigger, the time cost will be slightly longer then 46.2 ms, but less accurate.
 * The ultrasound3.ino file sent data via CAN.
 */

extern "C"
{
#include <hardware/watchdog.h>
};

// SPI to CAN
#include "DFRobot_MCP2515.h"

const int SPI_CS_PIN = 10;

DFRobot_MCP2515 CAN(SPI_CS_PIN);


#define trigPin0 5 // (RX) Pin to send trigger pulse
#define echoPin0 6 // (TX) Pin to receive echo pulse
#define echoPin1 7 // (TX) Pin to receive echo pulse
#define echoPin2 8 // (TX) Pin to receive echo pulse
#define echoPin3 9 // (TX) Pin to receive echo pulse

unsigned long minMeasureDuration = 1154;// 0.2m (us)
unsigned long maxMeasureDuration = 46162;// 8m (us)

#define SEND_ARR_SIZE 8
// distH0, distL0, distH1, distL1, distH2, distL2, distH3, distL3 (mm)
byte sendMsg[] = { 0, 0, 0, 0, 0, 0, 0, 0 };// No chksum.

void PrintDist()
{
  Serial.print("d0: ");
  Serial.print((sendMsg[4] << 8) + sendMsg[5]);
  Serial.print(" d1: ");
  Serial.print((sendMsg[6] << 8) + sendMsg[7]);
  Serial.print(" d2: ");
  Serial.print((sendMsg[8] << 8) + sendMsg[9]);
  Serial.print(" d3: ");
  Serial.println((sendMsg[10] << 8) + sendMsg[11]);
}

void SendDist()
{
//  char str[SEND_ARR_SIZE + 1];
//  memset(str, '\0', SEND_ARR_SIZE + 1);
//  for (int i = 0; i < SEND_ARR_SIZE; i++)
//    str[i] = sendMsg[i];
//  Serial.println(str);
//  Serial.write(sendMsg, SEND_ARR_SIZE);

  // send data:  id = 0x06, standrad flame, data len = 8, data: data buf
  CAN.sendMsgBuf(0x06, 0, SEND_ARR_SIZE, sendMsg);
}



unsigned long t0 = 0;
unsigned long t1 = 0;
unsigned long t2 = 0;
unsigned long t3 = 0;
unsigned int dist0 = 0;
unsigned int dist1 = 0;
unsigned int dist2 = 0;
unsigned int dist3 = 0;

void EchoChange0()
{
  if (digitalRead(echoPin0) == LOW)// High to low
    dist0 = (micros() - t0) * 346.6 / 2000.0;
  else// Low to high
    t0 = micros();
}

void EchoChange1()
{
  if (digitalRead(echoPin1) == LOW)// High to low
    dist1 = (micros() - t1) * 346.6 / 2000.0;
  else// Low to high
    t1 = micros();
}

void EchoChange2()
{
  if (digitalRead(echoPin2) == LOW)// High to low
    dist2 = (micros() - t2) * 346.6 / 2000.0;
  else// Low to high
    t2 = micros();
}

void EchoChange3()
{
  if (digitalRead(echoPin3) == LOW)// High to low
    dist3 = (micros() - t3) * 346.6 / 2000.0;
  else// Low to high
    t3 = micros();
}

void TrigProc()
{
  digitalWrite(trigPin0, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin0, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin0, LOW);
}

void FillMsg(unsigned int& dist, unsigned int hIdx)
{
  sendMsg[hIdx] = 0;
  sendMsg[hIdx + 1] = 0;
  if (dist >= 200 && dist <= 8000)
  {
    sendMsg[hIdx] = (dist >> 8) & 0xff;
    sendMsg[hIdx + 1] = dist & 0xff;
  }
}



void setup()
{
  watchdog_enable(5000, true);
  pinMode(trigPin0, OUTPUT);
  pinMode(echoPin0, INPUT_PULLUP);
  pinMode(echoPin1, INPUT_PULLUP);
  pinMode(echoPin2, INPUT_PULLUP);
  pinMode(echoPin3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(echoPin0), EchoChange0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin1), EchoChange1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin2), EchoChange2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin3), EchoChange3, CHANGE);

  Serial.begin(115200);

  while( CAN.begin(CAN_500KBPS) )
  {
      Serial.println("DFROBOT's CAN BUS Shield init fail");
      Serial.println("Please Init CAN BUS Shield again");
      delay(3000);
  }
  Serial.println("DFROBOT's CAN BUS Shield init ok!\n");
}

void loop()
{
  TrigProc();
  delayMicroseconds(100000);

  // Read Serial
  if (Serial.available() > 0)
  {
    byte buf[10];
    int len = Serial.readBytes(buf, 10);
    // 0x43, 0x4f, 0x43, 0x4f, 0, 0, 0x42, 0x49, 0x52, 0x44
    if (buf[0] == 0x43 && buf[1] == 0x4f && buf[2] == 0x43 && buf[3] == 0x4f && 
        buf[6] == 0x42 && buf[7] == 0x49 && buf[8] == 0x52 && buf[9] == 0x44)
    {
      int sig = (buf[4] << 8) + buf[5];
      if (sig == 18247)// Reset
        watchdog_reboot(0, 0, 0);
    }

    // Flush the buffer
    while (Serial.available() > 0)
      Serial.read();
  }

//  Serial.print("d0: ");
//  Serial.print(dist0);
//  Serial.print(" d1: ");
//  Serial.print(dist1);
//  Serial.print(" d2: ");
//  Serial.print(dist2);
//  Serial.print(" d3: ");
//  Serial.println(dist3);

  FillMsg(dist0, 0);
  FillMsg(dist1, 2);
  FillMsg(dist2, 4);
  FillMsg(dist3, 6);
  SendDist();
  watchdog_update();
}