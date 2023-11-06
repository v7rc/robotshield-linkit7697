// Company: V7 IDEA TECHNOLOGY LTD.
// Author: Louis Chuang
// Date: 2019-08-09
// Version: 1.00.00
// Subject: 給Linkit 7697與MiniPlan Robot Shield使用的程式碼，需要搭配V7RC使用;
// 原始碼使用說明： 請尊重智慧財產權，此為開源程式碼，但請尊重原創作者，如要使用於他處，請註明出處。

#include <LBLE.h>
#include <LBLEPeriphral.h>
#include <Servo.h>

#define SERVO_DEFAULT_VALUE 1500
#define SERVO_DEFAULT_MAX_VALUE 2000
#define SERVO_DEFAULT_MIN_VALUE 1000
#define PACKET_LENGTH 20
#define numOfServo 8

// Define a simple GATT service with only 1 characteristic
LBLEService v7rc_uart("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
LBLECharacteristicString uartInputCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", LBLE_WRITE);
LBLECharacteristicString bleNotifyer("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", LBLE_READ);

// Servo Pin,  目前先用8組;
const int SERVO_Pin[] = {2, 7, 3, 8, 4, 9, 5, 11};
Servo robotServo[numOfServo];

int count = 0;               //計數用

int servValue[numOfServo];            // 目前的伺服馬達數值
int oldServValue[numOfServo];            // 舊的的伺服馬達數值
int failSafeServValue[numOfServo];    // 各個伺服馬達FailSafe設定
int servoMAXValue[numOfServo];        // 每個Servo最大的數值
int servoMINValue[numOfServo];        // 每個Servo最小的數值
int bytes[PACKET_LENGTH];
int receiveServoValue[numOfServo];

int dcMotorPinA[] = {12, 10};     // DC motor A
int dcMotorPinB[] = {13, 17};     // DC motor B

long startProcessTime = 0;
long endProcessTime = 0;

// 測試Servo相關數據
boolean ifTestMode = false;   // 是否進入測試模式
boolean ifAddValue = false;   // 是否增加資料
int servoMoveStepValue = 80;  // 測試用的速度

#define LOST_SIGNAL_MAX_TIME 500 // 最大失去信號時間;

int currentLostSignalTime = 0;

int accelPWMValue = SERVO_DEFAULT_VALUE;   // 需要控制油門的的PWM;
int accelPWMChannel = 1;      // Channel0, 1, 2, 3, 4, 5, 6....
bool isControlAccelerator = false;          // 是否需要限制油門;

char thisBLEID[9];

void setup() {

  // put your setup code here, to run once:

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //Initialize serial and wait for port to open:
  Serial.begin(115200);

  // to check if USR button is pressed
  pinMode(6, INPUT);

  // Initialize BLE subsystem
  LBLE.begin();
  while (!LBLE.ready()) {
    delay(100);
  }

  Serial.println("BLE ready");
  Serial.print("Device Address = [");
  Serial.print(LBLE.getDeviceAddress());
  Serial.println("]");

  // char buf1 [100] = {'\0'};
  //sprintf (buf1, "%s", LBLE.getDeviceAddress().toString().c_str());
  String buf1 = LBLE.getDeviceAddress().toString();
  Serial.print("buf1:");
  Serial.println(buf1);

  // char thisBLEID[9] = {'V', '7', 'R', 'C', '-', buf1[0], buf1[1], buf1[3], buf1[5] };
  thisBLEID[0] = 'V';
  thisBLEID[1] = '7';
  thisBLEID[2] = 'R';
  thisBLEID[3] = 'C';
  thisBLEID[4] = '-';
  thisBLEID[5] =  buf1[0];
  thisBLEID[6] =  buf1[1];
  thisBLEID[7] =  buf1[3];
  thisBLEID[8] =  buf1[4];

  String bleName = thisBLEID;

  // bleName += String(buf1[0]);
  // bleName += String(buf1[1]);
  // bleName += String(buf1[3]);
  // bleName += String(buf1[4]);
  // bleName += String(buf1[6]);
  // bleName += String(buf1[7]);
  // bleName += String(buf1[9]);
  // bleName += String(buf1[10]);
  // bleName += String(buf1[12]);
  // bleName += String(buf1[13]);

  // configure our advertisement data.
  // In this case, we simply create an advertisement that represents an
  // connectable device with a device name

  LBLEUuid uuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

  LBLEAdvertisementData advertisement;

//  advertisement.configAsConnectableDevice(bleName.c_str(), uuid);
  advertisement.configAsConnectableDevice("V7RC", uuid); 

  // Configure our device's Generic Access Profile's device name
  // Ususally this is the same as the name in the advertisement data.
  LBLEPeripheral.setName(bleName.c_str());


  // Add characteristics into ledService
  v7rc_uart.addAttribute(uartInputCharacteristic);
  v7rc_uart.addAttribute(bleNotifyer);

  // Add service to GATT server (peripheral)
  LBLEPeripheral.addService(v7rc_uart);

  // start the GATT server - it is now
  // available to connect
  LBLEPeripheral.begin();

  // start advertisment
  LBLEPeripheral.advertise(advertisement);

  servoControlInit();

  // setup DC motor;
  pinMode(dcMotorPinA[0], OUTPUT);
  pinMode(dcMotorPinA[1], OUTPUT);
  pinMode(dcMotorPinB[0], OUTPUT);
  pinMode(dcMotorPinB[1], OUTPUT);

  processDCMotor(SERVO_DEFAULT_VALUE, dcMotorPinA);
   processDCMotor(SERVO_DEFAULT_VALUE, dcMotorPinB);

}

int dataIndex = 0;
byte dataBytes[PACKET_LENGTH];
String thisPacket;

void loop() {

  while (LBLEPeripheral.connected()) {

    if (uartInputCharacteristic.isWritten()) {

      thisPacket = uartInputCharacteristic.getValue();

      // Serial.println(thisPacket);
      // Serial.println(thisPacket.length());

        processRCString(thisPacket);

    } else {

      //Serial.println("is not write!");

    }

    delay(1);

  }

  // Serial.println("BLE斷線！");
  int thisAngle =  map(SERVO_DEFAULT_VALUE, 544, 2400, 0, 180);
  for(int i = 0; i < numOfServo; i ++) {
    // robotServo[i].write(thisAngle);
    robotServo[i].writeMicroseconds(SERVO_DEFAULT_VALUE);
  }

  delay(100);

}



//設定servo初始值
void servoControlInit()
{

  // 設定Servo, 並且設定成預設1500;

  for (int i = 0; i < sizeof(SERVO_Pin); i ++) {

    // servoMAXValue[i] = SERVO_DEFAULT_MAX_VALUE;
    // servoMINValue[i] = SERVO_DEFAULT_MIN_VALUE;
    // oldServValue[i] = SERVO_DEFAULT_VALUE;

    robotServo[i].attach(SERVO_Pin[i]);   // 設定
    robotServo[i].writeMicroseconds(SERVO_DEFAULT_VALUE);
    // int thisAngle = map(1500, 544, 2400, 0, 180);

    // Serial.println(thisAngle);
    // robotServo[i].write(thisAngle);



  }

  //  servo_channel1.attach(SERVO_Pin1);
  //  servo_channel2.attach(SERVO_Pin2);
  //  servo_channel3.attach(SERVO_Pin3);
  //  servo_channel4.attach(SERVO_Pin4);
  //  servo_channel5.attach(SERVO_Pin5);
  //  servo_channel6.attach(SERVO_Pin6);

}

// 目前只要大於等於16Bytes, 並且在最後有個井字好結尾，那就是合理的command;
void processRCString(String command) {

  int commandLength = command.length();

  if (commandLength > 15) {

    if (command.charAt(commandLength - 1) != '#') {  // 表示結尾不是預設的結果;

      return;

    }

  } else {

    return;
  }

  Serial.println(command);


  if (command.indexOf("SRV") > -1 || command.indexOf("SS4") > -1  ) {  // 表示伺服馬達操作;

    // Serial.println("接收到伺服馬達命令;");

    int i = 3;
    int servoIndex = 0;

    while (i < commandLength - 1) {     // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 4);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();

          
           if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

              if(receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
              if(receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
              oldServValue[servoIndex] = receiveServoValue[servoIndex];

//               int thisAngle = map(receiveServoValue[servoIndex], 544, 2400, 0, 180);
//               robotServo[servoIndex].write(thisAngle);
              robotServo[servoIndex].writeMicroseconds(receiveServoValue[servoIndex]);
           }
          
          servoIndex ++;

        }


      }

      i = i + 4;

    }

    processDCMotor(receiveServoValue[1], dcMotorPinA);
    processDCMotor(receiveServoValue[0], dcMotorPinB);

    // processServoCommand(receiveServoValue);     // 處理伺服馬達;

  } else if (command.indexOf("SRT") > -1 ) {  // 表示伺服馬達操作;

    // Serial.println("接收到伺服馬達命令;");

    int i = 3;
    int servoIndex = 0;

    while (i < commandLength - 1) {     // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 4);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();

          
           if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

              if(receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
              if(receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
              oldServValue[servoIndex] = receiveServoValue[servoIndex];

//               int thisAngle = map(receiveServoValue[servoIndex], 544, 2400, 0, 180);
//               robotServo[servoIndex].write(thisAngle);
              //crobotServo[servoIndex].writeMicroseconds(receiveServoValue[servoIndex]);
           }
          
          servoIndex ++;

        }

        

      }

      i = i + 4;

    }

    // 這裡要處理坦克的訊號
    int tankServo1 = 1500;
    int tankServo0 = 1500;

    if(receiveServoValue[1] >= 1500) {

      int duration = receiveServoValue[1] - 1500;
      
      tankServo1 = tankServo1 + duration;
      tankServo0 = tankServo0 - duration;
       
    } else {

      int duration = 1500 - receiveServoValue[1]  ;

      tankServo1 = tankServo1 - duration;
      tankServo0 = tankServo0 + duration;
       
    }

    if(receiveServoValue[0] >= 1500) {

       int duration = receiveServoValue[0] - 1500;
      
      tankServo1 = tankServo1 + duration;
      tankServo0 = tankServo0 + duration;
    } else {

      int duration = 1500 - receiveServoValue[0]  ;
      
      tankServo1 = tankServo1 - duration;
      tankServo0 = tankServo0 - duration;
    }

//    Serial.print("tankServo0:");
//    Serial.println(tankServo0);
//
//    Serial.print("tankServo1:");
//    Serial.println(tankServo1);

    if(tankServo0 < 1000) {
      tankServo0 = 1000;
    }

    if(tankServo0 > 2000) {
      tankServo0 = 2000;
    }

     if(tankServo1 < 1000) {
      tankServo1 = 1000;
    }

    if(tankServo1 > 2000) {
      tankServo1 = 2000;
    }

    receiveServoValue[0] = tankServo0;
    receiveServoValue[1] = tankServo1;

//    Serial.print("servo 0:");
//    Serial.println(receiveServoValue[0]);

    processDCMotor(receiveServoValue[0], dcMotorPinA);

    processDCMotor(receiveServoValue[1], dcMotorPinB);

    // 傳給ProcessServoCommand發送訊號;
    processServoCommand(receiveServoValue);     // 處理伺服馬達;

  } else if(command.indexOf("SR2") > -1) {
    // Serial.println("接收到第二組伺服馬達命令;");

    int i = 3;
    int servoIndex = 4;

    while (i < commandLength - 1) {     // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 4);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();

          
           if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

              if(receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
              if(receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
              oldServValue[servoIndex] = receiveServoValue[servoIndex];

//               int thisAngle = map(receiveServoValue[servoIndex], 544, 2400, 0, 180);
//               robotServo[servoIndex].write(thisAngle);
              robotServo[servoIndex].writeMicroseconds(receiveServoValue[servoIndex]);
           }
          
          servoIndex ++;

        }

        

      }

      i = i + 4;

    }
  } else if(command.indexOf("SS8") > -1) {
    // Serial.println("接收到第二組伺服馬達命令;");

    int i = 3;
    int servoIndex = 4;

    while (i < commandLength - 1) {     // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 2);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();

          
           if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

              if(receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
              if(receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
              oldServValue[servoIndex] = receiveServoValue[servoIndex];

               int pwmValue = map(receiveServoValue[servoIndex], 0, 99, 544, 2400);
               // robotServo[servoIndex].write(thisAngle);
               robotServo[servoIndex].writeMicroseconds(pwmValue);
           }
          
          servoIndex ++;

        }

      }

      i = i + 2;

    }
  }

}


//將命令發送至servo
void V7RCCommand(int servoValue1, int servoValue2, int servoValue3, int servoValue4, int servoValue5, int servoValue6, int servoValue7, int servoValue8)
{


  // Serial.println( servoValue1 );
  //       Serial.println("servoAngle[0]: "+String(servoAngle[0])+"servoAngle[1]: " +String(servoAngle[1]));

  if (servoValue1 != oldServValue[0]) {

    if (servoValue1 > servoMAXValue[0]) {
      servoValue1 = servoMAXValue[0];
    }
    else if (servoValue1 < servoMINValue[0]) {
      servoValue1 = servoMINValue[0];
    }

    oldServValue[0] = servoValue1;
    robotServo[0].write(servoValue1);
    //
    //delay(5);
  }

  if (servoValue2 != oldServValue[1]) {


    if (servoValue2 > servoMAXValue[1]) {
      servoValue2 = servoMAXValue[1];
    }
    else if (servoValue2 < servoMINValue[1]) {
      servoValue2 = servoMINValue[1];
    }

    oldServValue[1] = servoValue2;
    robotServo[1].write(servoValue2);
    //servo_channel2.write(servoValue2);
    //delay(5);
  }

  if (servoValue3 != oldServValue[2]) {

    if (servoValue3 > servoMAXValue[2]) {
      servoValue3 = servoMAXValue[2];
    }
    else if (servoValue3 < servoMINValue[2]) {
      servoValue3 = servoMINValue[2];
    }

    oldServValue[2] = servoValue3;
    robotServo[2].write(servoValue2);
    //servo_channel3.write(servoValue3);
    //delay(5);
  }

  if (servoValue4 != oldServValue[3]) {

    if (servoValue4 > servoMAXValue[3]) {
      servoValue4 = servoMAXValue[3];
    }
    else if (servoValue4 < servoMINValue[3]) {
      servoValue4 = servoMINValue[3];
    }

    oldServValue[3] = servoValue4;
    robotServo[3].write(servoValue4);
    //delay(5);
  }

  if (servoValue5 != oldServValue[4]) {

    if (servoValue5 > servoMAXValue[4]) {
      servoValue5 = servoMAXValue[4];
    }
    else if (servoValue5 < servoMINValue[4]) {
      servoValue5 = servoMINValue[4];
    }

    oldServValue[4] = servoValue5;
    robotServo[4].write(servoValue5);
    //delay(5);
  }

  if (servoValue6 != oldServValue[5]) {

    if (servoValue6 > servoMAXValue[5]) {
      servoValue6 = servoMAXValue[5];
    }
    else if (servoValue6 < servoMINValue[5]) {
      servoValue6 = servoMINValue[5];
    }

    oldServValue[5] = servoValue6;
    robotServo[5].write(servoValue6);
    //delay(5);
  }

  if (servoValue7 != oldServValue[6]) {

    if (servoValue7 > servoMAXValue[6]) {
      servoValue7 = servoMAXValue[6];
    }
    else if (servoValue7 < servoMINValue[6]) {
      servoValue7 = servoMINValue[6];
    }

    oldServValue[6] = servoValue7;
    robotServo[6].write(servoValue7);
    //delay(5);
  }

  if (servoValue8 != oldServValue[7]) {

    if (servoValue8 > servoMAXValue[7]) {
      servoValue8 = servoMAXValue[7];
    }
    else if (servoValue8 < servoMINValue[7]) {
      servoValue8 = servoMINValue[7];
    }

    oldServValue[7] = servoValue8;
    robotServo[7].write(servoValue8);
    //delay(5);
  }

  //  servo_channel1.write(servoValue2);
  //  servo_channel2.write(servoValue2);
  //  servo_channel3.write(servoValue3);
  //  servo_channel4.write(servoValue4);
  //  servo_channel5.write(servoValue5);
  //  servo_channel6.write(servoValue6);
  // servoPosition(SERVO_Pin1, servoAngle[0]);

}

void processServoCommand(int servoValue[]) {

  int thisIndex = 0;
  while (thisIndex < numOfServo && thisIndex < sizeof(servoValue)) {

    robotServo[thisIndex].writeMicroseconds(servoValue[thisIndex]);

    thisIndex ++;
  }

}

void processDCMotor(int pwmValue, int dcMotor[]) {
  if(pwmValue == 1500) {
    
    digitalWrite(dcMotor[0], LOW);
    analogWrite(dcMotor[1], 0);
  
  } else if(pwmValue > 1500) {
    int power = map(pwmValue, 1500, 2000, 0 , 255); 
    digitalWrite(dcMotor[0], LOW);
    analogWrite(dcMotor[1], power);
  } else {
    int power = map(pwmValue, 1500, 1000, 255 , 0); 
    digitalWrite(dcMotor[0], HIGH);
    analogWrite(dcMotor[1], power);
  }
}
