#include <Servo.h>
int motorNum;
Servo motor[4]; 
int recvMotorSpeed[4];// iPhoneから受信したモーターのスピード
int preRecvMotorSpeed[4];// iPhoneから受信した前回のモータースピード
int motorSpeed[4];
int mapdMotorSpeed[4];
int motorArm;
int rcData[4];
void setup() 
{ 
  Serial.begin(57600);
  Serial1.begin(57600);
  
  // RCリード   
  pinMode(2, INPUT);

  motor[0].attach(3,836,2400);
  motor[1].attach(5,836,2400);
  motor[2].attach(6,836,2400);
  motor[3].attach(7,836,2400);


  
  
  motorArm = 1;
  motorNum = 4;

  int i;
  for(i=0;i<motorNum;i++) {
    motorSpeed[i] = -1;
    motor[i].writeMicroseconds(motorSpeed[i]);
  }

  attachInterrupt(0,readRC,CHANGE);

  delay(1500);
  
  motor[0].writeMicroseconds(0);
  motor[1].writeMicroseconds(0);
  motor[2].writeMicroseconds(0);
  motor[3].writeMicroseconds(0);
  
  delay(1500);
  
} 



// PWMを割り込みで読み取る
unsigned long uptime;
unsigned long downtime;
unsigned long pWidth;

void readRC() {
  if(digitalRead(2)){
    uptime = micros();
  }
  else{
    downtime = micros();
    pWidth = downtime - uptime;
  }
}




void loop() 
{  
  int i;
  char c = 0;

  int commandIndex = 0;
  int findCount = 0;

  for(i=0;i<motorNum;i++) {
    recvMotorSpeed[i] = 0;
  }

  // Serial1を読み込む
  while (1){
    c = Serial1.read();

    if (c < 0){
      break;
    }

    if (c == 0){
      // 0は終端
      commandIndex = 0;
      if (findCount > 1){
        break;
      }
      findCount++;
    }
    else{
      recvMotorSpeed[commandIndex] = c;
      commandIndex++;
      if (commandIndex > 3){
        commandIndex = 0;
      }
    }
  }

  // 全部受信してなかったら前回の受信を使う
  if (findCount > 1){
    for(i=0;i<motorNum;i++) {
      preRecvMotorSpeed[i] = recvMotorSpeed[i];
    }
  }else{
    for(i=0;i<motorNum;i++) {
      recvMotorSpeed[i] = preRecvMotorSpeed[i];
    }
  }





  for(i=0;i<motorNum;i++) {
    motorSpeed[i] = pWidth;
  }

  // レシーバーを読み取る
  //  rcData[0] = pulseIn(2, HIGH, 25000);
  //  rcData[1] = pulseIn(4, HIGH, 25000);
  //  rcData[2] = pulseIn(5, HIGH, 25000);
  //  rcData[3] = pulseIn(6, HIGH, 25000);

  //  Serial.println(pWidth);

  // モーターの出力にマッピング
  for(i=0;i<motorNum;i++) {
    if (recvMotorSpeed[i] == 1){
      // 1の時はモーターを止める
      mapdMotorSpeed[i] = 0;
    }else{
      mapdMotorSpeed[i] = map(recvMotorSpeed[i],1,127,1175,2240);
    }
    
  }

  // モーターを回す
  for(i=0;i<motorNum;i++) {
    motor[i].writeMicroseconds(mapdMotorSpeed[i]);
  }
  
  

  // モーターの状態を知らせる(そのうちRCデータの全送信をする）
  Serial1.print(pWidth);

  for(i=0;i<motorNum;i++) {
    Serial1.print(':');
    Serial1.print(mapdMotorSpeed[i]);
  }

  for(i=0;i<motorNum;i++) {
    Serial1.print(':');
    Serial1.print(recvMotorSpeed[i]);
  }
  // 終端文字
  Serial1.print(';');

  delay(15);
} 














