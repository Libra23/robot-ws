//
//  @file KrsServoSetParameter.ino
//  @brief KRS Servo Parameter set program
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2017/12/26
//
//  ID:0のストレッチ、スピードを設定します
//  ストレッチもスピードも90に設定し続けるので、実際は動いていないように見えます
//
#include <IcsHardSerialClass.h>

const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 100;

IcsHardSerialClass krs(&Serial,EN_PIN,BAUDRATE,TIMEOUT);  //インスタンス＋ENピン(2番ピン)およびUARTの指定

void setup() {
  // put your setup code here, to run once:
  krs.begin();  //サーボモータの通信初期設定
  krs.setPos(0,7500);   //最初に7500の位置で固定をしておきます
  
}

void loop() {

  int flag;

  krs.setStrc(0,90);  //ID0のストレッチを90にします
  krs.setSpd(0,90);  //ID0のスピードを90にします

  delay(1000);
}
