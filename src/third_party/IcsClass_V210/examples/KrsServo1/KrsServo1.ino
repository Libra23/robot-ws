//
//  @file KrsServo1.ino
//  @brief KrsServoSample1
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2017/12/26
//
//  ID:0のサーボをポジション指定で動かす
//  範囲は、左5500 - 中央7500 - 右9500
//  0.5秒ごとに指定数値まで動く
//  ICSの通信にはHardwareSerialを使います。
//


#include <IcsHardSerialClass.h>

const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 1000;		//通信できてないか確認用にわざと遅めに設定

IcsHardSerialClass krs(&Serial,EN_PIN,BAUDRATE,TIMEOUT);  //インスタンス＋ENピン(2番ピン)およびUARTの指定


void setup() {
  // put your setup code here, to run once:
  krs.begin();  //サーボモータの通信初期設定
}


void loop() {
  // put your main code here, to run repeatedly:
    krs.setPos(0,7500);      //位置指令　ID:0サーボを7500へ 中央
    delay(500);              //0.5秒待つ
    krs.setPos(0,9500);      //位置指令　ID:0サーボを9500へ 右
    delay(500);              //0.5秒待つ
    krs.setPos(0,7500);      //位置指令　ID:0サーボを7500へ 中央
    delay(500);              //0.5秒待つ
    krs.setPos(0,5500);      //位置指令　ID:0サーボを5500へ 左
    delay(500);  
}
