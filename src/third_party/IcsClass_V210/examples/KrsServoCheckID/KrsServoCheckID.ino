//
//  @file KrsServoCheckId.ino
//  @brief KRS ID command check program
//  @author Kondo Kagaku Co.,Ltd.
//  @date 2020/02/20
//
//  IDを1に変更し、再度読み込んで書き込んだIDかどうかチェックします
//  ICSの通信にはHardwareSerialを使います。
//  表示機がないので、実際に動いていないように見えます。
//	あくまでも記述方法を参考にしてください。
//

#include <IcsHardSerialClass.h>

const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 100;

const byte ACK_CHECK_PIN = 3; 

IcsHardSerialClass krs(&Serial,EN_PIN,BAUDRATE,TIMEOUT);  //インスタンス＋ENピン(2番ピン)およびUARTの指定


void setup() {
  // put your setup code here, to run once:
  krs.begin();  //サーボモータの通信初期設定
  pinMode(ACK_CHECK_PIN ,OUTPUT);  //ACK確認を出力するピン設定
  digitalWrite(ACK_CHECK_PIN,HIGH);
}

void loop() {

  const byte SET_ID = 1;

  int reId;

  //IDの設定
  krs.setID(SET_ID);

  //IDの取得
  reId = krs.getID();
  if(reId != SET_ID)
  {
    //失敗した時の処理
    delay(1000);
    digitalWrite(ACK_CHECK_PIN,LOW);
  }  

  for(;;);//ここで終わり
  
}
