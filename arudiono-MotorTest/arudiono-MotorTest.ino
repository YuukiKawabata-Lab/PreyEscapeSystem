void setup() {
  //Motor
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(9,OUTPUT);
  //LED
  pinMode(12,OUTPUT);

}

void loop() {  
  digitalWrite(12,HIGH);
    
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);

// ////////////////////////////////    
    //モーターの設定
  analogWrite(9,100);//回転速度:1~255
  delay(1000);//回転時間:ミリ秒
// ////////////////////////////////    
  
  analogWrite(9,0);
  while(1){
      delay(3000);
    }
}
  
