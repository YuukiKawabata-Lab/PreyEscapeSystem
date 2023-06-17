

void setup() {
  //Motor
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(9,OUTPUT);
  //LED
  pinMode(12,OUTPUT);
  //Serial
  Serial.begin(115200);
}

void loop() {
  analogWrite(9,0);
  digitalWrite(12,LOW); 
      
  if (Serial.available()>0) {      // If anything comes in Serial (USB),
    char a=Serial.read();
    
    digitalWrite(12,HIGH);
    
    digitalWrite(4,HIGH);
    digitalWrite(5,LOW);
    

// ////////////////////////////////    
    //Motor settings
    analogWrite(9,13);//Change second parameter to change motor speed between 1 to 255
    delay(50);//Duration of motor running (ms) 
    analogWrite(9,13);
    delay(800);
// ////////////////////////////////    
  
    analogWrite(9,0);
    while(1){
      delay(3000);
      }
    }
  }
  
