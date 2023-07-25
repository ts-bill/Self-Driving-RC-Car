#include <Servo.h>
#define DIR_F 19
#define DIR_R 18
#define PWM 20
Servo sdcpservo ;
char Data[8];
bool use_Button[8];
bool Analog1_dir[8];
bool Analog2_dir[8];
int8_t Re,An1,An2,Angle,Speed;
int8_t RA_UD,RA_RL,LA_UD,LA_RL,Sign;
bool motorstop;
bool motorslowdown;
bool lock=true;
int s=0;
int pos ;
//int pos1;
//int pos2;
void setup() {  
Serial.begin(115200);
pinMode(DIR_F,OUTPUT);
pinMode(DIR_R,OUTPUT);
pinMode(PWM,OUTPUT);
pinMode(13,OUTPUT);
sdcpservo.attach(21);
sdcpservo.write(92);

} 

void loop() { 
  drivemotor();
  
  }
void serialEvent(){
  //Serial.println('c');
while(Serial.available()){
  //Serial.println('s');
  Data[s]=Serial.read();
  //Serial.print(Data[s]);
  if(Data[s-4]=='A' && Data[s]=='$' && s>=4){
    //Serial.println('a');
    lock=false;
    s = 0; 
    }
    else if(s>5){
      lock= true;
      s=0;
      //Serial.println('t');
      }
    else{
      //Serial.println('d');
      s++;  
      }
    }
     if(!lock){
   Angle = (int8_t) Data[1];
   Speed = (int8_t) Data[2];
   Sign =(int8_t) Data[3];
   //RA_UD=(byte) Data[4];
   //LA_RL=(byte) Data[5];
   //Serial.print("///");
   Serial.println(Angle);
   //Serial.println(r3);
   if(Sign == 1 || Sign == 2){
       motorstop=1;
    }
  else if(Sign == 4){
       motorslowdown=1;
  }
  
  else{
       motorstop=0;
       motorslowdown=0;  
  }

  }
}

void drivemotor(){
   if (Analog1_dir[1]==1  && !motorstop){
      if(motorslowdown==1){
      digitalWrite(DIR_F,HIGH);
      digitalWrite(DIR_R,LOW);
      analogWrite(PWM,abs(15+(LA_UD/25)));
      digitalWrite(13,HIGH);
      //Serial.println("FORWARD");
      }
      else{
      digitalWrite(DIR_F,HIGH);
      digitalWrite(DIR_R,LOW);
      analogWrite(PWM,abs(15+(LA_UD/18)));
      digitalWrite(13,HIGH);
      //Serial.println("FORWARD");
        
      }
      
     }
  else if(Analog1_dir[0]==1){
      digitalWrite(DIR_F,LOW);
      digitalWrite(DIR_R,HIGH);
      analogWrite(PWM,abs(15+(LA_UD/18)));
      digitalWrite(13,HIGH);
      //Serial.println("BACKWARD");
  }
  
  else{
      digitalWrite(DIR_F,LOW);
      digitalWrite(DIR_R,LOW);
      analogWrite(PWM,0);
      digitalWrite(13,LOW); 
      //Serial.println("STOP");  
      }
  if(Analog1_dir[6]==1){
  pos = map(RA_RL,0,255,91,64);
  }
  else if(Analog1_dir[7]==1){
  pos = map(RA_RL,0,255,91,118);
  }
  else {
  pos =91;  
 
    
  }
  sdcpservo.write(pos);   
  
}
