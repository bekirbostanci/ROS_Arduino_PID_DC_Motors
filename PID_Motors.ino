#include <TimerOne.h>
const int IN1=24;
const int IN2=25;
const int ENA=6;
const int IN3=28;
const int IN4=29;
const int ENB=7;


double kp = .07;
double ki = .01;
double kd = .03;
 
 

double error;
double lastError;
double input;
double out;
double Setpoint;
double cumError, rateError;


double Merror;
double MlastError;
double Minput;
double Mout;
double MSetpoint;
double McumError, MrateError;


volatile double Count = 0;
double rpm;


volatile double MCount = 0;
double Mrpm;



void setup(){
        
        Setpoint = 5000;   
        MSetpoint = 5000;   
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(ENA, OUTPUT);
        pinMode(IN4, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(ENB, OUTPUT);
      
        pinMode(3, INPUT);
        pinMode(2, INPUT);
        
        attachInterrupt(digitalPinToInterrupt(3), rightEncoderEvent, CHANGE);
        attachInterrupt(digitalPinToInterrupt(2), leftEncoderEvent, CHANGE);
        Timer1.initialize(100000); 
        Timer1.attachInterrupt(isr); 

        digitalWrite(30,HIGH);
        digitalWrite(31,LOW);
     
        digitalWrite(24,HIGH);
        digitalWrite(25,LOW); 
        Serial.begin(9600);
        //delay(500)  ;   
        
}    

void isr()        // interrupt service routine - tick every 0.1sec
{   
       rpm = 60.0*(Count/64)/0.1;  //calculate motor speed, unit is rpm
       Count=0;
            
        error = Setpoint - rpm; 
        cumError += error; //* elapsedTime;                
        rateError = (error - lastError);///elapsedTime;                                  
        double out = kp*error + ki*cumError + kd*rateError; 
        analogWrite(ENA,out);  
        lastError = error;                     


        Serial.println(out); 

        Mrpm = 60.0*(MCount/64)/0.1;  //calculate motor speed, unit is rpm
        MCount=0;
            
        Merror = MSetpoint - Mrpm; 
        McumError += Merror; //* elapsedTime;                
        MrateError = (Merror - MlastError);///elapsedTime;                                  
        double Mout = kp*Merror + ki*McumError + kd*MrateError; 
        analogWrite(ENB,Mout);   
        MlastError = Merror;          
                                                
}
int setcounter = 0 ;
void loop()
{
  /*
    setcounter ++; 
    if(setcounter == 300 )
    {
      Setpoint = Setpoint + 10; 
      MSetpoint = MSetpoint + 10; 
    }
  */
  Serial.print("  ");            

}
void leftEncoderEvent() {
Count ++ ;
}

void rightEncoderEvent() {
MCount ++ ;
}

  
