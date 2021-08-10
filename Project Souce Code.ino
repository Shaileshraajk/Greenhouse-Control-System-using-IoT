#include <Servo.h>
#define _LightwritePin 7
#define _PumpWritePin 8

Servo mainServo;
int _CurrentHumidityValue = 0;
double destmp;  //desired Temp value
double instmp; //Instantaineous Temp
double tmpact=0; //Temp_act

double deslt;  //desired Light value
double inslt; //Instantaineous Light
double ltact=0; //Lt_act

double desmst;  //desired Moist value
double insmst; //Instantaineous Moist
double mstact=0; //Moist_act

int light_temp= A0;      //temperature sensor
int moistsensor= A2;     //Moisture Sensor
int photoresistor= A1;   //LDR sensor
//initial variables:
int sensorValue = 0;
void Light(int _Status) 
{
  if (_Status == 0 ) {
    digitalWrite(_LightwritePin,LOW);
  } else {
    digitalWrite(_LightwritePin,HIGH);
  }  
}
void PumpEngine(int _Status) 
{
  if (_Status == 0 ) {
    digitalWrite(_PumpWritePin,LOW);
  } else {
    digitalWrite(_PumpWritePin,HIGH);
  }  
}

double computePIDTmp(double instval)
{     
        double Kp=-0.164, Ki=-0.0321, Kd=0;
  		unsigned long currentTime, previousTime; //Time parameters
		double elapsedTime;
		double error, lastError; //Error calculation
		double cumError, rateError;
		currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = destmp - instval;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = Kp*error + Ki*cumError + Kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
double computePIDLt(double instval)
{     
  		double Kp = -0.00796, Ki = -0.131, Kd = -5.43e-05;
        unsigned long currentTime, previousTime; //Time parameters
		double elapsedTime;
		double error, lastError; //Error calculation
		double cumError, rateError;
		currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = deslt - instval;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = Kp*error + Ki*cumError + Kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
double computePIDMst(double instval)
{     	double Kp = -3.41e+03, Ki = -1.26e+05, Kd = -23.2;
        unsigned long currentTime, previousTime; //Time parameters
		double elapsedTime;
		double error, lastError; //Error calculation
		double cumError, rateError;
		currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = desmst - instval;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = Kp*error + Ki*cumError + Kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
void setup()
 {
  // initialize serial communication:
  Serial.begin(9600);
  
  //outputs
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  mainServo.attach(10);
  //inputs
  pinMode(photoresistor, INPUT);
  pinMode(light_temp,INPUT);
  pinMode(moistsensor,INPUT);
  destmp = 28.00;
  deslt = 511.00;
  desmst = 28.00;
}

void loop() {

  //temperature sensor
  
 int tmp = analogRead(light_temp);//Reading data from the sensor.This voltage is stored as a 10bit number.
  float voltage = (tmp * 5.0)/1024;//(5*temp)/1024 is to convert the 10 bit number to a voltage reading.
  float milliVolt = voltage * 1000;//This is multiplied by 1000 to convert it to millivolt.
  float tmpf =  (milliVolt-500)/10 ;//For TMP36 sensor. Range(−40°C to +125°C)
  int tmpCel=int(tmpf);
  instmp=tmpCel;
  //Serial.println("TEMP:");
  //Serial.println(instmp);
  
  tmpact = computePIDTmp(instmp);
  
  
  if(tmpact<0)
  {
    Light(1);
       //Serial.println("LED ON ");
  }
  else{
    Light(0);
       //Serial.println("LED OFF");
  }
  
  // LDR sensor code
  sensorValue = analogRead(photoresistor); 
   Serial.println("LDR:");
 Serial.println(sensorValue);
 inslt=sensorValue;
 ltact=computePIDLt(inslt);
 if(ltact>0) 
 {
   //Serial.println("Ceil Open");
   mainServo.write(90);
 }
  else {
	//Serial.println("Ceil Close");
    mainServo.write(0);
  }
  //Code Humidity
  _CurrentHumidityValue = int((float(analogRead(moistsensor))*5/(1023))/0.01) - 49;
   //Serial.println("HUM:");
  //Serial.println(_CurrentHumidityValue);
  insmst =_CurrentHumidityValue;
  mstact=computePIDMst(insmst);
  
  if(mstact<0)
  {
    PumpEngine(1);
    //Serial.println("PUMP ON ");
  }
  else{
    PumpEngine(0);
   //Serial.println("PUMP OFF");
  }}
 