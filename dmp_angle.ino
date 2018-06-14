///CH1-> yaw-13; CH2-10; CH3-8 ; CH4-11 ; CH5-ARM-12 ; CH6-9



#include "I2Cdev.h"

#define mpu6050_address 0x68
float gyro[3],acc[3];
float gyro_cal[3];
unsigned int calibration=0;const float gyrolsb = 32.8;

#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h> 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


#define INTERRUPT_PIN 2  
 
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


double gyrocal[3]={0,0,0};
int cal_int, start, gyro_address;


////####GLOBAL VARIABLES##########\\\\\\\\\\\\ 

volatile int  roll_receiver,pitch_receiver,yaw_receiver;
volatile byte last_channel_1, last_channel_2, last_channel_3, last_channel_4,last_channel_5,last_channel_6;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4,timer_channel_5,timer_channel_6, esc_timer, esc_loop_timer,loop_timer_pid;

volatile long timer_1, timer_2, timer_3, timer_4,timer_5,timer_6, current_time;

unsigned int throttle,throttle1,last_throttle, rec_arm,rec6;
double u2[4];                 ///error matrix
int esc_1, esc_2, esc_3, esc_4;      ///outputs to motors

double roll_kp=6;
double roll_kd=300;
double pitch_kp=6.0;
double pitch_kd=300;
double roll_last_error = 0  , roll_error;
double pitch_last_error = 0 ,pitch_error;
double derivative_roll      ,derivative_pitch ;
double roll_pid=0, pitch_pid=0 ;
double yaw_kp = 3; double yaw_kd =50;
double yaw_last_error = 0 , yaw_error;
double yaw_pid=0;

////SET POINTS\\\\

float pitch_des  =   0;  
float roll_des   =   0;  
float yaw_des    =   0; 

float tem_roll=0, tem_pitch=0,tem_yaw=0;

///SENSOR DATAS\\\

double _pitch  ;  int pitch_dot;
double _roll   ;  int roll_dot;
double _yaw    ;  int yaw_dot;

///FILTER FOR DERIVATIVE TERM

double filter_roll[3]   = {0,0,0} ;        ///3 terms filtering
double filter_pitch[3]  = {0,0,0} ;        ///initialise with 0 to prevent blow up
double filtered_roll =0 ,filtered_pitch=0;
////###################################
/*
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long  timer_3, timer_6, current_time,timer_5;
unsigned long loop_timer;
*/
double gyro_pitch, gyro_roll, gyro_yaw;



float yaw,roll,pitch;

float roll_app=0,pitch_app=0;



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

bool isDiscontinuity;
void setup()
{
  
  
 char ch='s';

setupmpu();


for(int i =0 ; i<10000;i++){calcang();;yaw_des=yaw;Serial.println("##");}
Serial.print(yaw_des);
if(yaw_des<-90||yaw_des>90){
  isDiscontinuity=true;
  if(yaw_des<-90)yaw_des=-(yaw_des+180);
  else yaw_des=180-yaw_des;
}
else isDiscontinuity=false;
Serial.print(" yaw_des is : ");Serial.print(yaw_des);

/*    
while(1){
  calcang();
  yaw_des = yaw ;
  Serial.print("Enter y if yaw is stable ,");Serial.print(yaw_des);Serial.println();
  ch = Serial.read();
  if(ch=='y'){
    yaw_des = yaw ;
    Serial.print("Yaw is set to :");Serial.print(yaw_des);Serial.println();
    break;
  }
}

/*/

///////////CONTROLL PARTS BEGINr

  
  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.  
  PCMSK0 |= (1 << PCINT0);                                                //Set PCINT0 (digital input 8) to trigger an interrupt on state change.THROTLE
  PCMSK0 |= (1 << PCINT1);                                                //9  pin
  PCMSK0 |= (1 << PCINT4);                                                //12 pin
  PCMSK0 |= (1 << PCINT2);                                                // (digital input 10) as roll angle .
  PCMSK0 |= (1 << PCINT3);                                                // (digital input 11) as pitch angle 
 // PCMSK0 |= (1 << PCINT5);                                                //13 for yaw
   for (int cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);   }                                             //Wait 3000us.



    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.




   loop_timer_pid = micros();

  
}
unsigned long throttle2,loop_1_time;
unsigned long loop_count=0;
void loop()
{




/*
pitch_kp = roll_kp;

*/
//Serial.println(yaw_kp);

  calcang();  
loop_count=loop_count+1;
tem_roll  = (roll_receiver - 1500) * 0.25 ;     ///converting to degree and tilt range -20 to +20 roll
tem_pitch   = -(pitch_receiver- 1500) *   0.25 ;    ///converting  to degree and tilt range -20 to +20 pitch
tem_yaw    = (yaw_receiver - 1500)  * 0.25 ;

_roll   = roll  ;
_pitch  = pitch ;
_yaw    = yaw   ;
if(isDiscontinuity==1&&_yaw<0)_yaw=-(180+_yaw);
else if(isDiscontinuity==1&&_yaw>0)_yaw=(180-_yaw);

roll_error  =  roll_des  -  _roll ;
pitch_error =  pitch_des - _pitch ;
yaw_error   =  yaw_des   - _yaw   ;








derivative_roll   = roll_error  - roll_last_error  ; 
derivative_pitch  = pitch_error - pitch_last_error ;/*
filter_roll[2] = filter_roll[1];
filter_roll[1] = filter_roll[0];
filter_roll[0] = derivative_roll;
filtered_roll = (filter_roll[2] + filter_roll[1] + filter_roll[0])/3 ;

filter_pitch[2] = filter_pitch[1];
filter_pitch[1] = filter_pitch[0];
filter_pitch[0] = derivative_pitch;
filtered_pitch = (filter_pitch[2] + filter_pitch[1] + filter_pitch[0])/3 ;*/


roll_pid  =  roll_kp  *  (roll_error)   + roll_kd  * derivative_roll ;                     //.error in phi calculated
pitch_pid =  pitch_kp *  (pitch_error)  + pitch_kd * derivative_pitch;        //.error in theta calculated
yaw_pid   =  -yaw_kp * (yaw_error)      - yaw_kd * (yaw_error - yaw_last_error);
if(isDiscontinuity==1)yaw_pid=-yaw_pid;
roll_last_error=roll_error;
pitch_last_error=pitch_error;
yaw_last_error=yaw_error;

//Serial.print(esc_1);Serial.print(" , ");Serial.print(esc_2);Serial.print(" , ");Serial.print(esc_3);Serial.print(" , ");Serial.print(esc_4);Serial.println(throttle);
//Serial.print(derivative_roll);Serial.print(" , ");Serial.print(derivative_pitch);Serial.print(" , ");Serial.print(roll_error);Serial.print(" , ");Serial.println(pitch_error);



if((rec_arm>1500)&&throttle>1050)
{
  esc_1 = throttle  + roll_pid  + tem_roll + tem_yaw - yaw_pid; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_2 = throttle  + pitch_pid + tem_pitch - tem_yaw + yaw_pid ; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle  - roll_pid  - tem_roll + tem_yaw - yaw_pid; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_4 = throttle  - pitch_pid - tem_pitch - tem_yaw + yaw_pid; //Calculate the pulse for esc 4 (front-left - CW)


      esc_1 = constrain(esc_1,1000,2000);           
      esc_2 = constrain(esc_2,1000,2000);           
      esc_3 = constrain(esc_3,1000,2000);           
      esc_4 = constrain(esc_4,1000,2000);           

}

else{
  
esc_1 = 1000;
esc_2 = 1000;
esc_3 = 1000;
esc_4 = 1000;
}
if(throttle>1900){Serial.print(" , ");Serial.println("1");}      
if(throttle<900){Serial.print(" , ");Serial.println("-1");}
      while(micros() - loop_timer_pid < 4000);                                      //We wait until 4000us are passed.

      
       
       loop_timer_pid = micros();                                   //Set the timer for the next loop.










  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer_pid;                                     //Calculate the time of the falling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer_pid;                                     //Calculate the time of the falling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer_pid;                                     //Calculate the time of the falling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer_pid;                                     //Calculate the time of the falling edge of the esc-4 pulse.


  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }

  
}


ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    throttle1 = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }

//Channel 5=========================================12 pin
  if(PINB & B00010000){                                                    
    if(last_channel_5 == 0){                                              
      last_channel_5 = 1;                                                   
      timer_5 = current_time;                                             
    }
  }
  else if(last_channel_5 == 1){                                             
    last_channel_5 = 0;                                                    
    rec_arm = current_time - timer_5;                            
  }

 //Channel 6=========================================9
  if(PINB & B00000010){                                                    
    if(last_channel_6 == 0){                                              
      last_channel_6 = 1;                                                   
      timer_6 = current_time;                                             
    }
  }
  else if(last_channel_6 == 1){                                             
    last_channel_6 = 0;                                                    
    yaw_receiver = current_time - timer_6;                            
  }



if(throttle1-last_throttle<5||throttle1-last_throttle>-5)
throttle=last_throttle;
last_throttle=throttle1;



 //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    pitch_receiver = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }


  
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    roll_receiver = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
///channel 1 yaw
/*
  if(PINB & B00100000 ){                                                    //Is input 13 high?
    if(last_channel_2 == 0){                                                //Input 13 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 13 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
   rec6 = current_time - timer_2;                             //Channel 4 is current_time - timer_4.
  }
*/
}











void setupmpu() {
   
    //pinMode(LED_PIN, OUTPUT);
   /* for(calibration = 0;calibration<2000;calibration++)
{
 record_data(); 
}*/
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

   
    Serial.begin(115200);
   
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(16);    //33 
    mpu.setYGyroOffset(66);    //  43
    mpu.setZGyroOffset(28);     //  41
    mpu.setZAccelOffset(1432); // 1688 factory default for my test chip  // ( 1452
   mpu.setXAccelOffset(1196);
   mpu.setYAccelOffset(-1908);

    if (devStatus == 0) {
        
        
        mpu.setDMPEnabled(true);

        
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

      
        dmpReady = true;

       
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
   
    

}





void calcang() {
    
    if (!dmpReady) return;

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    
    fifoCount = mpu.getFIFOCount();

    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & 0x02) {
        
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

     
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
     
        fifoCount -= packetSize;

     mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            yaw = ypr[0]*180/M_PI;
            pitch = ypr[1]*180/M_PI;
            roll = ypr[2]*180/M_PI;
  
     
    }
}



void record_data(){


    Wire.beginTransmission(mpu6050_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                     //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(mpu6050_address,14);                                      //Request 14 bytes from the gyro.
   
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
/*    acc[0] = ((Wire.read()<<8|Wire.read())>>acc_smt)<<acc_smt;                               //Add the low and high byte to the acc_x variable.
    acc[1] = ((Wire.read()<<8|Wire.read())>>acc_smt)<<acc_smt;                               //Add the low and high byte to the acc_y variable.
    acc[2] = ((Wire.read()<<8|Wire.read())>>acc_smt)<<acc_smt;                               //Add the low and high byte to the acc_z variable.*/
    Wire.read()<<8|Wire.read();                                        //Add the low and high byte to the temperature variable.
    gyro[0] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.


/*
  if(1){
    gyro[0] -= gyro_cal[0];                                       //Only compensate after the calibration.
    gyro[1] -= gyro_cal[1];                                       //Only compensate after the calibration.
    gyro[2] -= gyro_cal[2];                                       //Only compensate after the calibration.

      for(int i=0;i<3;i++)
  {
    gyro[i]/=gyrolsb;
   // acc[i] /=acclsb;
  }

  }
 
  else
  {
    gyro_cal[0] += gyro[0];                                       //Only compensate after the calibration.
    gyro_cal[1] += gyro[1];                                       //Only compensate after the calibration.
    gyro_cal[2] += gyro[1];     
     
  
 

  }*/
}

