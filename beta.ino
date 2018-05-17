////////////////////IMU PARTS\\\\\\\\\\\\\\\\\\\\\\\\\\\

#include<Wire.h>
#include"IMU.h"


#define degconvert 57.2957786 
float accelw =0.004;
float gyrow= 0.996;
#define acc_smt 8
#define inv_degconvt 0.01745329279


int calibration;
double dt;
float roll=0,pitch=0,yaw=0;
float gyro[3],acc[3];
float gyro_cal[3];
uint32_t loop_timer;
bool flag=0;
float out[3];
float dcm[][3] = { 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1 };
double dtheta[3] = { 0, 0, 0};              //roll,pitch,yaw

////////////////IMU PATRS ENDS


/////CONTROL PART STARTS HERE

////############################################\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//unsigned long current_time,throttle,throttle1,last_throttle,timer_1; byte last_channel_1;

////####GLOBAL VARIABLES##########\\\\\\\\\\\\

int roll_receiver,pitch_receiver;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4,last_channel_5,last_channel_6;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4,timer_channel_5,timer_channel_6, esc_timer, esc_loop_timer,loop_timer_pid;

unsigned long timer_1, timer_2, timer_3, timer_4,timer_5,timer_6, current_time;

unsigned int throttle,throttle1,last_throttle, rec_arm,rec6;
double u2[4];                 ///error matrix
int esc_1, esc_2, esc_3, esc_4;      ///outputs to motors
////####COEFFICIENTS#########\\\\\\\

/*
double Kp_roll=20;
double Kd_roll=5;
double Kp_pitch=Kp_roll;
double Kd_pitch=Kd_roll;
double Kp_shi=0;
double Kd_shi=0;
double pwm2f;*/

///CASCADE PID CONTROLLER

double roll_kp_1 = 2.5  ;  double roll_kd_1     =   1.5  ;
double roll_kp_2 = 3.0  ;  double roll_kd_2     =   1.75 ;

double pitch_kp_1 = roll_kp_1 ; double pitch_kd_1 = roll_kd_1 ;
double pitch_kp_2 = roll_kp_2 ; double pitch_kd_2 = roll_kd_2 ;

////SET POINTS\\\\

float pitch_des  =   0;  
float roll_des   =   0;  
float yaw_des    =   0; 

///SENSOR DATAS\\\

int _pitch  ;  int pitch_dot;
int _roll   ;  int roll_dot;
int _yaw    ;  int yaw_dot;


////PITCH
float pitch_error_1 , pitch_error_2 , pitch_last_error_1 =0 , pitch_last_error_2=0;  //initialise to prevent initial blow up
float pitch_pd_1    , pitch_pd_2;
///ROLL
float roll_error_1 , roll_error_2 ,  roll_last_error_1=0, roll_last_error_2=0; 
float roll_pd_1    , roll_pd_2;
/////CONTROLL PART ENDS






void setup()
{
  ////////////////////IMU PARTS\\\\\\\\\\\\\\\\\\\\\\\\\\\

  Serial.begin(115200);
  Wire.begin();

  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #endif
  
setupmpu();
for(calibration = 0;calibration<2000;calibration++)
{
 record_data(); 
}
for(int i=0;i<3;i++)
{
gyro_cal[i]/=2000;
  
}
roll = atan2(acc[1],acc[2]);
pitch = atan2(-acc[0],acc[2]);                           //Starting Angle

float dcm[][3] = {cos(pitch) , sin(roll)*sin(pitch), cos(roll)*sin(pitch),
                  0          , cos(roll)           , -sin(roll)          ,
                  -sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch)};

 

////////////////IMU PATRS ENDS


///////////CONTROLL PARTS BEGIN

  
  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.  
  PCMSK0 |= (1 << PCINT0);                                                //Set PCINT0 (digital input 8) to trigger an interrupt on state change.THROTLE
  PCMSK0 |= (1 << PCINT1);                                                //9  pin
  PCMSK0 |= (1 << PCINT4);                                                //12 pin
  PCMSK0 |= (1 << PCINT2);                                                // (digital input 10) as roll angle .
  PCMSK0 |= (1 << PCINT3);                                                // (digital input 11) as pitch angle 
   for (int cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);   }                                             //Wait 3000us.



    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.


////////CONTROL PARTS ENDS

loop_timer = micros();
loop_timer_pid = micros();

}

//////////////////IMU functions ends
unsigned long throttle2,loop_1_time;

void loop()                   //***********************************************************************
{

dt = (micros()-loop_timer)/1000000.0;
loop_timer = micros();


angle_calc();
/*
Kp_phi = 0.05*rec6 - 50;
Kp_theta = Kp_phi;
*/
// prt_ang();


roll_des  = (roll_receiver - 1500) * 0.04;     ///converting to degree and tilt range -20 to +20 roll
pitch_des = (pitch_receiver- 1500) * 0.04;    ///converting  to degree and tilt range -20 to +20 pitch
////////#####


_roll=out[0];
roll_dot=gyro[0];
_pitch=out[1];
pitch_dot=gyro[1];
/*
u2[0]=Kp_roll*(roll_des-_roll)+Kd_phi*(-roll_dot);                     //.error in phi calculated
u2[1]=-Kp_pitch*(pitch_des-_pitch)-Kd_pitch*(-theta_dot);        //.error in theta calculated


/*
Serial.print(phi);
Serial.print(" *** ");
Serial.println(theta);
*/       
                                                                //YAW is not in self correction  YAW correction is removed 

roll_error_1=roll_des - _roll;

roll_pd_1          =        roll_kp_1*(roll_error_1)    + roll_kd_1*(roll_error_1-roll_last_error_1);
roll_last_error_1  =        roll_error_1;

roll_error_2       =        roll_pd_1-roll_dot;

roll_pd_2          =        roll_kp_2*roll_last_error_2 + roll_kd_2*(roll_error_2-roll_last_error_2);
roll_last_error_2  =        roll_error_2;



pitch_error_1=pitch_des + _pitch;  ///the _pitch is added as ,the direction of pitch is opposite

pitch_pd_1          =        pitch_kp_1*(pitch_error_1)    + pitch_kd_1*(pitch_error_1-pitch_last_error_1);
pitch_last_error_1  =        pitch_error_1;

pitch_error_2       =        pitch_pd_1-pitch_dot;

pitch_pd_2          =        pitch_kp_2*pitch_last_error_2 + pitch_kd_2*(pitch_error_2-pitch_last_error_2);
pitch_last_error_2  =        pitch_error_2;

/*
Serial.print(roll_des);Serial.print(",");Serial.print(pitch_des);Serial.println();//Serial.print(",");Serial.print(roll_pd_2);Serial.print(",");Serial.print(pitch_pd_2);Serial.println();
Serial.print("###");Serial.println();
Serial.print(roll_receiver - 1500);Serial.print(",");Serial.print(pitch_receiver - 1500);Serial.println();
Serial.print("###");Serial.println();
*/
if((rec_arm>1500)&&throttle>1010)
{
  
  /*
  esc_1 = throttle - u2[1]  + u2[0]; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_2 = throttle + u2[1]  + u2[0]; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle + u2[1]  - u2[0]; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_4 = throttle - u2[1]  - u2[0]; //Calculate the pulse for esc 4 (front-left - CW)
  */
  esc_1 = throttle - pitch_pd_2  + roll_pd_2; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_2 = throttle + pitch_pd_2  + roll_pd_2; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle + pitch_pd_2  - roll_pd_2; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_4 = throttle - pitch_pd_2  - roll_pd_2; //Calculate the pulse for esc 4 (front-left - CW)


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
      

/*
         Serial.print(esc_1);
    Serial.print(" ");
    Serial.print(esc_2);
    Serial.print(" ");
    Serial.print(esc_3);
    Serial.print(" ");
    Serial.print(esc_4);
    Serial.println();
 */  
      
      while(micros() - loop_timer_pid < 4000);                                      //We wait until 4000us are passed.
  //Serial.println(micros() - loop_timer_pid);
       loop_timer_pid = micros();                                   //Set the timer for the next loop.





  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer_pid;                                     //Calculate the time of the falling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer_pid;                                     //Calculate the time of the falling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer_pid;                                     //Calculate the time of the falling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer_pid;                                     //Calculate the time of the falling edge of the esc-4 pulse.


  record_data();



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

//Channel 5=========================================
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

 //Channel 6=========================================
  if(PINB & B00000010){                                                    
    if(last_channel_6 == 0){                                              
      last_channel_6 = 1;                                                   
      timer_6 = current_time;                                             
    }
  }
  else if(last_channel_6 == 1){                                             
    last_channel_6 = 0;                                                    
    rec6 = current_time - timer_6;                            
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

}


////////////IMU function defination start
void setupmpu() {
  
    Wire.beginTransmission(mpu6050_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(mpu6050_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(gyro_rate_select);                                                          //Set the register bits as 00011000 (2000dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

                                                                              //End the transmission with the gyro

    Wire.beginTransmission(mpu6050_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(acc_rate_select);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();        
 
    Wire.beginTransmission(mpu6050_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();  
   

}


void record_data(){


    Wire.beginTransmission(mpu6050_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                     //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(mpu6050_address,14);                                      //Request 14 bytes from the gyro.
   
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc[0] = ((Wire.read()<<8|Wire.read())>>acc_smt)<<acc_smt;                               //Add the low and high byte to the acc_x variable.
    acc[1] = ((Wire.read()<<8|Wire.read())>>acc_smt)<<acc_smt;                               //Add the low and high byte to the acc_y variable.
    acc[2] = ((Wire.read()<<8|Wire.read())>>acc_smt)<<acc_smt;                               //Add the low and high byte to the acc_z variable.
    Wire.read()<<8|Wire.read();                                        //Add the low and high byte to the temperature variable.
    gyro[0] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.




  if(calibration == 2000){
    gyro[0] -= gyro_cal[0];                                       //Only compensate after the calibration.
    gyro[1] -= gyro_cal[1];                                       //Only compensate after the calibration.
    gyro[2] -= gyro_cal[2];                                       //Only compensate after the calibration.

      for(int i=0;i<3;i++)
  {
    gyro[i]/=gyrolsb;
    acc[i] /=acclsb;
  }
  
  }
 
  else
  {
    gyro_cal[0] += gyro[0];                                       //Only compensate after the calibration.
    gyro_cal[1] += gyro[1];                                       //Only compensate after the calibration.
    gyro_cal[2] += gyro[1];     
     
  
 

  }
}


void angle_calc()
{


float norm = acc[0]*acc[0] + acc[1]*acc[1] + acc[2];

if(norm>1.3225||norm<.7225)
{
  accelw=0;
  gyrow=1;
}
else
{
  accelw=0.004;
  gyrow=0.996;
}

  dtheta[0] = gyrow*gyro[0]*dt*inv_degconvt+ accelw*(atan2(acc[1],acc[2])-roll);
  dtheta[1] = gyrow*gyro[1]*dt*inv_degconvt+ accelw*(atan2(-acc[0],acc[2])-pitch);
  dtheta[2] = 0;


vec_rot();



roll  = atan2(dcm[2][1],dcm[2][2]);
pitch = asin(-dcm[2][0]);
//yaw   = atan2(dcm[1][0],dcm[0][0]);

out[0] = int(roll*degconvert);
out[1] = int(pitch*degconvert);


  
}

void vec_rot()
{
float temp_dcm[3][3];
float dot = 0;

float euler[][3] = {   1         , -dtheta[2] , dtheta[1],
                       dtheta[2], 1         ,  -dtheta[0],
                       -dtheta[1] , dtheta[0],  1         
                   };



for(int i=0;i<3;i++)
{
  for(int j=0;j<3;j++)
  {
    temp_dcm[i][j] = 0;
    
    for(int k=0;k<3;k++)
    temp_dcm[i][j] += dcm[i][k]*euler[k][j];

   
  }
}

for(int i=0;i<3;i++){
for(int j=0;j<3;j++)
  {dcm[i][j] = temp_dcm[i][j];

  }  
}
/*
for(int i=0;i<3;i++)
dot += dcm[0][i]*dcm[1][i];

float dummy[2];
for(int i=0;i<3;i++)
{
dummy[0] = dcm[0][i] - dot*dcm[1][i]/2;
dummy[1] = dcm[1][i] - dot*dcm[0][i]/2;

dcm[0][i] = dummy[0];
dcm[1][i] = dummy[1];
  
}

dcm[2][0] = dcm[0][1]*dcm[1][2] - dcm[1][1]*dcm[0][2];
dcm[2][1] = dcm[1][0]*dcm[0][2] - dcm[0][0]*dcm[1][2];
dcm[2][2] = dcm[0][0]*dcm[1][1] - dcm[1][0]*dcm[0][1];
*/
}



void renorm()
{
float rnorm = dcm[0][0]*dcm[0][0] + dcm[0][1]*dcm[0][1] + dcm[0][2]*dcm[0][2];

for(int i=0;i<3;i++)
dcm[0][i] = .5*(3 - rnorm)*dcm[0][i];

rnorm = dcm[1][0]*dcm[1][0] + dcm[1][1]*dcm[1][1] + dcm[1][2]*dcm[1][2];

for(int i=0;i<3;i++)
dcm[1][i] = .5*(3 - rnorm)*dcm[1][i];

rnorm = dcm[2][0]*dcm[2][0] + dcm[2][1]*dcm[2][1] + dcm[2][2]*dcm[2][2];

for(int i=0;i<3;i++)
dcm[2][i] = .5*(3 - rnorm)*dcm[2][i];
 
}

