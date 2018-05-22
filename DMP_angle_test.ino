#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h> 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


#define INTERRUPT_PIN 2  
#define LED_PIN 13 
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




float pid_p_gain_roll = 0;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.0;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)    

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0.0;                //Gain setting for the pitch P-controller. //4.0           
float pid_i_gain_yaw = 0.0;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)



byte last_channel_3,last_channel_6,last_channel_5;

volatile int  receiver_input_channel_3,receiver_input_channel_6,receiver_input_channel_5;

int esc_1, esc_2, esc_3, esc_4;

int cal_int, start, gyro_address;

int throttle;





unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long  timer_3, timer_6, current_time,timer_5;
unsigned long loop_timer;

double gyro_pitch, gyro_roll, gyro_yaw;

float pid_error_temp=0;
float pid_i_mem_roll=0, pid_roll_setpoint=0, pid_output_roll, pid_last_roll_d_error=0;
float pid_i_mem_pitch=0, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error=0;
float pid_i_mem_yaw=0, pid_yaw_setpoint=0,  pid_output_yaw, pid_last_yaw_d_error=0;

float yaw,roll,pitch;

float roll_app=0,pitch_app=0;



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup()
{Serial.begin(115200);

 DDRD |= B11110000;                                                        //Configure digital port 4, 5, 6 and 7 as output.
 DDRB |= B00100000;                                                        //Configure digital port 12 and 13 as output.
  
  digitalWrite(13,HIGH);
  
  
  



for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
    if(cal_int % 50 == 0)digitalWrite(13, !digitalRead(13));
}

setupmpu();
    





 PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
PCMSK0 |= (1 << PCINT4);
PCMSK0 |= (1 << PCINT3); 
       loop_timer = micros();                                                    //Set the timer for the next loop.
digitalWrite(13,LOW); 
                                                  
start=0;


  
}

void loop()
{

Serial.println("Started");

pid_p_gain_roll = 0.05*receiver_input_channel_5-50;

pid_p_gain_pitch = pid_p_gain_roll;




  calcang();  





if((receiver_input_channel_6>1500)&&throttle>1010){
throttle = receiver_input_channel_3; 



                               
    esc_1 = throttle + pid_output_pitch - pid_output_roll;  //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle - pid_output_pitch - pid_output_roll; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle - pid_output_pitch + pid_output_roll;  //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle + pid_output_pitch + pid_output_roll;  //Calculate the pulse for esc 4 (front-left - CW)


  esc_1 = constrain(esc_1,1000,2000);           
  esc_2 = constrain(esc_2,1000,2000);           
  esc_3 = constrain(esc_3,1000,2000);           
  esc_4 = constrain(esc_4,1000,2000);  
}
else
{
  esc_1 = 1000;
esc_2 = 1000;
esc_3 = 1000;
esc_4 = 1000;
}

/*Serial.print("ESC 1  ");
Serial.println(esc_1);
Serial.print("ESC 2 ");
Serial.println(esc_2);
Serial.print("ESC 3  ");
Serial.println(esc_3);
Serial.print("ESC 4  ");
Serial.println(esc_4);

*/

 



Serial.print(pitch);
Serial.print(" ");
Serial.println(roll);
          
 
           

  while(micros() - loop_timer < 5000);                                      //We wait until 4000us are passed.
       loop_timer = micros(); 

  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  
  timer_channel_4 = esc_4 + loop_timer;   //Calculate the time of the falling edge of the esc-4 pulse.



PORTD |= B11110000;         
  
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

  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input_channel_3 = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }

  //CHANNEL 6
 if(PINB & B00010000 ){                                                    //Is input 10 high?
    if(last_channel_6 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_6 = 1;                                                   //Remember current input state.
      timer_6 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_6 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_6 = 0;                                                     //Remember current input state.
    receiver_input_channel_6 = current_time - timer_6;                             //Channel 3 is current_time - timer_3.

  }


if(PINB & B00001000 ){                                                    //Is input 10 high?
    if(last_channel_5 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_5 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input_channel_5 = current_time - timer_5;                             //Channel 3 is current_time - timer_3.

  }


}











void setupmpu() {
   
    pinMode(LED_PIN, OUTPUT);
    
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
    mpu.setXGyroOffset(33);
    mpu.setYGyroOffset(43);
    mpu.setZGyroOffset(41);
    mpu.setZAccelOffset(1452); // 1688 factory default for my test chip


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
    digitalWrite(13,LOW);
    

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


