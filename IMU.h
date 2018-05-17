

#define mpu6050_address 0x68

extern double dt;

extern float roll,pitch,yaw;

extern float gyro[3],acc[3];

extern float gyro_cal[3];

//////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//#define gyro250  0x00

//#define gyro500  0x08

//#define gyro1000  0x10

#define gyro2000  0x18


#define acc2g 0x00

//#define acc4g 0x08

//#define acc6g 0x10

//#define acc8g 0x18


#ifdef gyro250

const float gyrolsb = 131.0;
const int gyro_rate_select = gyro250;
#endif

#ifdef gyro500
const float gyrolsb = 65.5;
const int gyro_rate_select = gyro500;
#endif


#ifdef gyro1000
const float gyrolsb = 32.8;
const int gyro_rate_select = gyro1000;
#endif

#ifdef gyro2000
const float gyrolsb = 16.4;
const int gyro_rate_select = gyro2000;
#endif


#ifdef acc2g

const float acclsb = 16384.0;
const int acc_rate_select = acc2g;
#endif

#ifdef acc4g
const float acclsb = 8192.0;
const int acc_rate_select = acc4g;
#endif


#ifdef acc6g
const float acclsb = 4096.0;
const int acc_rate_select = acc6g;
#endif

#ifdef acc8g
const float acclsb = 2048.0;
const int acc_rate_select = acc8g;
#endif

//////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

void record_data();

void angle_calc();


