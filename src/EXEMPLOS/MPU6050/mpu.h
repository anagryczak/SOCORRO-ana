// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// define output readings
#define OUTPUT_READABLE_QUATERNION   // quaternion is the standart for imu readings
#define OUTPUT_READABLE_YAWPITCHROLL // get yaw is enought for odometry

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 accel;   // [x, y, z]            accel sensor measurements
VectorInt16 gyro;    // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float quaternion[3];
float acceleration[3];
float velocity[3];

float orientationCovariance[9];
float angularCovariance[9];
float linearCovariance[9];

int16_t ax, ay, az;
int16_t gx, gy, gz;

bool imu_setup()
{   

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    return devStatus;
}

float* imu_get_ypr()
{

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // // * display Euler angles in radians
        // Serial.print("ypr\t");
        // Serial.print(ypr[0], 5); // yaw
        // Serial.print("\t");
        // Serial.print(ypr[1],5);       //pitch
        // Serial.print("\t");
        // Serial.print(ypr[2],5);       //roll
        // Serial.println("\t");
    }
    return ypr;
}

float* orientation()
{

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {                                         // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer); // quaternion
    }

    quaternion[0] = q.x;
    quaternion[1] = q.y;
    quaternion[2] = q.z;
    quaternion[3] = q.w;

    // //* display Quaternion in radians
    // Serial.print("quaternion angles -> \t");
    // Serial.print("x: ");
    // Serial.print(quaternion[0], 5);     
    // Serial.print("\t");

    // Serial.print("y: ");
    // Serial.print(quaternion[1],5);      
    // Serial.print("\t");

    // Serial.print("z: ");
    // Serial.print(quaternion[2],5);     
    // Serial.print("\t");

    // Serial.print("w: ");
    // Serial.print(quaternion[3],5);       
    // Serial.println("\t");

    return quaternion;
}

float* linear_acceleration()
{
                                   
    mpu.dmpGetAccel(&accel, fifoBuffer);
    

    // for each range of the accelerometer, we need to divide the raw value by the following values: 
    // 2g  -> 16384.0
    //! 4g  -> 8192.0
    // 8g  -> 4096.0
    // 16g -> 2048.0 

    // By default, accel is in arbitrary units with a scale of 16384/1g.
    // Per http://www.ros.org/reps/rep-0103.html
    // and http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
    // should be in m/s^2.
    // 1g = 9.80665 m/s^2, so we go arbitrary -> g -> m/s^s

    acceleration[0] = accel.x * 1/8192.0 * 9.80665;
    acceleration[1] = accel.y * 1/8192.0 * 9.80665;
    acceleration[2] = accel.z * 1/8192.0 * 9.80665;

    //* display linear acceleration in m/s²
    // Serial.print("linear acceleration -> \t");
    // Serial.print("x: ");
    // Serial.print(acceleration[0], 5);     
    // Serial.print("\t");

    // Serial.print("y: ");
    // Serial.print(acceleration[1],5);      
    // Serial.print("\t");

    // Serial.print("z: ");
    // Serial.print(acceleration[2],5);     
    // Serial.println("\t");

    return acceleration; 
}

float* angular_velocity()
{

    mpu.dmpGetGyro(&gyro, fifoBuffer);

    // for each range of the accelerometer, we need to divide the raw value by the following values: 
    // 250°/s  -> 131.0
    // 500°/s  -> 65.5
    // 1000°/s -> 32.8
    // 2000°/s -> 16.4 

    velocity[0] = gyro.x * 1/16.4 * 3.141592/180 ;
    velocity[1] = gyro.y * 1/16.4 * 3.141592/180 ;
    velocity[2] = gyro.z * 1/16.4 * 3.141592/180 ;

    //* display angular velocity in rad/s 
    // Serial.print("angular velocity -> \t");
        
    // Serial.print("x: ");
    // Serial.print(velocity[0],5);      
    // Serial.print("\t");

    // Serial.print("y: ");
    // Serial.print(velocity[1],5);       
    // Serial.print("\t");

    // Serial.print("z: ");
    // Serial.print(velocity[2], 5); 
    // Serial.println("\t");

    return velocity;
}

float* orientation_covariance() {

        for (int i = 0; i < 9; i++) {
            orientationCovariance[i] = 0.0; 
        }

    return orientationCovariance; 

}

float* linear_acceleration_covariance() {


        for (int i = 0; i < 9; i++) {
            linearCovariance[i] = 0.0; 
        }

    return linearCovariance; 

}

float* angular_velocity_covariance() {

        for (int i = 0; i < 9; i++) {
            angularCovariance[i] = 0.0; 
        }

    return angularCovariance; 

}