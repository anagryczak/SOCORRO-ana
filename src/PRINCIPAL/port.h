// SENSOR IR

#define S1 14
#define S2 27
#define S3 26
#define S4 25
#define S5 33
#define S6 32
#define S7 35
#define S8 34


// SENSOR LATERAL 

#define Sensor_L 23
#define Sensor_R 13

// PONTE H 

#define PWMA 19
#define PWMB 18

//ENCODER 

#define ENCODERA_L 16
#define ENCODERB_L 17
#define ENCODERA_R 2
#define ENCODERB_R 4

// PWM

#define DEFAULT_LEDC_FREQ 800
#define _resolution 10
#define A_channel 0
#define B_channel 1

double KP = 0.1;
double KD = 0;
double KI = 0;
double VEL = 60;

bool iniciar = LOW;
bool calibrar = LOW;

bool readR = HIGH;
int contadorR = 0;
bool anteriorreadR = HIGH;

bool readL = HIGH;
int contadorL = 0;
bool anteriorreadL = HIGH;