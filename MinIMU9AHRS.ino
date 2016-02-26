#include <RNAControl.h>

#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

int values_elevator[5];
int values_aileron[5];
int values_rudder[5];

//Leitura PPM
unsigned long t_aileron;
unsigned long t_throttle;
unsigned long t_elevator;
unsigned long t_rudder;
unsigned long t_aux;

#define PIN_CH_AILERON   19
#define PIN_CH_ELEVATOR  2
#define PIN_CH_THROTTLE  18
#define PIN_CH_RUDDER    3
#define PIN_CH_AUX       9

#define PIN_LED_RUN      44
#define PIN_LED_RECORD   38

//Atuadores
#define PIN_AILERON   7
#define PIN_ELEVATOR  5
#define PIN_THROTTLE  4
#define PIN_RUDDER    6
Servo servo_aileron;
Servo servo_elevator;
Servo servo_throttle;
Servo servo_rudder;

int SENSOR_SIGN[9] = {1, -1, -1, -1, 1, 1, 1, -1, -1};

#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long rna_timer = 0;
long rna_timer_old = 0;

long timer = 0; //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; //Omega Integrator
float Omega[3] = {0, 0, 0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = {
  {
    1, 0, 0
  }
  , {
    0, 1, 0
  }
  , {
    0, 0, 1
  }
};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here


float Temporary_Matrix[3][3] = {
  {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
};

bool print_data;
int it;


File myFile;
RNAControl* _rnaControl;

void setup()
{
  it = 0;
  print_data = false;
  Serial.begin(115200);

  _rnaControl = new RNAControl();

  //Leitura PPM
  Serial.println("Configurando leitura de dados do RX");
  pinMode(PIN_CH_AILERON, INPUT);
  pinMode(PIN_CH_THROTTLE, INPUT);
  pinMode(PIN_CH_ELEVATOR, INPUT);
  pinMode(PIN_CH_RUDDER, INPUT);
  pinMode(PIN_CH_AUX, INPUT);

  pinMode(PIN_LED_RUN, OUTPUT);
  pinMode(PIN_LED_RECORD, OUTPUT);


  servo_aileron.attach(PIN_AILERON);
  servo_throttle.attach(PIN_THROTTLE);
  servo_elevator.attach(PIN_ELEVATOR);
  servo_rudder.attach(PIN_RUDDER);

  pinMode (STATUS_LED, OUTPUT); // Status LED

  I2C_Init();

  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  digitalWrite(STATUS_LED, LOW);
  digitalWrite(PIN_LED_RUN, LOW);
  digitalWrite(PIN_LED_RECORD, LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for (int i = 0; i < 32; i++) // We take some readings...
  {
    Read_Gyro();
    Read_Accel();
    for (int y = 0; y < 6; y++) // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
  }

  for (int y = 0; y < 6; y++)
    AN_OFFSET[y] = AN_OFFSET[y] / 32;

  AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for (int y = 0; y < 6; y++)
    Serial.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED, HIGH);
  digitalWrite(PIN_LED_RUN, HIGH);

  timer = millis();
  delay(20);
  counter = 0;

  pinMode(53, OUTPUT);
/*
  // Start the Arduino hardware serial port at 9600 baud
  Serial.print("Initializing SD card...");
  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  */
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  
}

void loop() //Main Loop
{
 
  if ((millis() - timer) >= 20) // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer = millis();
    //Serial.println(timer - timer_old);
    if (timer > timer_old)
      G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;

    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter = 0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();

    //Leitura do radio
    t_aileron = pulseIn(PIN_CH_AILERON, HIGH, 100000);
    t_throttle = pulseIn(PIN_CH_THROTTLE, HIGH, 100000);
    t_elevator = pulseIn(PIN_CH_ELEVATOR, HIGH, 100000);
    t_rudder = pulseIn(PIN_CH_RUDDER, HIGH, 100000);
    t_aux = pulseIn(PIN_CH_AUX, HIGH, 100000);

    int c_aileron = map(t_aileron, 1100, 1850, 0, 180);
    int c_throttle = map(t_throttle, 1100, 1830, 0, 180);
    int c_elevator = map(t_elevator, 1100, 1800, 0, 180);
    int c_rudder = map(t_rudder, 1150, 1900, 0, 180);

    c_aileron = c_aileron < 0 ? 0 : c_aileron;
    c_throttle = c_throttle < 0 ? 0 : c_throttle;
    c_elevator = c_elevator < 0 ? 0 : c_elevator;
    c_rudder = c_rudder < 0 ? 0 : c_rudder;

    //Serial.print("{");
    /*
    Serial.print((int)ToDeg(roll));
    Serial.print(", ");
    Serial.print((int)ToDeg(pitch));
    Serial.print(", ");
    Serial.print((int)ToDeg(yaw));
    Serial.print(", ");
    Serial.print(c_aileron);
    Serial.print(", ");
    Serial.print(c_throttle);
    Serial.print(", ");
    Serial.print(c_elevator);
    Serial.print(", ");
    Serial.print(c_rudder);
    */
    /*Serial.print(" -> AIL:  ");
    Serial.print(t_aileron);
    Serial.print(" -> THR:  ");
    Serial.print(t_throttle);
    //Serial.print(" -> RUDDER:  ");
    //Serial.print(t_rudder);
    Serial.print(" -> ELE:  ");
    Serial.print(t_elevator);
    Serial.print(" -> AUX:  ");
    Serial.print(t_aux);*/
    
    /*
    myFile = SD.open("test.txt", FILE_WRITE);
    if (myFile) {
      if(t_aux < 1500) {
        //Serial.print("Gravando dados no SD");
        myFile.print((int)ToDeg(roll));
        myFile.print(", ");
        myFile.print((int)ToDeg(pitch));
        myFile.println(";");
        myFile.close();
        Serial.print(" RECORDING");
        digitalWrite(PIN_LED_RECORD, HIGH);
      }
      else {
        digitalWrite(PIN_LED_RECORD, LOW);
      }

    } else {
      digitalWrite(PIN_LED_RECORD, LOW);
      Serial.println("error opening test.txt");
    }

    servo_aileron.write(c_aileron);
    servo_throttle.write(c_throttle);
    servo_elevator.write(c_elevator);
    servo_rudder.write(c_rudder);
    */
    //Serial.println("},");

    if(t_aux < 1500) {
      rna_timer_old = rna_timer;
      rna_timer = millis();
      double _roll = _map(ToDeg(roll), -90.0, 90.0, 0.0, 1.0);
      double _pitch = _map(ToDeg(pitch), -90.0, 90.0, 0.0, 1.0);

     _rnaControl->setRoll(_roll);
     _rnaControl->setPitch(_pitch);
     _rnaControl->run();
     
     double aileronAngle = _rnaControl->getAileronAngle();
     double elevatorAngle = _rnaControl->getElevatorAngle();
     double throttle = _rnaControl->getThrottle();

      int cmd_aileron = _map(_map(aileronAngle, 0.0, 1.0, 0.0, 180.0), 0, 180, 180, 0);
      int cmd_throttle = _map(throttle, 0.0, 1.0, 0, 180.0);
      int cmd_elevator = _map(elevatorAngle, 0.0, 1.0, 0, 180.0);

      Serial.print("Sensors: R ");
      Serial.print(_roll);
      Serial.print(" P ");
      Serial.print(_pitch);
      Serial.print("        Rna: A ");
      Serial.print(aileronAngle);
      Serial.print(" T ");
      Serial.print(throttle);
      Serial.print(" E ");
      Serial.print(elevatorAngle);
      Serial.print("        Cmds: A ");
      Serial.print(cmd_aileron);
      Serial.print(" T ");
      Serial.print(cmd_throttle);
      Serial.print(" E ");
      Serial.print(cmd_elevator);
      Serial.println(" ");

      servo_aileron.write(cmd_aileron);
      servo_throttle.write(cmd_throttle);
      servo_elevator.write(cmd_elevator);
      servo_rudder.write(90);
    }
    else {
      servo_elevator.write(c_elevator);
      servo_rudder.write(c_rudder);
      servo_aileron.write(c_aileron);
      servo_throttle.write(c_throttle);
    }

  }

}

double _map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}






