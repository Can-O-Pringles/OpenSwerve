#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <cmath>
#include <tgmath.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_I2CDevice.h>
#include <utility/imumaths.h>

//Drivetrain Motor/Servo Declarations
NoU_Motor Drive1(1);
NoU_Motor Turn1(4);
NoU_Motor Drive2(2);
NoU_Motor Turn2(5);
NoU_Motor Drive3(3);
NoU_Motor Turn3(6);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//turn magnitude and the vectors for each individual module in format {{mag1, dir1}, {mag2, dir2}, {mag3, dir3}}
double turnMag = 0.0;
double driveAngle = 0.0;
double driveMag = 0.0;
double drivetrainVectors[3][2] = { { 0, 0 },
                                   { 0, 0 },
                                   { 0, 0 } };

//Gyro variables
double theta;
double headingOffset = 0.0;

//offset from straight ahead
int mod1Offset = 240;  //240
int mod2Offset = 120;  //120
int mod3Offset = 0;    //0

//PID Constants
int lastUpdate = 0;
int deltaT = 0;

double turn1Error = 0;
double turn2Error = 0;
double turn3Error = 0;

double turn1LastError = 1;
double turn2LastError = 1;
double turn3LastError = 1;

double turn1LastTarget = 0;
double turn2LastTarget = 0;
double turn3LastTarget = 0;

const double P = 0.005;
const double D = 0.0001;


const bool AM_DEBUGGING = false;
bool AM_CALIBRATING = false;

void setup() {
  //Basic Board Setup
  NoU3.begin();
  NoU3.setServiceLight(LIGHT_DISABLED);

  //BNO Setup
  if (!bno.begin()) {
    while (1)
      Serial.println("BNO055 not detected. Check wiring or I2C pins!");
    ;  // Stop program if sensor not found
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  delay(100);
  bno.setMode((adafruit_bno055_opmode_t)0x08);  // 0x8C = IMU_PLUS mode

  //Motor configs
  Turn1.beginEncoder();
  Turn2.beginEncoder();
  Turn3.beginEncoder();
  Turn1.setMinimumOutput(0.65);
  Turn2.setMinimumOutput(0.65);
  Turn3.setMinimumOutput(0.65);

  Drive1.setMinimumOutput(0.7);
  Drive2.setMinimumOutput(0.7);
  Drive3.setMinimumOutput(0.7);
  Drive1.setExponent(2);
  Drive2.setExponent(2);
  Drive3.setExponent(2);

  PestoLink.begin("Zorp");
  Serial.begin(115200);
}

void loop() {

  // Set up BNO055 and its variables
  // ACCEL = m/s^2
  // ORRIENTATION = Degrees
  // ROTATION SPEED = Radians/Second
  sensors_event_t event;
  bno.getEvent(&event);
  theta = event.orientation.x - headingOffset;

  // get magnitude and direction and assign to drivetrainVectors array, add offsets
  // set turn vector magnitude
  // set RSL based on whether a gamepad is connected
  // ANGLES IN RADIANS
  if (PestoLink.update()) {

    driveAngle = atan2(PestoLink.getAxis(1), PestoLink.getAxis(0));
    driveMag = sqrt(pow(PestoLink.getAxis(1), 2) + pow(PestoLink.getAxis(0), 2));
    turnMag = PestoLink.getAxis(2);

    drivetrainVectors[0][0] = driveMag;
    drivetrainVectors[0][1] = driveAngle + (mod1Offset * (PI / 180));

    drivetrainVectors[1][0] = driveMag;
    drivetrainVectors[1][1] = driveAngle + (mod2Offset * (PI / 180));

    drivetrainVectors[2][0] = driveMag;
    drivetrainVectors[2][1] = driveAngle + (mod3Offset * (PI / 180));

    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    NoU3.setServiceLight(LIGHT_DISABLED);
  }
  //Offset debug
  if (AM_DEBUGGING) {
    Serial.println("Robot Angle");
    Serial.println(theta);
    Serial.println("Mod 1 Pos:");
    Serial.println(Turn1.getPosition() * (40.0 / 9));
    Serial.println("Mod 2 Pos:");
    Serial.println(Turn2.getPosition() * (40.0 / 9));
    Serial.println("Mod 3 Pos:");
    Serial.println(Turn3.getPosition() * (40.0 / 9));
    if (PestoLink.buttonHeld(0))
      mod1Offset++;
    Serial.println("Mod 1 Offset:");
    Serial.println(mod1Offset);
    if (PestoLink.buttonHeld(1))
      mod2Offset++;
    Serial.println("Mod 2 Offset:");
    Serial.println(mod2Offset);
    if (PestoLink.buttonHeld(2))
      mod3Offset++;
    Serial.println("Mod 3 Offset:");
    Serial.println(mod3Offset);
  }
    if (AM_CALIBRATING) {
    uint8_t sys, gyro, accel, mag;
    while (AM_CALIBRATING) {
      bno.getCalibration(&sys, &gyro, &accel, &mag);
      Serial.print("Calib: SYS=");
      Serial.print(sys);
      Serial.print(" GYRO=");
      Serial.print(gyro);
      Serial.print(" ACCEL=");
      Serial.print(accel);
      Serial.print(" MAG=");
      Serial.println(mag);
      if (sys == 3 && gyro == 3 && accel == 3 && mag == 3) {
        AM_CALIBRATING = false;
      }
    }
  }

  //Heading Offset Control

  if (PestoLink.update() && PestoLink.buttonHeld(3)) {
    bno.getEvent(&event);
    headingOffset = event.orientation.x;
  }


  //Vector Addition
  //Finds the component form of the current drive vector on the unit circle for each individual module
  //Uses the fact that turn vector is always 0 degrees, adds it to x coordinate.
  //reconverts back into magnitude and directon form
  //ANGLES IN DEGREES

  double xCord1 = drivetrainVectors[0][0] * cos(drivetrainVectors[0][1]) + turnMag;
  double yCord1 = drivetrainVectors[0][0] * sin(drivetrainVectors[0][1]);

  double xCord2 = drivetrainVectors[1][0] * cos(drivetrainVectors[1][1]) + turnMag;
  double yCord2 = drivetrainVectors[1][0] * sin(drivetrainVectors[1][1]);

  double xCord3 = drivetrainVectors[2][0] * cos(drivetrainVectors[2][1]) + turnMag;
  double yCord3 = drivetrainVectors[2][0] * sin(drivetrainVectors[2][1]);

  double cordArray[3] = { abs(xCord1), abs(xCord2), abs(xCord3) };

  //Find max x coordinate (since only adding to x)
  double massiveCord = 0.0;
  for (int i = 0; i < 2; i++) {
    if (cordArray[i] > massiveCord) {
      massiveCord = cordArray[i];
    }
  }
  //scales all components to the one with the largest magnitude
  if (massiveCord > 1) {
    xCord1 /= massiveCord;
    yCord1 /= massiveCord;
    xCord2 /= massiveCord;
    yCord2 /= massiveCord;
    xCord3 /= massiveCord;
    yCord3 /= massiveCord;
  }

  //component form --> magnitude and direction form as semifinal values
  drivetrainVectors[0][1] = atan2(yCord1, xCord1) * (180 / PI);
  drivetrainVectors[0][0] = sqrt(pow(xCord1, 2) + pow(yCord1, 2));
  drivetrainVectors[1][1] = atan2(yCord2, xCord2) * (180 / PI);
  drivetrainVectors[1][0] = sqrt(pow(xCord2, 2) + pow(yCord2, 2));
  drivetrainVectors[2][1] = atan2(yCord3, xCord3) * (180 / PI);
  drivetrainVectors[2][0] = sqrt(pow(xCord3, 2) + pow(yCord3, 2));

  drivetrainVectors[0][1] -= theta;
  drivetrainVectors[1][1] -= theta;
  drivetrainVectors[2][1] -= theta;


  //set drivetrain + deadzone
  turn1Error = (Turn1.getPosition() + (drivetrainVectors[0][1] * (40.0 / 9)));
  turn2Error = (Turn2.getPosition() - (drivetrainVectors[1][1] * (40.0 / 9)));
  turn3Error = (Turn3.getPosition() - (drivetrainVectors[2][1] * (40.0 / 9)));
  deltaT = millis() - lastUpdate;



  //PID Calculation
  if (PestoLink.update() && (abs(PestoLink.getAxis(0)) + abs(PestoLink.getAxis(1)) + abs(PestoLink.getAxis(2))) > 0.02) {
    Turn1.set((P * turn1Error) + (D * ((turn1Error - turn1LastError) / (deltaT / 1000.0))));
    Drive1.set(drivetrainVectors[0][0]);
    Turn2.set((P * turn2Error) + (D * ((turn2Error - turn2LastError) / (deltaT / 1000.0))));
    Drive2.set(drivetrainVectors[1][0]);
    Turn3.set((P * turn3Error) + (D * ((turn3Error - turn3LastError) / (deltaT / 1000.0))));
    Drive3.set(-drivetrainVectors[2][0]);

    turn1LastError = turn1Error;
    turn2LastError = turn2Error;
    turn3LastError = turn3Error;
    turn1LastTarget = drivetrainVectors[0][1];
    turn2LastTarget = drivetrainVectors[1][1];
    turn3LastTarget = drivetrainVectors[2][1];
    lastUpdate = millis();


  } else {
    Turn1.set(0);
    Drive1.set(0);
    Turn2.set(0);
    Drive2.set(0);
    Turn3.set(0);
    Drive3.set(0);
  }

  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);

  PestoLink.update();
  NoU3.updateServiceLight();
}
