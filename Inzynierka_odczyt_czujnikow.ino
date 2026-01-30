#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define MPU_quantity 3
#define TCAADDR 0x70

MPU6050 mpu[MPU_quantity];

bool dmpReady = false;   // set true if DMP init was successful
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64];  // FIFO storage buffer
uint8_t mput;
// orientation/motion vars
Quaternion q;  // [w, x, y, z]         quaternion container


void MUXselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void setup() {

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial)
    ;

  mput = 0;

  while (mput < MPU_quantity) {
    MUXselect(mput);
    // initialize device
    Serial.print(F("Initializing I2C device: "));
    Serial.println(mput);
    mpu[mput].initialize();

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu[mput].dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    if (mput == 0) {
      mpu[mput].setXGyroOffset(98);
      mpu[mput].setYGyroOffset(-117);
      mpu[mput].setZGyroOffset(25);
      mpu[mput].setZAccelOffset(9615);
    } else if (mput == 1) {
      mpu[mput].setXGyroOffset(-206);
      mpu[mput].setYGyroOffset(-108);
      mpu[mput].setZGyroOffset(48);
      mpu[mput].setZAccelOffset(8979);
    } else if (mput == 2) {
      mpu[mput].setXGyroOffset(-206);
      mpu[mput].setYGyroOffset(-108);
      mpu[mput].setZGyroOffset(48);
      mpu[mput].setZAccelOffset(8979);
    }

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu[mput].CalibrateAccel(6);
      mpu[mput].CalibrateGyro(6);
      mpu[mput].PrintActiveOffsets();

      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu[mput].setDMPEnabled(true);

      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;


    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
    delay(100);
    mput++;
  }
  mput = 0;
  // configure LED for output
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
/*
void loop() {
  MUXselect(mput);
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu[mput].dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    mpu[mput].dmpGetQuaternion(&q, fifoBuffer);
    Serial.print(q.w, 4);
    Serial.print(";");
    Serial.print(q.x, 4);
    Serial.print(";");
    Serial.print(q.y, 4);
    Serial.print(";");
    Serial.print(q.z, 4);
    if (mput != MPU_quantity -1) {
      Serial.print("|");
      mput++;
    }else{
      Serial.println();
    mput = 0;
    }
  }
  delay(10);
}
*/
void loop() {
  static char message[256];
  static uint16_t idx = 0;

  char fw[12], fx[12], fy[12], fz[12];

  MUXselect(mput);
  if (!dmpReady) return;
  if (mpu[mput].dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu[mput].dmpGetQuaternion(&q, fifoBuffer);
    // float → char[]
    dtostrf(q.w, 0, 4, fw);
    dtostrf(q.x, 0, 4, fx);
    dtostrf(q.y, 0, 4, fy);
    dtostrf(q.z, 0, 4, fz);
    idx += snprintf(message + idx, sizeof(message) - idx, "%s;%s;%s;%s", fw, fx, fy, fz);
    if (mput != MPU_quantity - 1) {
      idx += snprintf(message + idx, sizeof(message) - idx, "|");
      mput++;
    } else {
      Serial.println(message);  // ⬅ wysyłka CAŁOŚCI
      idx = 0;
      message[0] = '\0';
      mput = 0;
    }
  }
  delay(10);
}
