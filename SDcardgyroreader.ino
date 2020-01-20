/*
 *      --INSTRUCTIONS--
 * Set to:
 * Digital pin2 - Red Led
 * Digital pin3 - Green Led
 * Digital pin4 - EMPTY
 * Digital pin5 - Right wing servo
 * Digital pin6 - Left wing servo
 * Digital pin7 - Elevator servo
 * Digital pin8 - Back turn servo
 * 
 */
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

#define ARRAYSIZE 10

File myFile;
Servo servo_rightwing, servo_leftwing, servo_elevator, servo_turn;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
String filename[ARRAYSIZE] = { "file1.txt", "file1.txt", "file3.txt", "file4.txt", "file5.txt", "file6.txt", "file7.txt", "file8.txt", "file9.txt", "file10.txt" };

int cnt_read_time = 0;
int current_time = 0;
int next_time = 0;

int LED_RED = 2;
int LED_GREEN = 3;

float stv_ac = 4096.0; //Choosen accelerometer sensitivity in order to have +-8G scale range (from MPU6050 datasheet)

void setup(){
  int cnt = 0;

  analogReference(EXTERNAL);   // Use the external analog reference  
  servo_rightwing.attach(5); //Attach the servos to a digital port
  servo_leftwing.attach(6);
  servo_elevator.attach(7);
  servo_turn.attach(8);
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(10, OUTPUT);

  // initialize digital pin RED_LED and GREEN_LED as an output.
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

   
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1){
      //RED LED ERROR BLINKING
      red_led_error1();
    }
  }
  Serial.println("initialization done.");

  // Check to see if the file exists:
  while(cnt<11){
    if (SD.exists(filename[cnt])) {
      Serial.println("FILE exists.");
      cnt++;
    } else {
      Serial.println("FILE doesn't exist.");
      // open a new file
      Serial.println("Creating...");
      myFile = SD.open(filename[cnt], FILE_WRITE);
      //myFile.close();
      cnt=12;
    }
  }

  if(cnt=11){
    while(1){
      //RED LED ERROR BLINKING
      red_led_error2();
    }
  }

  //GREEN LED INDICATING SYSTEM IS OPERATIONAL
  all_systems_ready();
}

void loop(){
  if(cnt_read_time=2){
    delay(1000);
    Serial.println(millis()/1000);
    myFile.println("-------------------------------------");
    next_time = millis()/1000;
    current_time = next_time; //Control. Do nothing
    myFile.println("Second nº: "); myFile.print(next_time);
    cnt_read_time=0;
  }
  if(current_time = next_time && cnt_read_time<2){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read() / stv_ac;  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read() / stv_ac;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read() / stv_ac;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    Serial.print("AcX = "); Serial.print(AcX);
    myFile.println("AcX = "); myFile.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    myFile.println(" | AcY = "); myFile.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    myFile.println(" | AcZ = "); myFile.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    myFile.println(" | Tmp = "); myFile.print(Tmp/340.00+36.53);
    Serial.print(" | GyX = "); Serial.print(GyX);
    myFile.println(" | GcX = "); myFile.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    myFile.println(" | GcY = "); myFile.print(GyY);
    Serial.print(" | GyZ = "); Serial.print(GyZ);
    myFile.println(" | GcZ = "); myFile.print(GyZ);
    Serial.print(" | right wing servo"); Serial.print(servo_rightwing.read());
    myFile.println(" Right wing servo = "); myFile.print(servo_rightwing.read());
    Serial.print(" | left wing servo"); Serial.print(servo_leftwing.read());
    myFile.println(" left wing servo = "); myFile.print(servo_leftwing.read());
    Serial.print(" | elevator wing servo"); Serial.print(servo_elevator.read());
    myFile.println(" Elevator wing servo = "); myFile.print(servo_elevator.read());
    Serial.print(" | turn wing servo"); Serial.print(servo_turn.read());
    myFile.println(" Turn wing servo = "); myFile.print(servo_turn.read());

    cnt_read_time++;
  }

  //Calculation of roll and pitch angles from the accelerometer data
  //MPU6050 should be mounted in the aircraft in the manner that YY axis corresponds to roll and XX to pitch (page 21 of datasheet)
  AcX_angle = atan(AcX / (sqrt(pow(AcY,2) + pow(AcZ,2))) * 180 / PI) //Pitch angle - error should be added to equation
  AcY_angle = atan(AcY / (sqrt(pow(AcX,2) + pow(AcZ,2))) * 180 / PI) //Roll angle - error should be added to equation
  AcZ_angle = atan(AcZ / (sqrt(pow(AcY,2) + pow(AcZ,2))) * 180 / PI) //Yaw angle - error should be added to equation
}

/*
 * Function: red_led_error1
 * Purpose: Alert for non-initialized SD card
 */
void red_led_error1(){
  digitalWrite(LED_RED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_RED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

/*
 * Function: red_led_error2
 * Purpose: Alert for all files names are being used
 */
void red_led_error2(){
  digitalWrite(LED_RED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_RED, LOW);    // turn the LED off by making the voltage LOW
  delay(2000);                       // wait for a second
}

/*
 * Function: all_systems_ready
 * Purpose: Alert that arduino has been initialized sucessfully
 */
void all_systems_ready(){
  digitalWrite(LED_GREEN, HIGH);   // turn the LED on (HIGH is the voltage level)
}
