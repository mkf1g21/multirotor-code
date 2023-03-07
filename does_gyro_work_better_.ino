#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <SD.h>

//#define DEBUG
#define SHOW_VALS
#define WRITE_TO_CARD

//---------------------servos--------------------

Servo pitch_servo;
Servo roll_servo;

// pin to lock the gimbal to the axis
const int pause_pin = 5; 

// pin for the servos
const int pitch_pin = 9;
const int roll_pin = 10;

//-----------------------mpu------------------------

Adafruit_MPU6050 mpu;

// stores the position of the pitch, roll and yaw axes
float pitch = 0;
float roll = 0;
float yaw = 0;

// needed to find how far the whole thing has moved
float last_time = 0;
float last_x;
float last_z;
float last_y;

// how far off 0 the angle has to be before the servos do anything
const float error = 0.1; 

// -------------------SD card--------------------

// reminder for which pins go where
const int MOSI_reminder = 11; // to DI
const int MISO_reminder = 12; // to DO
const int CLK_reminder = 13; // to CLK


#ifdef WRITE_TO_CARD

// chip select
const int CS = 4;

// how many bytes to allocate for the filename, this is the maximum for the standard
const int filename_bytes = 12;


char* filename;

Sd2Card card;
#endif

void setup() {

  // need to use wire for the mpu because its all a bit sussy
  Wire.begin();
  Wire.setClock(10000);

  // lets the mpu restart if something goes wrong. REALLY not ideal
  Wire.setWireTimeout(10000);

  
  Serial.begin(19200);
  Serial.println("started");

  // idk why mpu.begin needs these
  uint8_t addr = 0x68;
  int32_t id = 1;

  // only carry on if th mpu can be started
  if (!mpu.begin(addr, &Wire, id)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("started MPU");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  setup_servos();
  delay(100);
  init_card(CS);
}


void loop() {

  // these are used in both branches, so may as well define them here
  float elapsed = 0;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  if (!digitalRead(pause_pin)) {
    // this is the branch to take if the gimbal isnt locked to the body
    
    
    #ifdef DEBUG
    Serial.println("got from MPU");
    #endif

    
    float now = millis();

    // should really be 1000 instead of 100 but this works well enough
    elapsed = (now - last_time)/100;

    // calculate how far it has moved since the last time 
    // the names don't necessarily reflect reality
    roll += ((a.gyro.x-last_x) * elapsed);
    yaw += ((a.gyro.y-last_y) * elapsed);
    pitch += ((a.gyro.z-last_z) * elapsed);

    #ifdef DEBUG
    Serial.println("got the offsets");
    #endif
    

    // block to move the pitch servo
    if (pitch > error){
      // get the last position written and then increase it by 1. this is where PID goes
      int temp = pitch_servo.read();
      pitch_servo.write(temp + 1);
    } else if (pitch < -error){
      // same as the other one
      int temp = pitch_servo.read();
      pitch_servo.write(temp - 1);
    }
    
    #ifdef DEBUG
    Serial.println("moved yaw");
    #endif

    // same as above for the other servo
    if (roll > error){
      int temp = roll_servo.read();
      roll_servo.write(temp + 1);
    } else if (roll < -error){
      int temp = roll_servo.read();
      roll_servo.write(temp - 1);
    }

    #ifdef DEBUG
    Serial.println("moved roll");
    #endif

    // store the values for the next run for the loop
    last_time = now;
    last_x = a.gyro.x;
    last_y = a.gyro.y;
    last_z = a.gyro.z;
    
    
    
  } else {
    // this is the branch to take if the gimbal is locked
    
    roll_servo.write(90);
    pitch_servo.write(90);
    
    // where it is pointing is the origin...
    yaw = 0;
    roll = 0;
    pitch = 0;

    // ...and this is the position of the origin
    last_x = a.gyro.x;
    last_y = a.gyro.y;
    last_z = a.gyro.z;
    last_time = millis();

  }

  #ifdef SHOW_VALS
  Serial.print(elapsed);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print(",");
  Serial.println(roll);
  #endif

  #ifdef WRITE_TO_CARD
  
  #endif
  
  delay(15);
}

void setup_servos() {
  pitch_servo.attach(pitch_pin);
  roll_servo.attach(roll_pin);

  pitch_servo.write(90);
  roll_servo.write(90);
}

#ifdef WRITE_TO_CARD
void write_to_card(char* file, char* toWrite) {
  /*
   * natalie's responsibility
   * 
   * takes in a filename and writes to it, and closes the file afterwards 
   */
  Serial.print("writing to ");
  Serial.println(file);
  int now = millis();
  File location = SD.open(file, FILE_WRITE);
  location.print(now);
  location.print(": ");
  location.println(toWrite);
  location.close();
}


bool init_card(int pin) {
  /*
   * natalie's responsibility
   * 
   * tries to intialise the SD card and get the filename to use
   */ 

  Serial.println("initialising SD card");

  
  SD.begin(pin);
  
  // can the card be initialised?
  if (!card.init(SPI_HALF_SPEED, CS)) {
    return false;
  } else {
     Serial.println("couldn't initialise SD card, exiting");
     while (true) {
      delay(1);
     }
  }

  filename = get_filename();


  Serial.print("activating the SD card worked, filename of ");
  Serial.println(filename);
  
  return true;
}

char* get_filename() {
  /*
   * function to get the filename to use, so each test gets a unique 
   * file
   * 
   */
  int i = 0;

  // files will follow the pattern test(number).csv
  char base_name[] = "mpu_test";
  char ending[] = ".txt";

  // need to allocate so the filename can be stored.
  // won't free this variable as it should always be useful
  char* return_memory = malloc(filename_bytes * sizeof(char));
  
  while (true) {
    // this checks every loop to see if a filename is free. 
    // if so, it returns it
    // if not, it tries the next one
    
    char filename [filename_bytes];
    sprintf(filename, "%s%d%s", base_name, i, ending); // formats the filename
    if (SD.exists(filename)) {
      i++;
    }else{
      strcpy(return_memory, filename);
      return return_memory;
    }
  }
}
#endif
