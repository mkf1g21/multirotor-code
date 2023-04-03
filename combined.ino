
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Servo.h>
MPU6050 mpu;

// Define the 3 servo motors
Servo servo0;
Servo servo1;
Servo servo2;
float correct;
int j = 0;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2    // use pin 2 on Arduino Uno & most boards

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;    // set true if DMP init was successful
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
uint8_t devStatus;            // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;         // count of all bytes currently in FIFO
uint8_t fifoBuffer[128]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;                     // [w, x, y, z]                 quaternion container
VectorInt16 aa;                 // [x, y, z]                        accel sensor measurements
VectorInt16 aaReal;         // [x, y, z]                        gravity-free accel sensor measurements
VectorInt16 aaWorld;        // [x, y, z]                        world-frame accel sensor measurements
VectorFloat gravity;        // [x, y, z]                        gravity vector
float euler[3];                 // [psi, theta, phi]        Euler angle container
float ypr[3];                     // [yaw, pitch, roll]     yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
// ================================================================
// ===                             INTERRUPT DETECTION ROUTINE                                ===
// ================================================================

volatile bool mpuInterrupt = false;         // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                                    NATALIES BIT OF SETUP                                     ===
// ================================================================

#include <SD.h>


#define filename_bytes 12

#define CS 6


char filename[filename_bytes];

File location;

Sd2Card card;

#define PRINT

// ================================================================
// ===                                            INITIAL SETUP                                             ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(17);
    mpu.setYGyroOffset(-69);
    mpu.setZGyroOffset(27);
    mpu.setZAccelOffset(1551); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
         Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // Define the pins to which the 3 servo motors are connected
    servo0.attach(10);
    servo1.attach(9);
    servo2.attach(8);


// ================================================================
// ===                                         SD CARD STUFF                                                ===
// ================================================================


    pinMode(CS, OUTPUT);
 
    // will almost definitely lead to a fifo overflow
    init_card(CS);

}
// ================================================================
// ===                                        MAIN PROGRAM LOOP                                         ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
            // try to get out of the infinite loop
            fifoCount = mpu.getFIFOCount();
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 128) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get Yaw, Pitch and Roll values
#ifdef OUTPUT_READABLE_YAWPITCHROLL

        // maybe could combine this down into one custom function?
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // adjust each value
        for (int i = 0; i < 3; i++) {
            ypr[i] = ypr[i] * 180/M_PI;
        }

        #ifdef PRINT    
        // print values if it is enabled    
        Serial.print(F("\nypr"));

        for (int i = 0; i < 3; i++) {
            Serial.print("    ");
            Serial.print(ypr[i]); 
        }
        #endif
        // Yaw, Pitch, Roll values - Radians to degrees
        
        // Skip 300 readings (self-calibration process)
        
        if (j <= 300) {
            correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
            j++;
            Serial.print("c");
        } else {
            // branch to run after calibration
            ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract    the last random Yaw value from the currrent value to make the Yaw 0 degrees
            // Map the values of the MPU6050 sensor from -90 to 90 to values suatable for the servo control from 0 to 180

            float servo_values[3];

            // calculate the values to send to the servos and maybeprint them
            for (int i = 0; i < 3; i++) {
                servo_values[i] = lerp(ypr[0], -90.0, 90.0, 0.0, 180.0);
                #ifdef PRINT
                //Serial.print(servo_values[i] + "    ");
                #endif
            }
            
            // Control the servos according to the MPU6050 orientation
            servo0.write(servo_values[0]);
            servo1.write(servo_values[1]);
            servo2.write(servo_values[2]);


            // log whatever is wanted to the SD card
            float buf[3] = {servo_values[0], servo_values[1], servo_values[2]};
            format_bytes(buf, 3);

        }
#endif
    }
}

float lerp(float x, float in_min, float in_max, float out_min, float out_max) {
    // reimplementation of map for floats, hopefully will lead to smoother movement
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void init_card(int pin) {
    /*
     * natalie's responsibility
     * 
     * tries to intialise the SD card and get the filename to use
     */ 

    Serial.println("initialising SD card");
    Serial.println(SD.begin(pin)); // should print a 1 to the serial console if all goes well
    
    // can the card be initialised?
    if (!card.init(SPI_HALF_SPEED, CS)) {
         Serial.println("couldn't initialise SD card, exiting");
         while (true) {
            delay(1);
         }
    }
    // finds the filename to use and stores it in global memory
    get_filename();
    // finds the location to store to and saves it in global memory
    location = SD.open(filename, FILE_WRITE);

    Serial.print("activating the SD card worked, filename of ");
    Serial.println(filename);

    //write_to_card("begin");
}

void format_array (float data[], int arr_size){
    // just a wrapper around write_to_card that makes the string look nice

    char *temp_str = (char*) malloc(8); // overestimation of how long each string will be
    char *output = (char*) malloc(4 * arr_size); // overestimation of how long the string to write to the SD card will be
    
    // for each piece of data in the array, add it to the output string
    for (int i = 0; i < arr_size + 1; i++){
        // float to string, should be x.xx
        dtostrf(data[i], 4, 2, temp_str);
        // add the string to the output string
        strcat(output, temp_str);
        //add a comma to make it CSV
        strcat(output, ",");
    }
    write_to_card(output);
    free(temp_str);
    free(output);
}

void format_bytes(float data[], int arr_size){

    char *output  = (char*) malloc(5*arr_size + 1); //overestimate of the length of the output string (i hope)
    for (int i = 0; i < arr_size; i++) {
        float x = data[i];
        output[5 * i + 0] = ((char*)&x)[0];
        output[5 * i + 1] = ((char*)&x)[1]; 
        output[5 * i + 2] = ((char*)&x)[2];
        output[5 * i + 3] = ((char*)&x)[3];
        output[5 * i + 4] = ',';
    }
    write_to_card(output);
    free(output);
}

void write_to_card(char* to_write) {
     /*
     * natalie's responsibility
     * 
     * takes in a filename and writes to it, making sure the file is actually written to 
     */
    
    location.write(to_write);
    location.write('\n');
    location.flush();
}

void get_filename() {
    /*
     * function to get the filename to use, so each test gets a unique 
     * file
     * 
     */
    int i = 0;

    // files will follow the pattern test(number).csv
    char base_name[] = "2a";
    char ending[] = ".txt";

    
    while (true) {
        // this checks every loop to see if a filename is free. 
        // if so, it returns it
        // if not, it tries the next one
        
        char test_filename[filename_bytes];
        sprintf(test_filename, "%s%d%s", base_name, i, ending); // formats the filename
        if (SD.exists(test_filename)) {

            i++;
        }else{
            strcpy(filename, test_filename);
            break;
        }
    }
}
