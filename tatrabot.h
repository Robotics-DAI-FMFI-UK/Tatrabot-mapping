/*
 * tatrabot.h
 *
 *  Created on: Oct 2, 2015
 *      Author: petrovic
 */

#ifndef TATRABOT_H_
#define TATRABOT_H_

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

//#include "dmp.h"

#define PC ((BaseSequentialStream *)&SD1)
#define BT ((BaseSequentialStream *)&SD3)

//compass sensor I2C address
#define HMC5883L_I2C_ADDRESS_7BIT 0x1E
// accelerometer I2C address
#define MPU6050_I2C_ADDRESS_7BIT 0b1101000
// temperature/pressure sensor I2C address
#define BMP180_I2C_ADDRESS_7BIT 0b1110111

#define UNUSED(x) (void)(x)

/* turn the specified LED to state ON(1) or OFF(0)
 * LED0 = built-in blue LED
 * LED1 - LED6 = external LED module */
void setLED(uint8_t led, uint8_t state);

/* determine the state of the specified button, only buttons K3, and K4
 * can be determined, K1 is reset and K2 is BOOT0 */
uint8_t senseButton(int i);

/* turn off sound */
void nosound(void);

/* start playing sound of the specified frequency */
void sound(int freq);

/* sample the 4-channel binary line sensor, fills bits0..3 in the returned value */
uint8_t senseLine(void);

/* sample the current value of the left obstacle IR sensor: 0/1 */
uint8_t obstacleLeft(void);

/* sample the current value of the right obstacle IR sensor: 0/1 */
uint8_t obstacleRight(void);

/* last reading of the SHARP infrared sensor, it is measured automatically all the time */
extern volatile adcsample_t distance;

/* last reading of the accelerometer x,y,z values */
extern uint16_t acc_vals[3];

/* last reading of the gyroscope x,y,z values */
extern uint16_t gyro_vals[3];

/* last reading of the magnetometer x,y,z values */
extern int16_t mag_vals[3];

/* measure the accelerometer values, result will be in acc_vals[] */
void senseAcc(void);

/* measure the gyroscope values, result will be in acc_vals[] */
void senseGyro(void);


extern int32_t temperature;
extern int32_t pressure;

/* measure temperature and pressure, result is in variables temperature, pressure
 * temperature is in 0.1 deg.C, pressure in Pascal,
 * precision of pressure measurement:
 *  value - mode, # of samples, avg. typ. current/1 sample
 *      0 - ultra low power, 1 internal sample, 4.5 ms, 3 uA
 *      1 - standard, 2 internal samples, 7.5 ms, 5 uA
 *      2 - high resolution, 4 internal samples, 13.5 ms, 7 uA
 *      3 - ultra high resolution, 8 internal samples, 25.5 ms, 12 uA
 * temperature measurement takes extra 4.5 ms.
 */
uint8_t senseTemperatureAndPressure(uint8_t precision);

/* measure the magnetometer values, result will be in mag_vals[] */
uint8_t senseMagneto(void);

/* convert the magnetometer readings to compass heading -1800 to 1800, must be called after
 * senseMagneto();
 */
int16_t compass(void);

// current value of incremental counters A and B
extern volatile uint32_t countA, countB;

/* control motor m=1 (left) or m=2 (right), 3 means to control both */
void setMotor(int m, int speed);

/* turn off the motors and disable the motor driver (save power) */
void motorsOff(void);

/* produce a greeting sound and flash the LEDs - called at the end of tatrabotInit() */
void startupGreeting(void);

/* initialize all the input/output devices, should be called at the start of each program */
void tatrabotInit(void);

// returns 0 if no byte is available from the stream term,
// otherwise returns that byte
char keyPressed(BaseSequentialStream *term);

// clock that is incremented automatically by timer every 20 usec,
//  overflows after 23,860929422... hours
extern volatile uint32_t tatra_clock;

// returns distance measured by ultrasonic in cm
uint16_t measure_distance(int);

// alternately, ultrasonic can be measured in background, by setting this variable to 1...
extern volatile uint8_t measure_ultrasonic;

// ...and the result of measurement is available here, after measure_ultrasonic == 5
extern volatile uint16_t ultrasonic_distance;

#endif /* TATRABOT_H_ */
