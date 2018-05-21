/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "tatrabot.h"
#include <math.h>

/* PWM configuration for timer1 - used to drive motors A and B - channels 1,4 */
static PWMConfig pwmcfg =
{
  100000,                                    /* 100kHz PWM clock frequency.   */
  50,                                        /* 50 ticks, i.e. 2kHz for PWM */
  0,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, 0},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  0,
  0,
#if STM32_PWM_USE_ADVANCED
  0
#endif
};

/* PWM config for timer 4, used to drive the sound */
static PWMConfig pwm4cfg =
{
  1000000L,        /* timer frequency 1M */
  500000L,         /* PWM frequency 500K */
  0,
  {
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  0,
  0,
#if STM32_PWM_USE_ADVANCED
  0
#endif
};

/* in order to use the JTAG pins as GPIO, JTAG must be disabled */
static void disableJTAG(void)
{
  /* JTAG-DP Disabled and SW-DP Disabled */
  uint32_t reg = AFIO->MAPR;
      reg &= ~(1<<24 | 1<<25);
      reg |= (1<<26);
      AFIO->MAPR = reg;
}

/* setup the 7 GPIOs for LEDs as outputs */
static void configLEDs(void)
{
  palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOC, 13);

//  palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL);
//  palClearPad(GPIOA, 7);
  palSetPadMode(GPIOB, 4, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOB, 4);

//  palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL);
//  palClearPad(GPIOB, 5);
  palSetPadMode(GPIOB, 8, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOB, 8);

  // we use LED5 for Ultrasonic ECHO (which is input)
  //palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL);
  // palClearPad(GPIOB, 12);
  palSetPadMode(GPIOB, 13, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOB, 13);
}

/* drive the specified LED 0-6 to state 1 or 0 (ON of OFF), use 255 for all */
void setLED(uint8_t led, uint8_t state)
{
  switch (led)
  {
  case 0: palWritePad(GPIOC, 13, 1 - state); break;
  case 1: palWritePad(GPIOA, 7, state); break;
  case 2: palWritePad(GPIOB, 4, state); break;
  case 3: palWritePad(GPIOB, 5, state); break;
  case 4: palWritePad(GPIOB, 8, state); break;
  case 5: palWritePad(GPIOB, 12, state); break;
  case 6: palWritePad(GPIOB, 13, state); break;
  case 255: for (int i = 0; i < 7; i++) setLED(i, state); break;
  }
}

/* setup the GPIOS for buttons K3 and K4 as inputs with pull-up resistors */
static void configButtons(void)
{
  palSetPadMode(GPIOA, 13, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOA, 14, PAL_MODE_INPUT_PULLUP);
}

/* read the state of the specified button 3 or 4 */
uint8_t senseButton(int i)
{
  switch (i)
  {
    case 3: return 1 - palReadPad(GPIOA, 13);
    case 4: return 1 - palReadPad(GPIOA, 14);
  }
  return 0;
}

/* setup the sound pin as driven by timer */
static void configSound(void)
{
  palSetPadMode(GPIOB, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
}

// internal state variable playing==1 if the sound is being played, otherwise 0
static uint8_t playing;

/* stop playing sound - disconnect PWM of timer 4 */
void nosound(void)
{
  palSetPadMode(GPIOB, 9, PAL_MODE_INPUT_PULLDOWN);
  pwmDisableChannel(&PWMD4, 3);
  pwmStop(&PWMD4);
  playing = 0;
}

/* start playing sound of specified frequency, using PWM at timer 4, channel 4 */
void sound(int freq)
{
  if (playing) nosound();
  pwm4cfg.frequency = freq * 100;
  pwm4cfg.period = 100;
  pwmStart(&PWMD4, &pwm4cfg);
  pwmEnableChannel(&PWMD4, 3, 10);
  palSetPadMode(GPIOB, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  playing = 1;
}

/* setup the digital 4-channel sensor GPIOs as input */
static void configLine(void)
{
  palSetPadMode(GPIOB, 0, PAL_MODE_INPUT);
  palSetPadMode(GPIOB, 1, PAL_MODE_INPUT);
  palSetPadMode(GPIOA, 2, PAL_MODE_INPUT);
  palSetPadMode(GPIOA, 3, PAL_MODE_INPUT);
}

/* sample the current value of the 4-channel line sensor, returns in bits0..3 */
uint8_t senseLine(void)
{
  return  (palReadPad(GPIOA, 2) << 3) |
          (palReadPad(GPIOA, 3) << 2) |
          (palReadPad(GPIOB, 0) << 1) |
           palReadPad(GPIOB, 1);
}

/* setup the GPIOs of the left and right obstacle sensors */
static void configObstacle(void)
{
  palSetPadMode(GPIOA, 4, PAL_MODE_INPUT);
  palSetPadMode(GPIOA, 5, PAL_MODE_INPUT);
}

/* sample the current value of the left obstacle IR sensor: 0/1 */
uint8_t obstacleLeft(void)
{
  return palReadPad(GPIOA, 4);
}

/* sample the current value of the right obstacle IR sensor: 0/1 */
uint8_t obstacleRight(void)
{
  return palReadPad(GPIOA, 5);
}

/* last reading of the SHARP distance IR sensor */
volatile adcsample_t distance;

/* ADC config for the ADC1 with SHARP distance sensor connected */
static const ADCConversionGroup adccfg = {
  TRUE,  //circular
  1,     //num channels
  0,     //callback
  0,     //error callback
  0, 0,  // CR1, CR2
  0,     // smpr1
  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_28P5), // SMPR2
  ADC_SQR1_NUM_CH(1),                //sqr1
  0,                                // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)  //sqr3
};

/* config the ADC1 for SHARP distance sensor, and start automatic measurement */
static void configSharp(void)
{
  palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
  adcStart(&ADCD1, NULL);
  adcStartConversion(&ADCD1, &adccfg, (adcsample_t *)&distance, 1);
}

/* config for SD3 serial driver for BT */
static SerialConfig btcfg;

/* setup SD3 serial driver for BT */
static void configBT(void)
{
  btcfg.speed = 57600;
  btcfg.cr1 = 0b0010000000001100;
  btcfg.cr2 = 0;
  btcfg.cr3 = 0;
  palSetPadMode(GPIOB, 10, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(GPIOB, 11, PAL_MODE_INPUT);
  sdStart(&SD3, &btcfg);
  chThdSleepMilliseconds(100);
}



/* I2C1 config for accelerometer */
static const I2CConfig i2cfg1 =
{
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

// internal receive buffer for communication with accelerometer
static uint8_t accel_rx_data[8];
// internal transmit buffer for communication with accelerometer
static uint8_t accel_tx_data[8];

// register where to start reading magnetometer data
#define HMC_5883_MAG_OUT_X_REG 03

// magnetometer mode register
#define HMC_5883_MAG_MODE_REG 02

// register where to start reading accelerometer data
#define MPU_6050_ACCEL_XOUT_H_REG 59
// register where to start reading gyroscope data
#define MPU_6050_GYRO_XOUT_H_REG 67
// sleep mode register of accelerometer
#define MPU_6050_ACCEL_PWR_MGMT_1_REG 107
// user control register of accelerometer
#define MPU_6050_ACCEL_USER_CONTROL_REG 106
// INT PIN / Bypass enable register of accelerometer
#define MPU_6050_ACCEL_INT_PIN_CFG_REG 55

/* last reading from the acceleromenter */
uint16_t acc_vals[3];

/* last reading from the gyroscope */
uint16_t gyro_vals[3];

/* last reading from the magnetometer: x, y, z*/
int16_t mag_vals[3];

/* setup I2C to communicate with accelerometer and wake up from its sleep mode */
static void configAcc(void)
{
  msg_t status;
  do {
    palSetPadMode(GPIOB, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
    palSetPadMode(GPIOB, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
    i2cStart(&I2CD1, &i2cfg1);

    status = MSG_OK;
    systime_t tmo = MS2ST(40);

    accel_tx_data[0] = MPU_6050_ACCEL_PWR_MGMT_1_REG;
    accel_tx_data[1] = 0b00000000;
    i2cAcquireBus(&I2CD1);
    status = i2cMasterTransmitTimeout(&I2CD1, MPU6050_I2C_ADDRESS_7BIT,
                            accel_tx_data, 2, 0, 0, tmo);
    if (status != MSG_OK) break;

    accel_tx_data[0] = MPU_6050_ACCEL_USER_CONTROL_REG;
    status = i2cMasterTransmitTimeout(&I2CD1, MPU6050_I2C_ADDRESS_7BIT,
                               accel_tx_data, 1, accel_rx_data, 2, tmo);
    // connect compass to primary I2C bus
    if (status != MSG_OK) break;
    accel_rx_data[0] &= 0b11011111;
    accel_rx_data[0] |= 0b10;
    accel_tx_data[0] = MPU_6050_ACCEL_USER_CONTROL_REG;
    accel_tx_data[1] = accel_rx_data[0];
    status = i2cMasterTransmitTimeout(&I2CD1, MPU6050_I2C_ADDRESS_7BIT,
                            accel_tx_data, 2, 0, 0, tmo);

    if (status != MSG_OK) break;

    accel_tx_data[0] = MPU_6050_ACCEL_INT_PIN_CFG_REG;
    status = i2cMasterTransmitTimeout(&I2CD1, MPU6050_I2C_ADDRESS_7BIT,
                            accel_tx_data, 1, accel_rx_data, 2, tmo);
    // bypass enable
    accel_tx_data[0] = MPU_6050_ACCEL_INT_PIN_CFG_REG;
    accel_tx_data[1] = accel_rx_data[0] | (1 << 1);
    status = i2cMasterTransmitTimeout(&I2CD1, MPU6050_I2C_ADDRESS_7BIT,
                            accel_tx_data, 2, 0, 0, tmo);
    if (status != MSG_OK) break;

    accel_tx_data[0] = HMC_5883_MAG_MODE_REG;
    accel_tx_data[1] = 0;
    status = i2cMasterTransmitTimeout(&I2CD1, HMC5883L_I2C_ADDRESS_7BIT,
                            accel_tx_data, 2, 0, 0, tmo);
    if (status != MSG_OK) break;
    chThdSleepMicroseconds(100);

  } while (0);
  i2cReleaseBus(&I2CD1);
  if (status != MSG_OK) chprintf(PC, "warn: could'n not init acc, status: %d %d\r\n", status, i2cGetErrors(&I2CD1));

}
/* read current value from accelerometer, result will be in acc_vals */
void senseAcc(void)
{
  msg_t status = MSG_OK;
  systime_t tmo = MS2ST(40);
  int i = 0;

  accel_tx_data[0] = MPU_6050_ACCEL_XOUT_H_REG;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, MPU6050_I2C_ADDRESS_7BIT,
                          accel_tx_data, 1, accel_rx_data, 6, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != MSG_OK) chprintf(PC, "warn: i2c status: %d\r\n", status);

  for (i = 0; i < 3; i++)
    acc_vals[i] = (accel_rx_data[i * 2] << 8) + accel_rx_data[i * 2 + 1];
}

/* read current value from gyroscope, result will be in gyro_vals */
void senseGyro(void)
{
  msg_t status = MSG_OK;
  systime_t tmo = MS2ST(40);
  int i = 0;

  accel_tx_data[0] = MPU_6050_GYRO_XOUT_H_REG;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, MPU6050_I2C_ADDRESS_7BIT,
                          accel_tx_data, 1, accel_rx_data, 6, tmo);
  i2cReleaseBus(&I2CD1);

  if (status != MSG_OK) chprintf(PC, "warn: i2c status: %d\r\n", status);

  for (i = 0; i < 3; i++)
    gyro_vals[i] = (accel_rx_data[i * 2] << 8) + accel_rx_data[i * 2 + 1];
}

#define BMP180_CHIP_ID_REGISTER 0xD0
#define BMP180_CALIB_REGISTER_AA 0xAA
#define BMP180_CALIB_REGISTER_B2 0xB2
#define BMP180_CALIB_REGISTER_BA 0xBA
#define BMP180_ADC_OUT_REGISTER 0xF6
#define BMP180_MEASUREMENT_CONTROL_REGISTER 0xF4

inline uint16_t get_uint16t(uint8_t *a)
{
  return (((uint16_t)(a[0])) << 8) | (int16_t)(a[1]);
}

inline int16_t get_int16t(uint8_t *a)
{
  return (((int16_t)(a[0])) << 8) | (int16_t)(a[1]);
}

static int16_t bmp180_ac1;
static int16_t bmp180_ac2;
static int16_t bmp180_ac3;
static uint16_t bmp180_ac4;
static uint16_t bmp180_ac5;
static uint16_t bmp180_ac6;
static int16_t bmp180_b1;
static int16_t bmp180_b2;
static int16_t bmp180_mb;
static int16_t bmp180_mc;
static int16_t bmp180_md;

int32_t temperature;
int32_t pressure;

uint8_t configTemperature(void)
{
  systime_t tmo = MS2ST(40);
  i2cAcquireBus(&I2CD1);

  accel_tx_data[0] = BMP180_CHIP_ID_REGISTER;
  msg_t status = i2cMasterTransmitTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                    accel_tx_data, 1, 0, 0, tmo);
  msg_t status2 = i2cMasterReceiveTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                    accel_rx_data, 2, tmo);

  if ((status != MSG_OK) || (status2 != MSG_OK) || (accel_rx_data[0] != 0x55))
  {
    i2cReleaseBus(&I2CD1);
    chprintf(PC, "Temperature/pressure sensor not responding.\n\r");
    return 0;
  }

  accel_tx_data[0] = BMP180_CALIB_REGISTER_AA;
  status = i2cMasterTransmitTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                        accel_tx_data, 1, 0, 0, tmo);
  status2 = i2cMasterReceiveTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                      accel_rx_data, 8, tmo);
  if ((status != MSG_OK) || (status2 != MSG_OK))
  {
    i2cReleaseBus(&I2CD1);
    return 0;
  }

  bmp180_ac1 = get_int16t(accel_rx_data);
  bmp180_ac2 = get_int16t(accel_rx_data + 2);
  bmp180_ac3 = get_int16t(accel_rx_data + 4);
  bmp180_ac4 = get_uint16t(accel_rx_data + 6);

  accel_tx_data[0] = BMP180_CALIB_REGISTER_B2;
  status = i2cMasterTransmitTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                        accel_tx_data, 1, 0, 0, tmo);
  status2 = i2cMasterReceiveTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                      accel_rx_data, 8, tmo);
  if ((status != MSG_OK) || (status2 != MSG_OK))
  {
    i2cReleaseBus(&I2CD1);
    return 0;
  }
  bmp180_ac5 = get_uint16t(accel_rx_data);
  bmp180_ac6 = get_uint16t(accel_rx_data + 2);
  bmp180_b1 = get_int16t(accel_rx_data + 4);
  bmp180_b2 = get_int16t(accel_rx_data + 6);

  accel_tx_data[0] = BMP180_CALIB_REGISTER_BA;
  status = i2cMasterTransmitTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                      accel_tx_data, 1, 0, 0, tmo);
  status2 = i2cMasterReceiveTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                      accel_rx_data, 8, tmo);
  if ((status != MSG_OK) || (status2 != MSG_OK))
  {
    i2cReleaseBus(&I2CD1);
    chprintf(PC, "unable to read calibration registers from BMP180\r\n");
    return 0;
  }
  bmp180_mb = get_int16t(accel_rx_data);
  bmp180_mc = get_int16t(accel_rx_data + 2);
  bmp180_md = get_int16t(accel_rx_data + 4);

  i2cReleaseBus(&I2CD1);
  return 1;
}

void calculate_true_temperature_and_pressure(int32_t ut, int32_t up, uint8_t precision)
{
  int32_t x1 = ((ut - bmp180_ac6) * bmp180_ac5) >> 15;
  int32_t x2 = (bmp180_mc << 11) / (x1 + bmp180_md);
  int32_t b5 = x1 + x2;
  temperature = (b5 + 8) >> 4;

  int32_t b6 = b5 - 4000;
  x1 = (bmp180_b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (bmp180_ac2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = ((((bmp180_ac1 << 2) + x3) << precision) + 2) >> 2;
  x1 = (bmp180_ac3 * b6) >> 13;
  x2 = (bmp180_b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = (x1 + x2 + 2) >> 4;
  uint32_t b4 = (bmp180_ac4 * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = ((uint32_t)up - (uint32_t)b3) * (50000 >> precision);
  int32_t p;
  if (b7 < 0x80000000) p = (b7 << 1) / b4;
  else p = (b7 / b4) << 1;
  x1 = (p >> 8);
  x1 *= x1;
  x1 *= 3038;
  x1 >>= 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;
  pressure = p;
}

uint8_t senseTemperatureAndPressure(uint8_t precision)
{
  systime_t tmo = MS2ST(40);
  accel_tx_data[0] = BMP180_MEASUREMENT_CONTROL_REGISTER;
  accel_tx_data[1] = 0x2E;
  i2cAcquireBus(&I2CD1);
  msg_t status = i2cMasterTransmitTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                    accel_tx_data, 2, 0, 0, tmo);
  if (status != MSG_OK)
  {
    i2cReleaseBus(&I2CD1);
    return 0;
  }
  i2cReleaseBus(&I2CD1);
  chThdSleepMicroseconds(4500);
  accel_tx_data[0] = BMP180_ADC_OUT_REGISTER;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                      accel_tx_data, 1, 0, 0, tmo);
  msg_t status2 = i2cMasterReceiveTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                      accel_rx_data, 2, tmo);
  if ((status != MSG_OK) || (status2 != MSG_OK))
  {
    i2cReleaseBus(&I2CD1);
    return 0;
  }
  int32_t ut = (int32_t)get_uint16t(accel_rx_data);

  accel_tx_data[0] = BMP180_MEASUREMENT_CONTROL_REGISTER;
  accel_tx_data[1] = 0x34 + (precision << 6);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                      accel_tx_data, 2, 0, 0, tmo);
  if (status != MSG_OK)
  {
    i2cReleaseBus(&I2CD1);
    return 0;
  }

  uint16_t waiting_time = 1500 + 3000 * (1 << precision);
  i2cReleaseBus(&I2CD1);
  chThdSleepMicroseconds(waiting_time);

  accel_tx_data[0] = BMP180_ADC_OUT_REGISTER;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                      accel_tx_data, 1, 0, 0, tmo);
  status2 = i2cMasterReceiveTimeout(&I2CD1, BMP180_I2C_ADDRESS_7BIT,
                                        accel_rx_data, 3, tmo);

  i2cReleaseBus(&I2CD1);
  if ((status != MSG_OK) || (status2 != MSG_OK)) return 0;
  int32_t up = (int32_t)get_uint16t(accel_rx_data) << 8;
  up += (int8_t)(accel_rx_data[2]);
  up >>= 8 - precision;

  calculate_true_temperature_and_pressure(ut, up, precision);
  return 1;
}

/* read current value from magnetometer, result will be in mag_vals */
uint8_t senseMagneto(void)
{
  msg_t status = MSG_OK;
  systime_t tmo = MS2ST(40);
  int i = 0;

  accel_tx_data[0] = HMC_5883_MAG_OUT_X_REG;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, HMC5883L_I2C_ADDRESS_7BIT,
                          accel_tx_data, 1, 0, 0, tmo);
  msg_t status2 = i2cMasterReceiveTimeout(&I2CD1, HMC5883L_I2C_ADDRESS_7BIT,
                                        accel_rx_data, 6, tmo);
  i2cReleaseBus(&I2CD1);

  if ((status != MSG_OK) || (status2 != MSG_OK))
    return 0;

  for (i = 0; i < 3; i++)
    mag_vals[i] = (((int16_t)(accel_rx_data[i * 2])) << 8) + (int16_t)(accel_rx_data[i * 2 + 1]);
  return 1;
}

int16_t compass(void)
{
  double x = mag_vals[0];
  double z = mag_vals[2];
  /* find your own minx,maxx, minz,maxz using testCompass() */
  /* minx= -380, maxx=  150, miny=-432, maxy=-369, minz=-315, maxz=203 */
  static double offx = (150 + (-380)) / 2.0;
  static double offz = (203 + (-315)) / 2.0;
  x -= offx;
  z -= offz;
  double cmps = 180.0 * atan2(z, x) / 3.1415926536;
  int16_t cmpsi = (int16_t) (cmps * 10.0 + 0.5);
  return cmpsi;
}

// current value of incremental counter 2
volatile uint32_t countA;
// current value of incremental counter 3
volatile uint32_t countB;


#if 0

// internal counters for the incremental encoders
static volatile icucnt_t last_width2, last_period2, last_overflow2, ignored2;

// incremental encoder callback - called with each pulse
static void icuperiodcb2(ICUDriver *icup)
{
  UNUSED(icup);
  icucnt_t lp2 = (last_overflow2 << 14) | icuGetPeriodX(icup);
  last_overflow2 = 0;
//  setLED(5, 0);
  if (lp2 > 30) //filter out spurious readings (sort of a transparent wheel we have)
  {
    last_period2 = lp2;
    countA++;
  }
  else ignored2++;
}

/* in case the pulse is too long, this overflow callback will be used */
static void icuoverflowcb2(ICUDriver *icup)
{
  UNUSED(icup);
  last_overflow2++;
}

/* config for ICU driver, it uses timer 2, channel 1 */
static ICUConfig icucfg2 =
{
  ICU_INPUT_ACTIVE_HIGH,
  1000,                                    /* ICU clock frequency */
  0,
  icuperiodcb2,
  icuoverflowcb2,
  ICU_CHANNEL_1,
  0
};

// internal counters for the incremental encoders
static volatile icucnt_t last_width3, last_period3, last_overflow3, ignored3;

// incremental encoder callback - called with each pulse
static void icuperiodcb3(ICUDriver *icup)
{
  icucnt_t lp3 = (last_overflow3 << 14) | icuGetPeriodX(icup);
  last_overflow3 = 0;
//  setLED(6, 0);
  if (lp3 > 30)
  {
    last_period3 = lp3;
    countB++;
  }
  else ignored3++;
}

/* in case the pulse is too long, this overflow callback will be used */
static void icuoverflowcb3(ICUDriver *icup)
{
  UNUSED(icup);
  last_overflow3++;
}

/* config for ICU driver, it uses timer 3, channel 1 */

static ICUConfig icucfg3 = {
  ICU_INPUT_ACTIVE_HIGH,
  1000,                                    /* ICU clock frequency.   */
  NULL,
  icuperiodcb3,
  icuoverflowcb3,
  ICU_CHANNEL_1,
  0
};
*/

/* config the ICU drivers 2 and 3 on timers 2 and 3 */
static void configICU(void)
{
  countA = countB = 0;
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_PULLDOWN);
  palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_PULLDOWN);
  icuStart(&ICUD2, &icucfg2);
//  icuStart(&ICUD3, &icucfg3);
  icuStartCapture(&ICUD2);
  icuStartCapture(&ICUD3);
  icuEnableNotifications(&ICUD2);
  icuEnableNotifications(&ICUD3);
}

#endif

static uint8_t encA_previous_state, encB_previous_state;

static void encoders_output_callback(GPTDriver *gptp)
{
    UNUSED(gptp);
    uint8_t encA_new_state = palReadPad(GPIOA, 0);
    if (encA_new_state != encA_previous_state)
    {
      encA_previous_state = encA_new_state;
      countA++;
    }
    uint8_t encB_new_state = palReadPad(GPIOA, 6);
    if (encB_new_state != encB_previous_state)
    {
      encB_previous_state = encB_new_state;
      countB++;
    }
}

static GPTConfig encoders_timer_cfg =
{
    1000,                       /* timer clock.*/
    encoders_output_callback,     /* Timer callback.*/
    0, 0
};

static void configICU(void)
{
  countA = countB = 0;
  encA_previous_state = encB_previous_state = 0;
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_PULLDOWN);
  palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_PULLDOWN);
  gptStart(&GPTD2, &encoders_timer_cfg);
  gptStartContinuous(&GPTD2, 1); // dT = 1,000 / 1 = 1 kHz, i.e. 1 msec
}

volatile uint32_t tatra_clock;

volatile uint8_t measure_ultrasonic;

volatile uint16_t ultrasonic_distance;

static volatile uint32_t measurement_start;

// LEFT = 3, FRONT = 1, RIGHT = 2
// trigger 4(2) , 8(3) , 13(1)
// echo 7(2) , 5(3) , 12(1)

uint16_t measure_distance(int sensor_num)
{
  if (sensor_num == 0){
    measure_ultrasonic = 1;
    while (measure_ultrasonic != 5);
    measure_ultrasonic = 0;
    return ultrasonic_distance;
  }
  if (sensor_num == 1){
    measure_ultrasonic = 7;
    while (measure_ultrasonic != 11);
    measure_ultrasonic = 0;
    return ultrasonic_distance;
  }
  if (sensor_num == 2){
    measure_ultrasonic = 13;
    while (measure_ultrasonic != 17);
    measure_ultrasonic = 0;
    return ultrasonic_distance;
  }
  return 1000;
}

static void tatra_clock_output_callback(GPTDriver *gptp)
{
    UNUSED(gptp);
    tatra_clock++;

    // ULTRASONIC SENSOR 1
    if (measure_ultrasonic)
    {
      if (measure_ultrasonic == 1)
      {
        palSetPad(GPIOB, 13);
        measure_ultrasonic++;
      }
      else if (measure_ultrasonic == 2)
      {
        palClearPad(GPIOB, 13);
        measure_ultrasonic++;
      }
      else if (measure_ultrasonic == 3)
      {
        if (palReadPad(GPIOB, 12))
        {
          measurement_start = tatra_clock;
          measure_ultrasonic++;
        }
      }
      else if (measure_ultrasonic == 4)
      {
        if (!palReadPad(GPIOB, 12))
        {
          ultrasonic_distance = ((uint16_t)(tatra_clock - measurement_start)) * 20 / 58;
          measure_ultrasonic++;
        }
        else if (tatra_clock - measurement_start > 870)  // if pulse longer than 1.5 m, give up waiting
        {
          palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL);
          palClearPad(GPIOB, 12);
          measure_ultrasonic = 6;
        }
      }
      else if (measure_ultrasonic == 6)
      {
        palSetPadMode(GPIOB, 12, PAL_MODE_INPUT_PULLDOWN);
        ultrasonic_distance = 1000;
        measure_ultrasonic = 5;
      }

      // ULTRASONIC SENSOR 2

      if (measure_ultrasonic == 7)
      {
        palSetPad(GPIOB, 4);
        measure_ultrasonic++;
      }
      else if (measure_ultrasonic == 8)
      {
        palClearPad(GPIOB, 4);
        measure_ultrasonic++;
      }
      else if (measure_ultrasonic == 9)
      {
        if (palReadPad(GPIOA, 7))
        {
          measurement_start = tatra_clock;
          measure_ultrasonic++;
        }
      }
      else if (measure_ultrasonic == 10)
      {
        if (!palReadPad(GPIOA, 7))
        {
          ultrasonic_distance = ((uint16_t)(tatra_clock - measurement_start)) * 20 / 58;
          measure_ultrasonic++;
        }
        else if (tatra_clock - measurement_start > 870)  // if pulse longer than 1.5 m, give up waiting
        {
          palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL);
          palClearPad(GPIOA, 7);
          measure_ultrasonic = 12;
        }
      }
      else if (measure_ultrasonic == 12)
      {
        palSetPadMode(GPIOB, 7, PAL_MODE_INPUT_PULLDOWN);
        ultrasonic_distance = 1000;
        measure_ultrasonic = 11;
      }


      // ULTRASONIC SENSOR 3

      if (measure_ultrasonic == 13)
      {
        palSetPad(GPIOB, 8);
        measure_ultrasonic++;
      }
      else if (measure_ultrasonic == 14)
      {
        palClearPad(GPIOB, 8);
        measure_ultrasonic++;
      }
      else if (measure_ultrasonic == 15)
      {
        if (palReadPad(GPIOB, 5))
        {
          measurement_start = tatra_clock;
          measure_ultrasonic++;
        }
      }
      else if (measure_ultrasonic == 16)
      {
        if (!palReadPad(GPIOB, 5))
        {
          ultrasonic_distance = ((uint16_t)(tatra_clock - measurement_start)) * 20 / 58;
          measure_ultrasonic++;
        }
        else if (tatra_clock - measurement_start > 870)  // if pulse longer than 1.5 m, give up waiting
        {
          palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL);
          palClearPad(GPIOB, 5);
          measure_ultrasonic = 18;
        }
      }
      else if (measure_ultrasonic == 18)
      {
        palSetPadMode(GPIOB, 5, PAL_MODE_INPUT_PULLDOWN);
        ultrasonic_distance = 1000;
        measure_ultrasonic = 17;
      }
    }



}

static GPTConfig tatra_clock_cfg =
{
    100000,                               /* timer clock */
    tatra_clock_output_callback,          /* Timer callback.*/
    0, 0
};

static void configUltrasonic(void)
{
  //B12: ECHO, B13: TRIG
  palSetPadMode(GPIOB, 12, PAL_MODE_INPUT_PULLDOWN);
  palSetPadMode(GPIOB, 13, PAL_MODE_OUTPUT_PUSHPULL);

  palSetPadMode(GPIOB, 5, PAL_MODE_INPUT_PULLDOWN);
  palSetPadMode(GPIOB, 8, PAL_MODE_OUTPUT_PUSHPULL);

  palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_PULLDOWN);
  palSetPadMode(GPIOB, 4, PAL_MODE_OUTPUT_PUSHPULL);
  tatra_clock = 0;
  measure_ultrasonic = 0;
  gptStart(&GPTD3, &tatra_clock_cfg);
  gptStartContinuous(&GPTD3, 1); // dT = 100,000 / 1 = 100 kHz, i.e. 10 usec
}

/* setup all control pins for driving motors and connect PWM pins to timer1 */
static void configMotors(void)
{
  palSetPadMode(GPIOB, 3, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 14, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 15, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 12, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 15, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 8, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(GPIOA, 11, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palClearPad(GPIOB, 14);
  palClearPad(GPIOB, 15);
  palClearPad(GPIOA, 12);
  palClearPad(GPIOA, 15);
  palClearPad(GPIOB, 3);
  pwmStart(&PWMD1, &pwmcfg);
  pwmEnablePeriodicNotification(&PWMD1);
}

/* control motor m=1 (left) or m=2 (right), 3 means to control both */
void setMotor(int m, int speed)
{
  if (m & 1)
  {
    if (speed > 0)
    {
      palClearPad(GPIOB, 14);
      palSetPad(GPIOB, 15);
      pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, speed));
    }
    else
    {
      palSetPad(GPIOB, 14);
      palClearPad(GPIOB, 15);
      pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, -speed));
    }
  }
  if (m & 2)
  {
    if (speed > 0)
    {
      palClearPad(GPIOA, 12);
      palSetPad(GPIOA, 15);
      pwmEnableChannel(&PWMD1, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, speed));
    }
    else
    {
      palSetPad(GPIOA, 12);
      palClearPad(GPIOA, 15);
      pwmEnableChannel(&PWMD1, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, -speed));
    }
  }
  palSetPad(GPIOB, 3);
}

/* turn off the motors and disable the motor driver (save power) */
void motorsOff(void)
{
  pwmDisableChannel(&PWMD1, 0);
  pwmDisableChannel(&PWMD1, 3);
  palClearPad(GPIOB, 3);
}

/* produce a greeting sound and flash the LEDs - called at the end of tatrabotInit() */
void startupGreeting(void)
{
  int f = 55;
  setLED(1, 1);
  sound(f);
  chThdSleepMilliseconds(100);
  for (int i = 1; i <= 5; i++)
  {
    setLED(i, 0);
    setLED(i + 1, 1);
    chThdSleepMilliseconds(100);
    f <<= 1;
    sound(f);
  }
  for (int i = 5; i >= 1; i--)
  {
    setLED(i + 1, 0);
    setLED(i, 1);
    chThdSleepMilliseconds(100);
    f >>= 1;
    sound(f);
  }
  setLED(1, 0);
  nosound();
}

/* setup serial driver 1 to communicate with the PC over the USB-serial conv. */
static void configSerial(void)
{
  sdStart(&SD1, NULL);
  palSetPadMode(GPIOA, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(GPIOA, 10, PAL_MODE_INPUT);
}

// returns 0 if no byte is available from the stream term,
// otherwise returns that byte
char keyPressed(BaseSequentialStream *term)
{
  uint8_t chr;
  int nread = 1;
  nread = sdAsynchronousRead((SerialDriver *)term, &chr, 1);
  if (nread == 1) return chr;
  else return 0;
}

/* setup all input/output devices of Tatrabot */
void tatrabotInit(void)
{
  halInit();
  chSysInit();

  disableJTAG();
  configSerial();
  configBT();
  configMotors();
  configLEDs();
  configUltrasonic();
  configButtons();
  configLine();
  configObstacle();
  configSound();
  configSharp();
  configAcc();
  configTemperature();
  configICU();
  startupGreeting();
}
