/*
 * demo.c
 *
 *  Created on: Oct 4, 2015
 *      Author: petrovic
 */

#include "tatrabot.h"

BaseSequentialStream *term = PC;

/* test the internal and external LEDs */
static void testLEDs(void)
{
  for (int i = 0; i < 7; i++)
  {
    setLED(i, 1);
    chThdSleepMilliseconds(400);
  }
  for (int i = 0; i < 7; i++)
  {
    setLED(i, 0);
    chThdSleepMilliseconds(400);
  }

  setLED(0, 1);
  chThdSleepMilliseconds(400);
  for (int i = 1; i < 7; i++)
  {
    setLED(i - 1, 0);
    setLED(i, 1);
    chThdSleepMilliseconds(400);
  }

  for (int i = 6; i >= 0; i--)
  {
    setLED(i, 0);
    setLED(i - 1, 1);
    chThdSleepMilliseconds(400);
  }
  setLED(0, 0);
}

/* test buttons K3, K4 */
static void testButtons(void)
{
  chprintf(term, "...please do K1-K2 restart when done...\r\n");
  while (true)
  {
    uint8_t but3 = senseButton(3);
    for (uint8_t i = 1; i < 4; i++)
      setLED(i, but3);
    uint8_t but4 = senseButton(4);
    for (uint8_t i = 4; i < 7; i++)
      setLED(i, but4);
  }
}

/* play that annoying infamous song */
static void testSound(void)
{
  sound(262);
  chThdSleepMilliseconds(700);

  sound(294);
  chThdSleepMilliseconds(700);

  sound(330);
  chThdSleepMilliseconds(1400);

  sound(349);
  chThdSleepMilliseconds(1400);

  sound(349);
  chThdSleepMilliseconds(700);

  sound(349);
  chThdSleepMilliseconds(700);

  sound(349);
  chThdSleepMilliseconds(700);

  sound(330);
  chThdSleepMilliseconds(700);

  sound(294);
  chThdSleepMilliseconds(1400);

  sound(330);
  chThdSleepMilliseconds(1400);

  sound(330);
  chThdSleepMilliseconds(700);

  sound(330);
  chThdSleepMilliseconds(700);

  sound(330);
  chThdSleepMilliseconds(700);

  sound(294);
  chThdSleepMilliseconds(700);

  sound(262);
  chThdSleepMilliseconds(1400);

  sound(294);
  chThdSleepMilliseconds(1400);

  sound(294);
  chThdSleepMilliseconds(700);

  sound(294);
  chThdSleepMilliseconds(700);

  sound(294);
  chThdSleepMilliseconds(700);

  sound(330);
  chThdSleepMilliseconds(700);

  sound(294);
  chThdSleepMilliseconds(1400);

  sound(262);
  chThdSleepMilliseconds(1400);

  sound(262);
  chThdSleepMilliseconds(700);

  sound(262);
  chThdSleepMilliseconds(700);

  nosound();
}

/* light the LEDs depending on the state of the line sensors */
static void testLine(void)
{
  while (!senseButton(3))
  {
    uint8_t ln = senseLine();
    setLED(1, ln & 1);
    setLED(2, (ln >> 1) & 1);
    setLED(3, (ln >> 2) & 1);
    setLED(4, (ln >> 3) & 1);
    chprintf(term, "%d %d %d %d\n\r", ln & 1, (ln >> 1) & 1, (ln >> 2) & 1, (ln >> 3) & 1);
  }
}

/*
static void testQuaternions(void)
{
  dmp_init();
  long q[4];
  unsigned char more;

  while (!senseButton(3))
  {
    dmp_read(q, &more);
    chprintf(PC, "%D %D %D %D\n\r", q[0], q[1], q[2], q[3]);
    chThdSleepMilliseconds(150);
  }
}
*/

/* light the LEDs depending on the state of the line sensors */
static void testObstacle(void)
{
  while (!senseButton(3))
  {
    setLED(1, obstacleLeft());
    setLED(2, obstacleRight());
  }
}

/* test the SHARP distance sensor - also show the distance on the serial line 1 */
static void testSharp(void)
{
  while (!senseButton(3))
  {
    chprintf(term, "d = %d\n\r", distance);
    sound(distance);
    chThdSleepMilliseconds(100);
  }
  nosound();
}

/* test the accelerometer - show the x,y,z values on serial line 1 */
static void testAcc(void)
{
  while (!senseButton(3))
  {
    senseAcc();
    chprintf(term, "x = %5d, y = %5d, z = %5d\n\r", (int16_t)(acc_vals[0]), (int16_t)(acc_vals[1]), (int16_t)(acc_vals[2]));
    chThdSleepMilliseconds(100);
  }
}

/* test the gyroscope - show the x,y,z values on serial line 1 */
static void testGyro(void)
{
  while (!senseButton(3))
  {
    senseGyro();
    chprintf(term, "x = %5d, y = %5d, z = %5d\n\r", (int16_t)(gyro_vals[0]), (int16_t)(gyro_vals[1]), (int16_t)(gyro_vals[2]));
    chThdSleepMilliseconds(100);
  }
}

/* test the temperature and pressure sensor */
static void testTemperature(void)
{
  while (!senseButton(3))
  {
    senseTemperatureAndPressure(0);
    chprintf(term, "temperature = %5D, pressure = %5D\n\r", temperature, pressure);
    chThdSleepMilliseconds(100);
  }
}

/* test the magnetometer/compass - show the x,y,z values and dir on serial line 1 */
/* minx= -380, maxx=  150, miny=-432, maxy=-369, minz=-315, maxz=203 */
static void testCompass(void)
{
  int minx = 10000;
  int maxx = -10000;
  int miny = 10000;
  int maxy = -10000;
  int minz = 10000;
  int maxz = -10000;
  while (!senseButton(3))
  {
    senseMagneto();
    chprintf(term, "x=%5d, y=%5d, z=%5d, dir=%5d, minx=%5d, maxx=%5d, miny=%5d, maxy=%5d, minz=%5d, maxz=%5d\n\r",
             (int16_t)(mag_vals[0]), (int16_t)(mag_vals[1]),
             (int16_t)(mag_vals[2]), compass(), minx, maxx, miny, maxy, minz, maxz);
    chThdSleepMilliseconds(300);
    if (mag_vals[0] < minx) minx = mag_vals[0];
    if (mag_vals[0] > maxx) maxx = mag_vals[0];
    if (mag_vals[1] < miny) miny = mag_vals[1];
    if (mag_vals[1] > maxy) maxy = mag_vals[1];
    if (mag_vals[2] < minz) minz = mag_vals[2];
    if (mag_vals[2] > maxz) maxz = mag_vals[2];
  }
}


/* test the rotation sensors - show the countA, countB on serial line 1 */
static void testICU(void)
{
  static unsigned int last_countA, last_countB;

  while (!senseButton(3))
  {
    if ((last_countA == countA) && (last_countB == countB)) continue;
    chprintf(term, "%5d\t%5d\r\n", countA, countB);
    last_countA = countA;
    last_countB = countB;
  }
}

int angle_difference(int alpha, int beta)
{
  int diff = beta - alpha;
  if (diff > 1800) return diff - 3600;
  else if (diff < -1800) return diff + 3600;
  return diff;
}


/* test forward movement with simple regulation using compass sensors */
void compassFwd(unsigned int distance, int speed)
{
  int speedL, speedR;
  speedL = speedR = speed;
  setMotor(3, speedL);
  senseMagneto();
  int starting_direction = compass();
  chprintf(PC, "start: %d", starting_direction);
  countA = 0;
  countB = 0;
  int iterationDelay = 20;
  while (countA < distance)
  {
    setMotor(1, speedL);
    setMotor(2, speedR);
    chThdSleepMilliseconds(iterationDelay);
    senseMagneto();
    int current_direction = compass();
    chprintf(PC, "current: %d", current_direction);
    int delta = angle_difference(starting_direction, current_direction);
    if (delta > 10)
    {
      speedL = 2 * speed / 3;
      speedR = speed;
    }
    else if (delta < -10)
    {
      speedL = speed;
      speedR = 2 * speed / 3;
    }
    else
    {
      speedL = speed;
      speedR = speed;
    }
  }
  motorsOff();
}

/* test forward movement with simple regulation using rotation sensors */
static void regulatedFwd(unsigned int distance, int speed)
{
  int speedL, speedR;
  speedL = speedR = speed;
  setMotor(3, speedL);
  countA = countB = 0;

  int iterationDelay = 50;
  while (countA < distance)
  {
    setMotor(1, speedL);
    setMotor(2, speedR);
    chThdSleepMilliseconds(iterationDelay);
    if (countA + countB > 16)
    {
      if (countA > countB + 1)
      {
        if (speedL < speed) speedL += 30;
        else speedR -= 30;
      }
      else if (countA < countB - 1)
      {
        if (speedR < speed) speedR += 30;
        else speedL -= 30;
      }
    }
  }
  motorsOff();
}

/* show some simple movements of the motors */
static void testMotors(void)
{
  chprintf(term, "\r\nleft fwd...\r\n");
  setMotor(1, 4000);
  chThdSleepMilliseconds(2000);
  chprintf(term, "right fwd...\r\n");
  setMotor(1, 0);
  setMotor(2, 4000);
  chThdSleepMilliseconds(2000);

  chprintf(term, "both fwd...\r\n");
  for (int i = 0; i < 10000; i += 3)
  {
    setMotor(3, i);
    chThdSleepMilliseconds(3);
  }
  chThdSleepMilliseconds(200);
  for (int i = 10000; i > 3500; i -= 3)
  {
    setMotor(3, i);
    chThdSleepMilliseconds(3);
  }
  for (int i = 0; i < 4; i++)
  {
    setMotor(3, 3500);
    chThdSleepMilliseconds(1000);
    setMotor(1, -3500);
    chThdSleepMilliseconds(2000);
  }
  setMotor(3, -3500);
  chThdSleepMilliseconds(2000);
  setMotor(1, 3500);
  chThdSleepMilliseconds(2000);
  for (int i = 3500; i > 0; i -= 3)
  {
    setMotor(3, i);
    chThdSleepMilliseconds(5);
  }
  motorsOff();
}

static void testBlueTooth(void)
{
  chprintf(PC, "please connect to the BT from your computer, and press K4 then...\r\n");
  while (!senseButton(4));
  chThdSleepMilliseconds(100); //debounce

  chprintf(PC, "everything you type on the BT terminal will be echoed back here, press K4 when happy...\r\n");
  while (senseButton(4));
  chThdSleepMilliseconds(100); //debounce

  while (!senseButton(4))
  {
    uint8_t chr = streamGet(BT);
    streamPut(PC, chr);
  }
  chThdSleepMilliseconds(100); //debounce
  while (senseButton(4));
  chThdSleepMilliseconds(100); //debounce

  chprintf(PC, "now everything you type here will be echoed back to the BT terminal, press K3 when done...\r\n");
  while (senseButton(4));
  chThdSleepMilliseconds(100); //debounce
  while (!senseButton(3))
  {
    uint8_t chr = streamGet(PC);
    streamPut(BT, chr);
  }
}

static void testRelayBlueTooth(void)
{
  chprintf(PC, "please connect to the BT from your computer, and press K4 then...\r\n");
  while (!senseButton(4));
  chThdSleepMilliseconds(100); //debounce

  chprintf(PC, "BT relay on, press K4 when done...\r\n");
  while (senseButton(4));
  chThdSleepMilliseconds(100); //debounce

  while (!senseButton(4))
  {
    uint8_t chr;
    int nread = 1;
    nread = sdAsynchronousRead(&SD3, &chr, 1);
    if (nread == 1) streamPut(PC, chr);

    nread = 1;
    nread = sdAsynchronousRead(&SD1, &chr, 1);
    if (nread == 1) streamPut(BT, chr);
  }

  chThdSleepMilliseconds(100); //debounce
  while (senseButton(4));
  chThdSleepMilliseconds(100); //debounce

  chprintf(PC, "done.\r\n");
}

static void cleanup_serial_ports(void)
{
  char a, b;
  while ((a = keyPressed(PC)));
  while ((b = keyPressed(BT)));
}

static unsigned char demo_menu(BaseSequentialStream *term)
{
  cleanup_serial_ports();
  chprintf(term, " *** Tatrabot demo program ***\r\n");
  chprintf(term, "  a] test LEDs\r\n");
  chprintf(term, "  b] test buttons K3,K4\r\n");
  chprintf(term, "  c] test sound\r\n");
  chprintf(term, "  d] test distance sensor\r\n");
  chprintf(term, "  e] test obstacle sensors\r\n");
  chprintf(term, "  f] test line sensor\r\n");
  chprintf(term, "  g] test accelerometer\r\n");
  chprintf(term, "  h] test rotation sensors\r\n");
  chprintf(term, "  i] test motors\r\n");
  chprintf(term, "  j] test regulated movement\r\n");
  chprintf(term, "  k] test BlueTooth communication\r\n");
  chprintf(term, "  l] set term=BT\r\n");
  chprintf(term, "  m] set term=PC\r\n");
  chprintf(term, "  n] test gyroscope\r\n");
  chprintf(term, "  o] test compass\r\n");
  chprintf(term, "  r] test BlueTooth Relay\r\n");
  //chprintf(term, "  q] test quaternions\r\n");
  //chprintf(term, "  p] print acc status\r\n");
  chprintf(term, "  t] test temperature and pressure sensor\r\n");
  chprintf(term, "  s] move with compass\r\n");
  chprintf(term, "\r\nyour selection> ");
  uint8_t choice = 0;
  do {
    char a = keyPressed(PC);
    char b = keyPressed(BT);
    if (a != 0) choice = a;
    else if (b != 0) choice = b;
  } while (choice == 0);
  //uint8_t choice = chSequentialStreamGet(term);
  chprintf(term, "%c\r\n\r\n...test in progress, wait for finish or terminate with button K3 or K1-K2 restart...", choice);
  return choice;
}

void testRegulatedFwd(void)
{
  for (int i = 0; i < 5; i++)
  {
    chThdSleepMilliseconds(5000);
    regulatedFwd(64, 5000);
  }
}

void testCompassFwd(void)
{
  chThdSleepMilliseconds(5000);
  compassFwd(640, 5000);
}

/* a simple demo program that demonstrates the capabilities of Tatrabot */
void demo(void)
{
  while (true)
  {
    switch (demo_menu(term))
    {
    case 'a': testLEDs(); break;
    case 'b': testButtons(); break;
    case 'c': testSound(); break;
    case 'd': testSharp(); break;
    case 'e': testObstacle(); break;
    case 'f': testLine(); break;
    case 'g': testAcc(); break;
    case 'h': testICU(); break;
    case 'i': testMotors(); break;
    case 'j': testRegulatedFwd(); break;
    case 'k': testBlueTooth(); break;
    case 'l': term = BT; break;
    case 'm': term = PC; break;
    case 'n': testGyro(); break;
    case 'o': testCompass(); break;
    case 'r': testRelayBlueTooth(); break;
    case 't': testTemperature(); break;
    case 's': testCompassFwd(); break;
    //case 'q': testQuaternions(); break;
    //case 'p': print_status_regs(); break;
    }
    chprintf(term, "test finished.\r\n\r\n");
  }
}
