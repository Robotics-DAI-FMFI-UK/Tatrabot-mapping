#include "tatrabot.h"
#include "demo.h"

static int16_t sn[16][16];
static int16_t so[16][16];

static int16_t map[16][16];
static int16_t visited[16][16];

static int8_t robot_row, robot_col;
static int8_t dx, dy;

#define SHOW_THE_ROBOT 1
#define DO_NOT_SHOW_THE_ROBOT 0


static void init_fake_map(void)
{
  map[8][5] = 1;
  map[8][4] = 1;
}

static void follow_line_to_crossing(int rotations, int measuring)
{
  int measured = 0;
  countB=0;
  while (((senseLine() & 1) == 0) && (countB < rotations))
  {
      if (measuring)
      {
        measured++;
//        sense();
      }
      // if the second from the left sees the line, a little bit to the right
      if (((senseLine() >> 2) & 1) == 1)
      {
        setMotor(1, 5000);
        setMotor(2, 0);
      }
      // otherwise if the leftmost sees no line, a little bit to the left
      else if (((senseLine() >> 3) & 1) == 0)
      {
        setMotor(2, 5000);
        setMotor(1, 0);
      }
      else setMotor(3, 4000);
  }

  setMotor(3, -5000);
  chThdSleepMilliseconds(200);
  setMotor(3, 0);
  if (measured)
  {
//    chprintf(BT, "Measured %d times!\n", measured);
  }
}



static void forward(void){
  setMotor(3, 4000);
  chThdSleepMilliseconds(500);
  follow_line_to_crossing(1000, 1);

  robot_row += 2 * dy;
  robot_col += 2 * dx;

}

static void right(void){
  //setMotor(3, 4000);
  //chThdSleepMilliseconds(100);
  //setMotor(3, 0);
  countB=0;
  setMotor(1, 4000);
  while (countB<25){
    //chprintf(PC, "%d %d\r\n", countA, countB);
  }
  setMotor(1, 0);
  countB=0;
  setMotor(3, -4000);

  while (countB<38){

  }
  setMotor(3,0);
  follow_line_to_crossing(1000, 0);

  // (a,b) => (-b, a),  (b, -a)

  // dx = 0, dy = -1  =>
  // dx = 1, dy = 0  => dx = 0, dy = 1 => dx = -1, dy = 0  => dx = 0, dy = -1

  int8_t ndx = -dy;
  int8_t ndy = dx;

  dx = ndx;
  dy = ndy;

  robot_row -= dy;
  robot_col -= dx;

  return;

    //countA countB
      // if the second from the left sees the line, a little bit to the right
      //if (((senseLine() >> 2) & 1) && ((senseLine() >> 1) & 1) && ((senseLine() >> 0) & 1) && (((senseLine() >> 3)&1) == 0))
      /*{
        setMotor(3, 0);
      }*/
      while ((senseLine() & 1) == 0)
      {
          // if the second from the left sees the line, a little bit to the right
          if (((senseLine() >> 2) & 1) == 1)
          {
            setMotor(2, -5000);
            setMotor(1, 0);
          }
          // otherwise if the leftmost sees no line, a little bit to the left
          else if (((senseLine() >> 3) & 1) == 0)
          {
            setMotor(1, -5000);
            setMotor(2, 0);
          }
          else setMotor(3, -4000);
      }

  setMotor(3, -5000);
  chThdSleepMilliseconds(200);
  setMotor(3, 0);
}

static void left(void){

  setMotor(3, 4000);
  chThdSleepMilliseconds(500);
  follow_line_to_crossing(17, 0);
  setMotor(3, 0);
  setMotor(1, -4000);
  countB=0;

  while (countB < 25){

  }
  setMotor(3, 0);
  setMotor(3, -4000);
  while (countB < 10){

  }
  setMotor(3, 0);
  sound(880);
  chThdSleepMilliseconds(200);
  nosound();
  setMotor(3, 4000);
  follow_line_to_crossing(1000, 0);
  chThdSleepMilliseconds(200);
  sound(1760);
  chThdSleepMilliseconds(200);
  nosound();

  robot_row += dy;
  robot_col += dx;

  int8_t ndx = dy;
  int8_t ndy = -dx;

  dx = ndx;
  dy = ndy;
}

#define KEY_FORWARD  'w'
#define KEY_LEFT  'a'
#define KEY_RIGHT  'd'
#define KEY_FUN 'f'
#define KEY_PRINT 'p'
#define KEY_MEASURE 'm'
#define KEY_TRAVEL 't'


#define KEY_READ_DISTANCE  'r'
#define KEY_SENSE  's'

int DISTANCE_UNIT = 42;

int getDistanceUnit(void)
{
  countA = 0;
  // Pass cross
  setMotor(3, 4000);
  chThdSleepMilliseconds(500);

  // Find cross
  while ((senseLine() & 1) == 0)
  {
      // if the second from the left sees the line, a little bit to the right
      if (((senseLine() >> 2) & 1) == 1)
      {
        setMotor(1, 5000);
        setMotor(2, 0);
      }
      // otherwise if the leftmost sees no line, a little bit to the left
      else if (((senseLine() >> 3) & 1) == 0)
      {
        setMotor(2, 5000);
        setMotor(1, 0);
      }
      else setMotor(3, 4000);
  }

  // Get back before cross
  setMotor(3, -5000);
  chThdSleepMilliseconds(200);
  setMotor(3, 0);
  return countA;
}

#define box_width 12.5
#define cone_spread 15.2
#define half_box_width (box_width / 2)

float calculate_distance_left(float x, float y)
{
  float a = -100 / (cone_spread + 101);
  float b = -1 + 100 / (cone_spread + 101);
  return a*x + b*y + 1;
}

float calculate_distance_right(float x, float y)
{
  float a = 100 / (cone_spread - 101);
  float b = -1 - 100 / (cone_spread - 101);
//  chprintf(BT, "X: %d || Y: %d \n\r", (long) (10000 * x), (long) (10000 * y));
  return a*x + b*y + 1;
}

int is_in_cone(float x, float y)
{
  float line_value_left = calculate_distance_left(x, y);
  float line_value_right = calculate_distance_right(x, y);
//  chprintf(BT, "LEFT: %d || RIGHT: %d \n\r", (int) (1000 * line_value_left), (int) (1000 * line_value_right));
  return line_value_left <= 0 && line_value_right >= 0;
}

int overfill_cone(float x, float y)
{
  float line_value_left = calculate_distance_left(x, y);
  float line_value_right = calculate_distance_right(x + box_width, y);
//  chprintf(BT, "LEFT: %d || RIGHT: %d \n\r", (int) (1000 * line_value_left), (int) (1000 * line_value_right));
  return line_value_left > 0 && line_value_right < 0;
}

// indexy su X, Y
float sensor_shifts[3][2] = {{-1            , box_width - 1},
                             {-5            , 2*box_width - 7},
                             {5 - box_width , box_width - 4}};

void sensor_vector(int sensor, int *sdx, int *sdy)
{
  switch (sensor) {
    case 0:
      *sdx = dx;
      *sdy = dy;
      break;
    case 1:
      *sdx = -dy;
      *sdy = dx;
      break;
    case 2:
      *sdx = dy;
      *sdy = -dx;
      break;
  }
}

void update_boxes_unseen_with_delta(int sensor, int distance, int delta_r)
{
  int sdx, sdy;
  sensor_vector(sensor, &sdx, &sdy);
  int sdx90, sdy90;
  sdx90 = -sdy;
  sdy90 = sdx;
  int delta = 1;
//  chprintf(BT, "----------sensor: %d, distance: %d, delta_r: %d\n\r", sensor, distance, delta_r);

  for (float d = 0; d <= distance - box_width; d += box_width, delta++) {
    int in_cone = 1;
    int r = ((delta_r == -1) ? -1  : 0);
    while (in_cone) {
      float x = r*box_width + sensor_shifts[sensor][0];
      float y = d + sensor_shifts[sensor][1];
      if (!is_in_cone(x, y) && !is_in_cone(x + box_width, y)) {
        in_cone = 0;
      } else {
        int vertex_row = robot_row + (sdy * delta + sdy90 * r);
        int vertex_col = robot_col + (sdx * delta + sdx90 * r);
//        chprintf(BT, "%d,%d\n\r", vertex_row, vertex_col);

        if (vertex_row >= 0 && vertex_row <= 15 && vertex_col >= 0 && vertex_col <= 15) {
          sn[vertex_row][vertex_col] += 1;
        } else {
          in_cone = 0;
        }
      }
      r += delta_r;
    }
  }
}

void update_boxes_seen_with_delta(int sensor, int distance, int delta_r)
{
  int sdx, sdy;
  sensor_vector(sensor, &sdx, &sdy);
  int sdx90, sdy90;
  sdx90 = -sdy;
  sdy90 = sdx;
  int delta = ((int) (distance / box_width)) + 1;
  int in_cone = 1;
  int r = ((delta_r == -1) ? -1  : 0);
  while (in_cone) {
    float x = r*box_width + sensor_shifts[sensor][0];
    float y = (delta - 1) * box_width + sensor_shifts[sensor][1];
    if (!overfill_cone(x, y) && !is_in_cone(x, y) && !is_in_cone(x + box_width, y)) {
      in_cone = 0;
//      chprintf(BT, "DONE X:%d Y:%d\n\r", (int) (10000 * x), (int) (10000 * y));
    } else {
      int vertex_row = robot_row + (sdy * delta + sdy90 * r);
      int vertex_col = robot_col + (sdx * delta + sdx90 * r);
      if (vertex_row >= 0 && vertex_row <= 15 && vertex_col >= 0 && vertex_col <= 15) {
        so[vertex_row][vertex_col] += 1;
      } else {
        in_cone = 0;
      }
    }
    r += delta_r;
  }
}

void update_boxes_unseen(int sensor, int distance)
{
  update_boxes_unseen_with_delta(sensor, distance, 1);
  update_boxes_unseen_with_delta(sensor, distance, -1);
}

void update_boxes_seen(int sensor, int distance)
{
  update_boxes_seen_with_delta(sensor, distance, 1);
  update_boxes_seen_with_delta(sensor, distance, -1);
}

// pri vzdialenosti 100 CM, od stredu vidi do cone s odchylkou 15 CM do oboch stran
void sense(void)
{
  for(int i = 0; i < 3; i++){
    int sense_temp = measure_distance(i);
    if (sense_temp != 1000){
      update_boxes_seen(i, sense_temp);
      update_boxes_unseen(i, sense_temp);
    } else{
      update_boxes_unseen(i, 200);
    }
  }
}

char readKey(void)
{
  char key;
  do {
    key = keyPressed(BT);
  } while (key == 0);
  return key;
}

char robot_orientation(void)
{
  if (dx == 1) return '>';
  if (dx == -1) return '<';
  if (dy == 1) return 'v';
  if (dy == -1) return '^';
  return '@';
}

void print_array(uint8_t show_robot)
{
  chprintf(BT, "-----see obstacle----------------\n\r");
  for (int row = 0; row < 16; row++)
  {
    for (int col = 0; col < 16; col++)
    {
       if (show_robot && (row == robot_row) && (col == robot_col))
          chprintf(BT, " R%c ", robot_orientation());
       else
          chprintf(BT, "%3d ", so[row][col]);
    }
    chprintf(BT, "\n\r");
  }
  chprintf(BT, "-----see nothing-----------------\n\r");
  for (int row = 0; row < 16; row++)
  {
    for (int col = 0; col < 16; col++)
    {
      if (show_robot && (row == robot_row) && (col == robot_col))
         chprintf(BT, " R%c ", robot_orientation());
      else
         chprintf(BT, "%3d ", sn[row][col]);
    }
    chprintf(BT, "\n\r");
  }
  chprintf(BT, "--------------------------------\n\r");
  chprintf(BT, "row=%d, col=%d, dx=%d, dy=%d\n\r", robot_row, robot_col, dx, dy);
}


void fun(void)
{
  for (int row = 0; row < 16; row++)
  {
    for (int col = 0; col < 16; col++)
    {
       if (row > col) so[row][col] = 1;
       else if (row < col) so[row][col] = 0;
       else so[row][col] = 2;
    }
  }

  for (int row = 0; row < 16; row++)
  {
    for (int col = 0; col < 16; col++)
    {
       chprintf(BT, "%d", so[row][col]);
    }
    chprintf(BT, "\n\r");
  }
}

void measure(void)
{
  sense();
}

void navigate_to(int drow, int dcol)
{
  // travel from location robot_row, robot_col with heading dx,dy

  //  to location drow, dcol, avoiding non-zero elements in map[][]
  for (int row = 0; row < 16; row++)
     for (int col = 0; col < 16; col++)
        visited[row][col] = 0;

  int r = robot_row;
  int c = robot_col;

/*
  while ((r != drow) || (c != dcol))
  {
    visited[r][c] = 1;  // 1 = decrease row, 2 = increase col, 3 = decrease col, 4 = increase row
    if ()
  }

  for (int row = 0; row < 16; row++)
    {
      for (int col = 0; col < 16; col++)
      {
         robot_row
         robot_col
      }

    }
  if*/
}

void processKey(char key)
{
  switch (key)
  {
    case KEY_FORWARD:
      forward();
      break;
    case KEY_LEFT:
      left();
      break;
    case KEY_RIGHT:
      right();
      break;
    case KEY_READ_DISTANCE:
      DISTANCE_UNIT = getDistanceUnit();
      chprintf(BT, " Distance unit %d!\n\r", DISTANCE_UNIT);
      break;
    case KEY_SENSE:
      while (true) sense();
      break;
    case KEY_PRINT:
      print_array(SHOW_THE_ROBOT);
      break;
    case KEY_MEASURE:
      measure();
      break;
    case KEY_FUN:
      fun();
      break;
    case KEY_TRAVEL:
      chprintf(BT, " Current location: [%d,%d]  heading: dx=%d, dy=%d!\n\r", robot_col, robot_row, dx, dy);
      chprintf(BT, " Enter Destination: col=");
      int dcol = readKey() - '0';
      chprintf(BT, "%d row=", dcol);
      int drow = readKey() - '0';
      chprintf(BT, "%d\n\r", drow);
      navigate_to(drow, dcol);
      break;




    default:
      chprintf(BT, "Unknown key %c \n\r", key);
      break;
  }
}


int main(void)
{
  tatrabotInit();

  robot_row = 15;
  robot_col = 8;

  dx = 0;
  dy = -1;

  chprintf(PC, " Hello  hello, this is Tatrabot speaking...\n\r");
  chprintf(BT, " Hello Tatrabot on BT!\n");

  while (true) {
    int key = readKey();
    processKey(key);
  }

//  forward();
//  chThdSleepMilliseconds(2000);
//  right();
//  forward();
//  left();
//  forward();
//  left();
//  forward();
//  left();
//  forward();
//  right();
//  forward();
//  right();
//  forward();
//  right();
//  forward();
//  right();
//  forward();
//  forward();

  while(true)
  {
    chprintf(PC, "1: %d || 2: %d || 3: %d\n\r", measure_distance(0), measure_distance(1), measure_distance(2) );
    chprintf(PC, "%d ", tatra_clock);
    setLED(0, 1);
    chThdSleepMilliseconds(100);
    setLED(0, 0);
    chThdSleepMilliseconds(100);
  }
}





