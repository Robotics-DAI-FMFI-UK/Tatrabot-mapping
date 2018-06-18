#include <stdlib.h>
#include "tatrabot.h"
#include "demo.h"

#define MAP_OBSTACLE_THRESHOLD 3

#define TICK_DISTANCE_IN_CM 0.640625

static int8_t fake_map[17][17] = {
  { 0, 0, 0, 5, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 5, 0, 5, 5, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 5, 0, 5, 0, 0, 5, 0, 0, 5, 0, 0, 0, 0, 0 },
  { 0, 0, 5, 0, 0, 5, 0, 0, 5, 0, 0, 5, 0, 0, 0, 0, 0 },
  { 0, 0, 5, 0, 5, 5, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0 },
  { 0, 0, 5, 0, 0, 5, 5, 5, 5, 5, 5, 5, 0, 0, 0, 0, 0 },
  { 0, 0, 5, 0, 0, 0, 5, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0 },
  { 0, 5, 5, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 5, 0, 0, 0, 0, 5, 0, 0, 5, 5, 5, 0, 0, 0, 0, 0 },
  { 0, 5, 0, 0, 5, 5, 5, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0 },
  { 0, 5, 0, 5, 5, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0 },
  { 0, 5, 0, 0, 5, 0, 0, 5, 5, 5, 5, 5, 5, 5, 0, 0, 0 },
  { 0, 5, 5, 0, 5, 0, 0, 5, 5, 0, 0, 5, 0, 5, 0, 0, 0 },
  { 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 5, 0, 5, 0, 0, 0 },
  { 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 0, 0 },
  { 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0 },
  { 0, 0, 0, 0, 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
};


static int16_t sn[17][17];
static int16_t so[17][17];

static int8_t map[17][17];
static int8_t visited[17][17];
static int8_t steps_from_start[17][17];
static int8_t came_from[17][17];
static int8_t map[17][17];

static int8_t queue_row[16*16];
static int8_t queue_col[16*16];

static int16_t queue_start;
static int16_t queue_end;

static int8_t plan_row[16*16];
static int8_t plan_col[16*16];
static int8_t plan_length;


static int8_t robot_row, robot_col;
static int8_t dx, dy;

#define SHOW_THE_ROBOT 1
#define DO_NOT_SHOW_THE_ROBOT 0


static void prepare_fake_map(void)
{
  for (int r = 0; r < 17; r++)
    for (int c = 0; c < 17; c++)
      map[r][c] = fake_map[r][c];
}

static void follow_line_to_crossing(int rotations, int measuring)
{
  int measured = 0;
  countB=0;
  while (((senseLine() & 1) == 0) && (countB < (unsigned int)rotations))
  {
      if (measuring)
      {
        measured++;
        sense(countB);
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
    updateMap();
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
}

static void left(void){

  setMotor(3, 4000);
  chThdSleepMilliseconds(500);
  follow_line_to_crossing(17, 0);
  setMotor(3, 0);
  setMotor(1, -5500);
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
#define KEY_MAPPRINT 'P'
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

void correct_position_with_movement(int *x, int *y, uint32_t distance)
{
  if (dx == 0) {
    if (dy < 0) {
      // Hore
      *y = *y + distance;
    } else {
      // Dole
      *y = *y - distance;
    }
  } else {
    if (dx < 0) {
      // Dolava
      *x = *x - distance;
    } else {
      // Doprava
      *x = *x + distance;
    }
  }
}

void update_boxes_unseen_with_delta(int sensor, int distance, int delta_r, uint32_t distanceY)
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
      correct_position_with_movement(&x, &y, distance);
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

void update_boxes_seen_with_delta(int sensor, int distance, int delta_r, uint32_t distanceY)
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
    correct_position_with_movement(&x, &y, distance);
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

void update_boxes_unseen(int sensor, int distance, uint32_t distanceY)
{
  update_boxes_unseen_with_delta(sensor, distance, 1, distanceY);
  update_boxes_unseen_with_delta(sensor, distance, -1, distanceY);
}

void update_boxes_seen(int sensor, int distance, uint32_t distanceY)
{
  update_boxes_seen_with_delta(sensor, distance, 1, distanceY);
  update_boxes_seen_with_delta(sensor, distance, -1, distanceY);
}

// pri vzdialenosti 100 CM, od stredu vidi do cone s odchylkou 15 CM do oboch stran
void sense(uint32_t rotates)
{
  uint32_t distanceY = rotates * TICK_DISTANCE_IN_CM;
  for(int i = 0; i < 3; i++){
    int sense_temp = measure_distance(i);
    if (sense_temp != 1000){
      update_boxes_seen(i, sense_temp, distanceY);
      update_boxes_unseen(i, sense_temp, distanceY);
    } else{
      update_boxes_unseen(i, 200, distanceY);
    }
  }
}

void heart_beat(int kind)
{
  sound(200 + kind * 200);
  chThdSleepMilliseconds(5);
  nosound();
  chThdSleepMilliseconds(250);
}

char readKey(void)
{
  char key;
  do {
    key = keyPressed(BT);
    heart_beat(9);
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

int is_on_the_plan(int r, int c)
{
  for (int i = 0; i < plan_length; i++)
    if ((plan_row[i] == r) && (plan_col[i] == c)) return 1;
  return 0;
}


void print_level_of_gray(int level)
{
  switch (level)
  {
  case 0: chprintf(BT, " ");
         break;
  case 1: chprintf(BT, ".");
         break;
  case 2: chprintf(BT, ":");
         break;
  case 3: chprintf(BT, "o");
         break;
  case 4: chprintf(BT, "O");
         break;
  case 5: chprintf(BT, "@");
         break;
  case 6: chprintf(BT, "%c%c%c", 0xE2, 0x96, 0x91);
         break;
  case 7: chprintf(BT, "%c%c%c", 0xE2, 0x96, 0x92);
         break;
  case 8: chprintf(BT, "%c%c%c", 0xE2, 0x96, 0x93);
         break;
  case 9:
  case 10:
    chprintf(BT, "%c%c%c", 0xE2, 0x96, 0x88);
         break;
  case 20: chprintf(BT, "x");
           break;
  }
}



void print_map(uint8_t show_robot)
{
  chprintf(BT, "-----map----------------\n\r");
  for (int row = 0; row < 16; row++)
  {
    for (int col = 0; col < 16; col++)
    {
       if (show_robot && (row == robot_row) && (col == robot_col))
          chprintf(BT, " R%c ", robot_orientation());
       else
       {
          print_level_of_gray(map[row][col]);
          print_level_of_gray(map[row][col]);
          print_level_of_gray(map[row][col]);
          if (is_on_the_plan(row, col))
          {
            chprintf(BT, "*");
          }
          else
          {
            print_level_of_gray(map[row][col]);
          }
       }
    }
    chprintf(BT, "\n\r");
  }
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


void updateMap(void)
{
  for (int row = 0; row < 16; row++)
    {
      for (int col = 0; col < 16; col++)
      {
        int sum = sn[row][col] + so[row][col];
        if (sum == 0)
          map[row][col] = 20;
        else
        map[row][col] = 10*so[row][col]/sum;

      }
    }
}

void measure(void)
{
  sense(0);
  heart_beat(3);
  updateMap();
  heart_beat(5);
}


void initialize_plan_data_structures(void)
{
  for (int row = 0; row < 16; row++)
     for (int col = 0; col < 16; col++)
        visited[row][col] = 0;

  queue_row[0] = robot_row;
  queue_col[0] = robot_col;
  visited[robot_row][robot_col] = 1;
  queue_start = 0;
  queue_end = 1;

}

void insert_into_queue(int from_start, int r, int c, int drow, int dcol)
{
  int value = from_start + abs(drow - r) + abs(dcol - c);
  int i;
  for (i = queue_start; i < queue_end; i++)
  {
    if (value < steps_from_start[queue_row[i]][queue_col[i]] + abs(queue_row[i] - drow) + abs(queue_col[i] - dcol))
      break;
  }
  for (int j = queue_end; j > i; j--)
  {
    queue_row[j] = queue_row[j - 1];
    queue_col[j] = queue_col[j - 1];
  }
  queue_row[i] = r;
  queue_col[i] = c;
  queue_end++;
}


int8_t dir[4][2] = { {-1, 0}, {0, 1}, {1, 0}, {0, -1} };

int create_plan(int drow, int dcol)
{
   int current_row, current_col;

   initialize_plan_data_structures();
   while (queue_end > queue_start)
   {
     current_row = queue_row[queue_start];
     current_col = queue_col[queue_start];
     queue_start++;
     //chprintf(BT, "Q:%d,%d\r\n", current_row, current_col);

     for (int try_direction = 0; try_direction < 4; try_direction++)
     {
       int8_t candidate_row = current_row + dir[try_direction][0];
       int8_t candidate_col = current_col + dir[try_direction][1];
       //chprintf(BT, "  C:%d,%d\r\n", candidate_row, candidate_col);

       if ((candidate_row < 0) || (candidate_row > 15) ||
           (candidate_col < 0) || (candidate_col > 15))
         continue;
       if (map[candidate_row][candidate_col] > MAP_OBSTACLE_THRESHOLD)
         continue;
       if (visited[candidate_row][candidate_col])
         continue;
       //chprintf(BT, "  ok %d, %d\r\n", drow, dcol);
       visited[candidate_row][candidate_col] = 1;
       steps_from_start[candidate_row][candidate_col] = steps_from_start[current_row][current_col] + 1;
       came_from[candidate_row][candidate_col] = try_direction;

       if ((candidate_row == drow) && (candidate_col == dcol))
       {
         queue_start = queue_end;
         break;
       }

       //chprintf(BT, "  ok %d,%d\r\n", queue_start, queue_end);
       insert_into_queue(steps_from_start[candidate_row][candidate_col],
                         candidate_row, candidate_col, drow, dcol);
     }
   }
   if (!visited[drow][dcol]) return 0;
   int step = steps_from_start[drow][dcol];
   current_row = drow;
   current_col = dcol;
   while (step >= 0)
   {
      plan_row[step] = current_row;
      plan_col[step] = current_col;
      step--;
      int going_to = (came_from[current_row][current_col] + 2) % 4;
      //chprintf(BT, "[%d,%d,%d,%d,%d] - ", current_row, current_col, going_to, dir[going_to][0], dir[going_to][0] );
      current_row = current_row + dir[going_to][0];
      current_col = current_col + dir[going_to][1];

   }
   plan_length = steps_from_start[drow][dcol];
   //chprintf(BT, "plan len: %d\r\n", plan_length);
   return 1;
}

#define MOVE_STAY  0
#define MOVE_RIGHT 1
#define MOVE_LEFT  2
#define MOVE_BACK  3

#define INDEX_UP    0
#define INDEX_DOWN  1
#define INDEX_LEFT  2
#define INDEX_RIGHT 3

int direction_correction[4][4] = {
  {MOVE_STAY,  MOVE_BACK,  MOVE_LEFT,  MOVE_RIGHT },  // Sme otoceny hore
  {MOVE_BACK,  MOVE_STAY,  MOVE_RIGHT, MOVE_LEFT  },  // Sme otecny dole
  {MOVE_RIGHT, MOVE_LEFT,  MOVE_STAY,  MOVE_BACK  },  // Sme otoceny dolava
  {MOVE_LEFT,  MOVE_RIGHT, MOVE_BACK,  MOVE_STAY  }   // Sme otoceny doprava
};
// pohyb smerom hore | Pohyb smerom dole | Pohyb smerom dolava |pohyb smerom doprava

int get_move_direction(int y, int x) {
  if (y < 0) return INDEX_UP;
  if (y > 0) return INDEX_DOWN;
  if (x < 0) return INDEX_LEFT;
  return INDEX_RIGHT;
}

void correct_position_for_movement(int move)
{
  switch (move) {
    case MOVE_BACK:
      right();
    case MOVE_RIGHT:
      right();
      break;
    case MOVE_LEFT:
      left();
    default:
      break;
  }
}

void perform_movement(int ddy, int ddx){
  int row = get_move_direction(dy, dx);
  int col = get_move_direction(ddy, ddx);

  int move = direction_correction[row][col];
  correct_position_for_movement(move);

  forward();
}

int not_visited(int r, int c)
{
  if (r < 0 || r >= 17 || c < 0 || c >= 17) {
    return 0;
  }
  return !visited[r][c];
}

void execute_plan(void)
{
  for (int row = 0; row < 16; row++)
  {
    for (int col = 0; col < 16; col++)
      visited[row][col] = 0;
  }

  int next_row, next_col, ddy, ddx;
  for (int i = 0; i < plan_length; i++) {
    next_row = plan_row[i];
    next_col = plan_col[i];

    ddy = next_row - robot_row;
    ddx = next_col - robot_col;
    perform_movement(ddy, ddx);
  }
}


void navigate_to(int drow, int dcol)
{
  prepare_fake_map();
  if (create_plan(drow, dcol))
    execute_plan();
  else
    chprintf(BT, "No trajectory exists.\r\n");
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
      while (true) sense(0);
      break;
    case KEY_PRINT:
      print_array(SHOW_THE_ROBOT);
      break;
    case KEY_MAPPRINT:
      print_map(SHOW_THE_ROBOT);
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
  robot_col = 9;

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





