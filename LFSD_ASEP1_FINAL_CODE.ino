#include <SoftwareSerial.h>

SoftwareSerial BT(9, 10);  //BT(RX, TX);
unsigned long last_ping = 0;

//  TB6612 MOTOR PINS
#define PWMA 11  // Left motor PWM
#define AIN1 12
#define AIN2 13

#define PWMB 5  // Right motor PWM
#define BIN1 7
#define BIN2 6

//  TURN DEFS AND VARIABLES
#define turn_speed 200
#define delay_before_turn 100
#define platform_confirm_count 500
char turn = 'N';
#define just_follow_duration 1300

//  SENSOR Variables
int s_pins[8] = { 3, 2, A5, A4, A3, A2, A1, A0 };
int S[8] = {};
int s_weights[8] = { -4, -3, -2, -1, 1, 2, 3, 4 };
int read_sum = 0;

//  PID VALUES
float kp = 35.0;
float kd = 8.0;
float ki = 0.0;

const int base_speed = 190;
float error = 0, last_error = 0;

//  NAVIGATION STATES
enum roboState {
  IDLE,
  MOVING,
  UTURN,
  INTURN
};

roboState state = IDLE;

enum intersectionType {
  NONE,
  T_VERTICAL,
  T_CAP,
  CROSS_INT
};

intersectionType last_intersection = NONE;

enum intTurn {
  STRAIGHT,
  LEFT,
  RIGHT
};

intTurn int_decision_plan[5] = {};
intTurn pending_turn = STRAIGHT;

// DISTANCE CALIBERATION PARAMETERS
uint16_t test_time = 1650;
int measured_distance = 30; //CM

// NAVIGATION PARAMETERS
char current_location = 'D';
char target_location = 'N'; // OTHER LOCATIONS: '1', '2', '3', 'D'
char final_target = 'N';
bool chaining = false;
unsigned long u_turn_start_time = 0;
unsigned long after_uturn_reverse_time = 0;
unsigned long just_follow_time = 0;
int intersection_count = 0;
char requested_location = 'N';

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Enable TB6612
  pinMode(4, OUTPUT);  // STBY pin
  digitalWrite(4, HIGH);

  for (int i = 0; i < 8; i++) {
    pinMode(s_pins[i], INPUT);
  }

  BT.println("#DUMP(by default)");
  BT.println("@");
}

void loop() {

  // CONNECTION STATUS CHECK
  if (millis() - last_ping > 1000) {
    BT.println("PING");
    last_ping = millis();
  }
  
  // REACTING TO BT COMMANDS
  if (BT.available() > 0) {
    String cmd = BT.readStringUntil('\n');
    cmd.trim();
    Serial.println("Recieved: " + cmd);

    if (cmd == "C1") {
      planner('1');
      justFollowCommand(just_follow_duration);
    }
    if (cmd == "C2") {
      planner('2');
      justFollowCommand(just_follow_duration);
    }
    if (cmd == "C3") {
      planner('3');
      justFollowCommand(just_follow_duration);
    }
    if (cmd == "D") {
      planner('D');
      justFollowCommand(just_follow_duration);
    }
    if (cmd == "G") {
      state = MOVING;
    }
    if (cmd == "S") {
      state = IDLE;
    }
  }  

  // ROBO IN IDLE STATE
  if (state == IDLE) {
    stopAll();
    return;
  }

  // READING THE LINE
  read_sum = 0;
  for (int i = 0; i < 8; i++) {
    S[i] = digitalRead(s_pins[i]);
    read_sum += S[i];
  }

  // NOTING THE PATTERN (LEFT TO RIGHT) (ye muze bhi abtak nhi smza)
  byte pattern = 0;
  for (int i = 0; i < 8; i++) {
    pattern = (pattern << 1) | S[i];
  }

  //PLATFORM / VERTICAL T DETECTION WITH DIFFERENT APPROACH
  if (state == MOVING) {
    if (read_sum == 8) {
      distance(5);
      readLine();
      if (read_sum == 8) { //MEANS IT'S A PLATFORM
        state = UTURN;
        u_turn_start_time = millis();
        intersection_count = 0;
        BT.println("@");
      }

      else if (read_sum == 0) { //MEANS IT'S A VETICAL T POINT
        intersection_count++;
        last_intersection = T_VERTICAL;
        intDecision(current_location, target_location, intersection_count);
        state = INTURN;
        BT.println("@Intersection: " + String(intersection_count) + " (vertical T)");
      }

      else if (read_sum <= 3) { //MEANS IT'S A CROSS INTERSECTION / NOT CURRENTLY IN TRACK / JUST FOR TESTING
        intersection_count++;
        last_intersection = CROSS_INT;
        intDecision(current_location, target_location, intersection_count);
        state = INTURN;
        turnIntLeft();
        BT.println("@It's a cross intersection :)");
        turn = 'N';
        state = IDLE;
      }
    }
  }

  // UTURN LOGIC
  if (state == UTURN) {

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 120);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 120);

    // turn for fixed time (~180°)
    if ((millis() - u_turn_start_time) > 2400) {
      stopAll();
      after_uturn_reverse_time = millis();
      while (millis() - after_uturn_reverse_time < 1400) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, 120);

        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMB, 120);
      }

      current_location = target_location;
      target_location = 'N';
      //CHAIN REACTION
      if (current_location == 'D' && chaining) {
        state = MOVING;
        justFollowCommand(just_follow_duration);
        target_location = final_target;
        final_target = 'N';
        chaining = false;
        return;
      }
      if (current_location == '1') BT.println("#Cabin1");
      if (current_location == '2') BT.println("#Cabin2");
      if (current_location == '3') BT.println("#Cabin3");
      if (current_location == 'D') BT.println("#Dump");
                 
      state = IDLE;
      return;
    }

    return;
  }  

  // DETECTING TURNS
  if (pattern == 0b11110000 || pattern == 0b11111000 || pattern == 0b11111100) {
    turn = 'L';
   } else if (pattern == 0b00001111 || pattern == 0b00011111 || pattern == 0b00111111) {
    turn = 'R';
  }

  //CAP T DETECTION USING DIFFERENT APPROACH
  if (turn != 'N') {
    stopAll();
    delay(50);
    readLine();
    if (read_sum == 8) {
      return;
    }
    distance(5);
    readLine();
    if (read_sum == 2 || read_sum == 1) { // MEANS IT'S A T_CAP INTERSECTION
      intersection_count++;
      last_intersection = T_CAP;
      intDecision(current_location, target_location, intersection_count);
      state = INTURN;
      BT.println("@Intersection: " + String(intersection_count) + " (cap T)");
    }
  }

  if (state == INTURN) {
    if (pending_turn == LEFT) {
      turnIntLeft();
      turn = 'N';
    } 
    else if (pending_turn == RIGHT) {
      turnIntRight();
      turn = 'N';
    } 
    else {
      // STRAIGHT → do nothing, just continue
      turn = 'N';
    }
  
    state = MOVING;
    return;
  }

  // === LOST LINE DETECTION ===
  static int lost_count = 0;
  if (read_sum == 0) lost_count++;
  else lost_count = 0;

  if (lost_count > 10) {
    stopAll();
    state = IDLE;
    lost_count = 0;
    BT.println("@");

    return;
  }

  // EXECUTING TURNS
  if (read_sum == 0 && turn != 'N') {
    // delay(delay_before_turn);
    if (turn == 'L') {
      turnLeft();
    }
    if (turn == 'R') {
      turnRight();
    }

    return;
  }

  // ===== PID LOGIC =====
int weighted_sum = 0;

  for (int i = 0; i < 8; i++) {
    weighted_sum += (S[i] * s_weights[i]);
  }

  error = (float)weighted_sum / read_sum;

  float derivative = error - last_error;
  last_error = error;

  float correction = kp * error + kd * derivative;
  correction = constrain(correction, -180, 180);

  // === SLOW DOWN ON SHARP TURNS ===
  int dynamic_base = base_speed;
  if (abs(error) > 3) dynamic_base = 70;

  int left_speed = constrain(dynamic_base + correction, 60, 180);
  int right_speed = constrain(dynamic_base - correction, 60, 180);

  setMotors(left_speed, right_speed);

}

// ===== MOTOR FUNCTIONS =====
void setMotors(int left, int right) {
  // Left motor forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, left);

  // Right motor forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, right);
}

void stopAll() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void turnLeft() {
  do {
    //left motor anti clockwise
    analogWrite(PWMA, 0);

    //right motor clockwise
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, turn_speed);

    //keeping the reading going
    readLine();

  } while (S[3] != 1);
  turn = 'N';
}

void turnRight() {
  do {
    // left motor clockwise
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, turn_speed);

    // right motor anti clockwise
    analogWrite(PWMB, 0);

    //keeping the reading process going
    readLine();

  } while (S[4] != 1);
  turn = 'N';
}

//RIGHT LEFT TURNS AT INTERSECTION
void turnIntLeft() {
  if (last_intersection != CROSS_INT) BT.println("@INT: turning left");

  // Phase 1: lose old line
  while (true) {
    // right motor forward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, turn_speed);

    // left motor 
    analogWrite(PWMA, 0);
    // digitalWrite(AIN1, LOW);
    // digitalWrite(AIN2, HIGH);
    // analogWrite(PWMA, turn_speed);

    readLine();

    if (read_sum == 0) break;
  }

  // Phase 2: find new line
  while (true) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, turn_speed);

    // left motor back
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, turn_speed);

    readLine();

    if (S[3] == 1 || S[2] == 1) break;
  }

  turn = 'N';
}


void turnIntRight() {
  BT.println("@INT: turning right");

  // Phase 1: rotate until OLD line is lost
  while (true) {
    // left motor forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, turn_speed);

    // right motor 
    analogWrite(PWMB, 0);

    readLine();

    if (read_sum == 0) break;   // old line gone
  }

  // Phase 2: keep rotating until NEW line appears
  while (true) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, turn_speed);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, turn_speed);

    readLine();

    if (S[4] == 1 || S[5] == 1) break;  // new line detected
  }

  turn = 'N';
}


// ===== SENSOR READING FUNCTION ======
void readLine() {
  read_sum = 0;
  for (int i = 0; i < 8; i++) {
    S[i] = digitalRead(s_pins[i]);
    read_sum += S[i];
  }
}

void distanceCaliberation(uint16_t test_time) {
  unsigned long start_time = millis();
  while (millis() - start_time < test_time) {
    setMotors(base_speed, base_speed);
  }
  stopAll();
}

void distance(uint16_t dist) {
  int time_per_cm = test_time / measured_distance;
  unsigned long move_time = dist * time_per_cm;
  unsigned long start_time = millis();

  while(millis()- start_time < move_time) {
    setMotors(base_speed, base_speed);
  }
}

void planner(char req) {
  requested_location = req;

  //CASE 1 ALREADY AT DESTINATION
  if (current_location == requested_location) {
    target_location = 'N';
    chaining = false;
    return;
  }

  //CASE 2 EITHER TARGET OR CURRENT IS DUMP
  if (current_location == 'D' || requested_location == 'D') {
    target_location = requested_location;
    chaining = false;
    state = MOVING;
    return;
  }

  //CASE 3 CABIN TO CABIN
  final_target = requested_location;
  target_location = 'D';
  state = MOVING;
  chaining = true;
}

void intDecision(char current, char target, char int_count) {
  //SETTING THE DECISION PLAN
  getIntDecisionPlan(current, target);
  intDecisionExecuter(int_decision_plan, intersection_count);
}

void getIntDecisionPlan(char current, char target) {
  switch (current) {
    case '1': 
      switch (target) {
        case 'D': int_decision_plan[0] = STRAIGHT; break;
    } break;

    case '2': 
      switch (target) {
        case 'D': int_decision_plan[0] = RIGHT; int_decision_plan[1] = LEFT; break;
    } break;

    case '3': 
      switch (target) {
        case 'D': int_decision_plan[0] = STRAIGHT; int_decision_plan[1] = LEFT; break;
    } break;

    case 'D': 
      switch(target) {
        case '1': int_decision_plan[0] = STRAIGHT; break;
        case '2': int_decision_plan[0] = RIGHT; int_decision_plan[1] = LEFT; break;
        case '3': int_decision_plan[0] = RIGHT; int_decision_plan[1] = STRAIGHT; break;
      } break;
  }      
}

void intDecisionExecuter(intTurn plan[], int int_num) {
  pending_turn = plan[int_num - 1];
}

void justFollowTheLine() {
  // READING THE LINE
  read_sum = 0;
  for (int i = 0; i < 8; i++) {
    S[i] = digitalRead(s_pins[i]);
    read_sum += S[i];
  }

  // NOTING THE PATTERN (LEFT TO RIGHT)
  byte pattern = 0;
  for (int i = 0; i < 8; i++) {
    pattern = (pattern << 1) | S[i];
  }

  // === LOST LINE DETECTION ===
  static int lost_count = 0;
  if (read_sum == 0) lost_count++;
  else lost_count = 0;

  if (lost_count > 10) {
    stopAll();
    state = IDLE;
    lost_count = 0;
    return;
  }

  // DETECTING TURNS
  if (pattern == 0b11110000 || pattern == 0b11111000 || pattern == 0b11111100) {
    turn = 'L';
  } else if (pattern == 0b00001111 || pattern == 0b00011111 || pattern == 0b00111111) {
    turn = 'R';
  }

  // EXECUTING TURNS
  if (read_sum == 0 && turn != 'N') {
    delay(delay_before_turn);
    if (turn == 'L') {
      turnLeft();
    }
    if (turn == 'R') {
      turnRight();
    }
    return;
  }

  // === WEIGHTED POSITION ERROR ===
  int weighted_sum = 0;

  for (int i = 0; i < 8; i++) {
    weighted_sum += (S[i] * s_weights[i]);
  }

  error = (float)weighted_sum / read_sum;

  float derivative = error - last_error;
  last_error = error;

  float correction = kp * error + kd * derivative;
  correction = constrain(correction, -180, 180);

  // === SLOW DOWN ON SHARP TURNS ===
  int dynamic_base = base_speed;
  if (abs(error) > 3) dynamic_base = 70;

  int left_speed = constrain(dynamic_base + correction, 60, 180);
  int right_speed = constrain(dynamic_base - correction, 60, 180);

  setMotors(left_speed, right_speed);
}

void justFollowCommand(int duration) {
  if (state == MOVING) {
    just_follow_time = millis();
    while (millis() - just_follow_time < duration) {
      justFollowTheLine();
    }
    turn = 'N';
    just_follow_time = 0;
  }
}
