// ---------------------- includes & config ----------------------
#include "vex.h"
using namespace vex;

brain Brain;
inertial BrainInertial = inertial();
motor MotorLeft      = motor(PORT3,  false);
motor MotorRight     = motor(PORT6,  true);
motor MotorDispense  = motor(PORT1,  false);  // <- change if needed

// ---------------------- misc init ----------------------
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  int seed = int(xAxis + yAxis + zAxis);
  srand(seed);
}

void vexcodeInit() {
  initializeRandomSeed();
}

// ---------------------- setup ----------------------
void configureAllSensors(){
  BrainInertial.calibrate();
  wait(2, seconds);
  BrainInertial.setHeading(0, degrees);
  BrainInertial.setRotation(0, degrees);
  MotorLeft.setPosition(0, turns);
  MotorRight.setPosition(0, turns);
  MotorDispense.setPosition(0, deg);
}

// ---------------------- helpers ----------------------
static inline double clampAbs(double v, double mn, double mx){
  double a = fabs(v);
  if (a < mn) a = mn;
  if (a > mx) a = mx;
  return copysign(a, v);
}

static inline double normDeg(double a){
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// ---------------------- PID rotate to absolute heading ----------------------
bool rotateToHeadingPID(double targetDeg,
                        double kp=1.2, double ki=0.0, double kd=0.12,
                        int timeoutMs=2000, double tolDeg=2.0)
{
  const double maxPct = 70.0;
  const double minPct = 8.0;
  const double iLimit = 25.0;   // integral guard (deg)
  const int    loopDt = 15;     // ms

  // set once; keep while-loop empty of spin() calls
  MotorLeft.setStopping(brake);
  MotorRight.setStopping(brake);
  MotorLeft.spin(forward);
  MotorRight.spin(forward);

  timer t;
  double err      = normDeg(targetDeg - BrainInertial.heading(degrees));
  double prevErr  = err;
  double integral = 0.0;

  while (fabs(err) > tolDeg && t.time(msec) < timeoutMs) {
    if (fabs(err) < iLimit) integral += err * (loopDt/1000.0); else integral = 0.0;
    double derivative = (err - prevErr) / (loopDt/1000.0);

    // control effort (u>0 => need CW; u<0 => need CCW)
    double u = kp*err + ki*integral + kd*derivative;

    // convert to tank turn velocities: left = +u, right = -u
    double leftPct  = clampAbs( u, minPct, maxPct);
    double rightPct = clampAbs(-u, minPct, maxPct);

    MotorLeft.setVelocity(leftPct,  percent);
    MotorRight.setVelocity(rightPct, percent);

    wait(loopDt, msec);
    prevErr = err;
    err = normDeg(targetDeg - BrainInertial.heading(degrees));
  }

  MotorLeft.stop();
  MotorRight.stop();
  return fabs(err) <= tolDeg;
}

// ---------------------- dispense one card (v3: anti-double-deal) ----------------------
const double DEG_PER_CARD = 200.0;      // tune for your feeder
const int    MAX_MS = 240;              // watchdog for dispense motion

void dispenseOneCard(){
  double startDispense = MotorDispense.position(deg);
  MotorDispense.setVelocity(90, percent);
  MotorDispense.spin(forward);

  timer t;
  while ((MotorDispense.position(deg) - startDispense) < DEG_PER_CARD
         && t.time(msec) < MAX_MS) { }

  MotorDispense.stop(brake);

  int dispenseTime = t.time(msec);  // get actual dispense time

  MotorDispense.setVelocity(90, percent);  // same speed as forward
  MotorDispense.spin(reverse);             // spin backwards

  wait(dispenseTime, msec);  // run backwards for same duration

  MotorDispense.stop(brake);
}

// ---------------------- shuffle algorithm helpers ----------------------

// Deal multiple cards to a position
void dealCardsToPosition(double heading, int numCards) {
  rotateToHeadingPID(heading, 1.25, 0.0, 0.12, 2000, 2.0);
  for (int i = 0; i < numCards; i++) {
    dispenseOneCard();
    wait(80, msec);
  }
}

bool allDealt(int seats[],int numplayers, int req) {
  for (int i = 0; i < numplayers; i ++) {
    if (seats[i] != numplayers) return false;
  }

  return true;
}

// ---------------------- random shuffling algorithm ----------------------
void shuffleDeal(int numSeats, int totalCardsPerSeat) {
  // tracks how many cards each seat has received
  int cardsDealt[numSeats];

  for (int i = 0; i < numSeats; i++) {
    cardsDealt[i] = 0;
  }

  while (!allDealt(cardsDealt,numSeats,totalCardsPerSeat)) {
    int index = rand() % numSeats;

    int pileSize = cardsDealt[index];

    if (pileSize!=totalCardsPerSeat) {
      dealCardsToPosition(360.0/numSeats*index,1);
      cardsDealt[index] ++;
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    for (int n = 0; n < numSeats; n++) {
      Brain.Screen.print("%d ", cardsDealt[n]);
    }
  }
}

// mode constants
const int MODE_DEAL = 0;
const int MODE_SHUFFLE = 1;
const int MODE_SORT = 2;

/*
method runs user interface for selecting which process to run
returns an integer indicating mode (deal, shuffle, or sort)
*/
int selectMode() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Press chk to prcd");

  // waits until player presses and releases checkmark
  while (!Brain.buttonCheck.pressing()) {}
  while (Brain.buttonCheck.pressing()) {}

  Brain.Screen.clearScreen();
  
  wait(1,seconds);

  // mode is set to deal as default
  int i = MODE_DEAL; 

  // runs ui process until return statement
  while (true) {
    Brain.Screen.setCursor(1,1);

    // displays the mode options
    // & *** next to the mode selected
    Brain.Screen.print("DEAL");
    if (i == MODE_DEAL) 
      Brain.Screen.print(" ***");
    Brain.Screen.newLine();
    Brain.Screen.print("SHUFFLE");
    if (i == MODE_SHUFFLE) 
      Brain.Screen.print(" ***");
    Brain.Screen.newLine();
    Brain.Screen.print("SORT");
    if (i == MODE_SORT) 
      Brain.Screen.print(" ***");
    Brain.Screen.newLine();

    // waits until any button is pressed
    while(!Brain.buttonLeft.pressing() 
          && !Brain.buttonRight.pressing() 
          && !Brain.buttonCheck.pressing()) {}

    Brain.Screen.clearScreen();

    // if the checkmark is pressed
    if (Brain.buttonCheck.pressing()) {
      // wait until button released and return selected mode
      while (Brain.buttonCheck.pressing()) {}
      return i;
    }

    // changes the current mode based on input
    if (Brain.buttonLeft.pressing()) {
      i --; 
    } else if (Brain.buttonRight.pressing()) {
      i ++;
    }

    // waits until buttons are no longer being pressed
    while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing()) {}

    // overflow conditions
    if (i == -1) {
      i = MODE_SORT;
    } else if (i==3) {
      i = MODE_DEAL;
    }
  }
}

/*
method runs user interface for choosing number of players
taking part in the game. returns a number between 2 and max
*/ 
int getNumPlayers(int max) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Press chk to prcd");

  // waits until the player presses and releases check button
  while (!Brain.buttonCheck.pressing()) {}
  while (Brain.buttonCheck.pressing()) {}

  Brain.Screen.clearScreen();
  
  wait(1,seconds);

  // number of players starts at 2
  int i = 2; 

  // runs ui process until return statement
  while (true) {
    // prompts the user and displays selected number of players
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("within [2,%d]",max);
    Brain.Screen.newLine();
    Brain.Screen.print("%d",i);

    // waits until any button is pressed
    while(!Brain.buttonLeft.pressing() 
          && !Brain.buttonRight.pressing() 
          && !Brain.buttonCheck.pressing()) {}

    Brain.Screen.clearScreen();

    // checks if the checkmark was pressed
    if (Brain.buttonCheck.pressing()) {
      // waits until button is released and then returns # players
      while (Brain.buttonCheck.pressing()) {}
      return i;
    }

    // ensures # players selected stays within bounds
    if (i > 2 && Brain.buttonLeft.pressing()) {
      i --; 
    } else if (i < max && Brain.buttonRight.pressing()) {
      i ++;
    }

    // waits until buttons are released before proceeding
    while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing()) {}

  }
}

// ---------------------- main: random shuffle dealing ----------------------
int main() {
  vexcodeInit();
  configureAllSensors();

  srand(Brain.Timer.time(msec));

  // const int NUM_SEATS = 4;
  // const double seats[NUM_SEATS] = { 0.0, 90.0, 180.0, 270.0};
  // const int CARDS_PER_SEAT = 13;  // e.g., dealing half a deck to each position

  // while (true) {
  //   // Perform one complete shuffle deal
  //   shuffleDeal(seats, NUM_SEATS, CARDS_PER_SEAT);

  //   // Wait before next shuffle (or break if you only want one shuffle)
  //   wait(5, seconds);
  //   Brain.programStop();
  // }

  int mode = selectMode();

  Brain.Screen.setCursor(1,1);

  if (mode == MODE_DEAL) {
    Brain.Screen.print("deal selected");
  } else if (mode == MODE_SHUFFLE) {
    Brain.Screen.print("shuffle selected");
  } else if (mode == MODE_SORT) {
    Brain.Screen.print("sort selected");
  } else {
    Brain.Screen.print("%d", mode);
  }

  wait(1,seconds);

  int players = getNumPlayers(10);

  Brain.Screen.setCursor(1,1);

  Brain.Screen.print("%d",players);

  shuffleDeal(players,3);
  
}