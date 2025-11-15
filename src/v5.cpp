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

// Fisher-Yates shuffle algorithm
void shuffleArray(int arr[], int size) {
  for (int i = size - 1; i > 0; i--) {
    int j = rand() % (i + 1);
    // Swap arr[i] and arr[j]
    int temp = arr[i];
    arr[i] = arr[j];
    arr[j] = temp;
  }
}

// Deal multiple cards to a position
void dealCardsToPosition(double heading, int numCards) {
  rotateToHeadingPID(heading, 1.25, 0.0, 0.12, 2000, 2.0);
  for (int i = 0; i < numCards; i++) {
    dispenseOneCard();
    wait(80, msec);
  }
}

// ---------------------- random shuffling algorithm ----------------------
void shuffleDeal(const double seats[], int numSeats, int totalCardsPerSeat) {
  // Track how many cards each seat has received
  int cardsDealt[numSeats];
  for (int i = 0; i < numSeats; i++) {
    cardsDealt[i] = 0;
  }

  // Create a list of all card "assignments" (which seat gets which card)
  int totalCards = numSeats * totalCardsPerSeat;
  int cardAssignments[totalCards];

  // Fill array: first totalCardsPerSeat entries are for seat 0, next for seat 1, etc.
  for (int i = 0; i < totalCards; i++) {
    cardAssignments[i] = i / totalCardsPerSeat;  // which seat this card goes to
  }

  // Shuffle the assignments - this randomizes the dealing order
  shuffleArray(cardAssignments, totalCards);

  // Now deal cards in the shuffled order, grouping consecutive cards to same seat
  int currentIndex = 0;
  while (currentIndex < totalCards) {
    int targetSeat = cardAssignments[currentIndex];

    // Count how many consecutive cards go to this same seat (random burst size)
    int burstSize = 1;
    while (currentIndex + burstSize < totalCards &&
           cardAssignments[currentIndex + burstSize] == targetSeat) {
      burstSize++;
    }

    // Deal this burst of cards to the target seat
    dealCardsToPosition(seats[targetSeat], burstSize);
    cardsDealt[targetSeat] += burstSize;

    // Move to next card assignment
    currentIndex += burstSize;

    // Optional: small delay between different seat visits
    wait(100, msec);
  }

  // Verification (optional - can be removed for production)
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Shuffle complete!");
  for (int i = 0; i < numSeats; i++) {
    Brain.Screen.setCursor(i + 2, 1);
    Brain.Screen.print("Seat %d: %d cards", i, cardsDealt[i]);
  }
}

const int MODE_DEAL = 0;
const int MODE_SHUFFLE = 1;
const int MODE_SORT = 2;

int selectMode() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Press chk to prcd");

  while (!Brain.buttonCheck.pressing()) {}
  while (Brain.buttonCheck.pressing()) {}

  Brain.Screen.clearScreen();
  
  wait(1,seconds);

  int i = 0; // stores current mode selected 0 indicates top option

  while (true) {
    Brain.Screen.setCursor(1,1);

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

    while(!Brain.buttonLeft.pressing() 
          && !Brain.buttonRight.pressing() 
          && !Brain.buttonCheck.pressing()) {}

    Brain.Screen.clearScreen();

    if (Brain.buttonCheck.pressing()) {
      while (Brain.buttonCheck.pressing()) {}
      return i;
    }

    if (Brain.buttonLeft.pressing()) {
      i --; 
    } else if (Brain.buttonRight.pressing()) {
      i ++;
    }

    while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing()) {}

    // overflow
    if (i == -1) {
      i = MODE_SORT;
    } else if (i==3) {
      i = MODE_DEAL;
    }
  }
}

int getNumPlayers(int max) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Press chk to prcd");

  while (!Brain.buttonCheck.pressing()) {}
  while (Brain.buttonCheck.pressing()) {}

  Brain.Screen.clearScreen();
  
  wait(1,seconds);

  int i = 1; // number of players

  while (true) {
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("within [1,%d]",max);
    Brain.Screen.newLine();
    Brain.Screen.print("%d",i);

    while(!Brain.buttonLeft.pressing() 
          && !Brain.buttonRight.pressing() 
          && !Brain.buttonCheck.pressing()) {}

    Brain.Screen.clearScreen();

    if (Brain.buttonCheck.pressing()) {
      while (Brain.buttonCheck.pressing()) {}
      return i;
    }

    if (i > 1 && Brain.buttonLeft.pressing()) {
      i --; 
    } else if (i < max && Brain.buttonRight.pressing()) {
      i ++;
    }

    while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing()) {}

  }
}

// ---------------------- main: random shuffle dealing ----------------------
int main() {
  vexcodeInit();
  configureAllSensors();

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
}
