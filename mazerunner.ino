#include "systick.h"
#include "simplepid.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "config.h"

// Motor Pins
extern const int enca[NMOTORS]; // only pins 2 and 3 are external interrupt capable on arduino uno
extern const int encb[NMOTORS];
extern const int pwm[NMOTORS]; // blue:10
extern const int in1[NMOTORS];
extern const int in2[NMOTORS]; // green:11

// reached goal
bool is_goal_reached = false;

// Globals
float prevMillis = 0;
int counter = 0;
long target[NMOTORS] = {0, 0};
float cross_track_error = 0;
float error1 = 0;
float error2 = 0;

// PID class instances
extern SimplePID pid[NMOTORS];

int path[PATH_SIZE] = {0};// 1 left, 2 forward, 3 right, 4 backwards
int nodeIdx = 0;// index to last node

// medians of distances from left and right
float distance_left;
float distance_right;

// FSM states
#define FORWARD_STATE 1
#define LEFT_TURN_STATE 2
#define RIGHT_TURN_STATE 3
#define TURN_AROUND_STATE 4

// FSM variables
int state = FORWARD_STATE;
bool is_turning = false;

void setup() {
    Serial.begin(115200); // prev 115200
    setup_systick();

    for (int k = 0; k < NMOTORS; k++) {
        pinMode(enca[k], INPUT);
        pinMode(encb[k], INPUT);
        pinMode(pwm[k], OUTPUT);
        pinMode(in1[k], OUTPUT);
        pinMode(in2[k], OUTPUT);

        pid[k].setParams(kP, kI, kD, 255);
    }
    pinMode(irPin1, INPUT);
    pinMode(irPin2, INPUT);
    pinMode(irPinF, INPUT);
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);

    // TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
    bitClear(TCCR1B, CS11);
    bitSet(TCCR1B, CS10);

    attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);

    Serial.println("starting forward profile");
}

int n = 0;

void loop() {
    getUltrasDistances();
    if (is_turning == false) {

        if (distance_left < 5)
            error1--;
        if (distance_right < 5)
            error1++;
        if (distance_left > 9 && distance_left < 20)
            error2++;
        if (distance_right > 9 && distance_right < 20)
            error2--;
        cross_track_error = error1 + 0.5 * error2;
    }
    Serial.println(measureDistance(echoPin2, trigPin2));
    int irPins[3];
    irPins[0] = digitalRead(irPin1);
    irPins[1] = digitalRead(irPin2);
    irPins[2] = digitalRead(irPinF);


    if (irPins[2] == 0) {
        forward.stop();
    }

    float currMillis = millis() / 1000;
    if (currMillis - prevMillis >= WAITING_TIME) {
        is_turning = false;
        if (is_goal_reached == true) {
            if (currMillis - prevMillis >= 1.7) {
                if (n >= nodeIdx) {
                    delay(500000);
                }
                while (path[n] == 0) {
                    n++;
                }
                if (path[n] == 1) {
                    turn_left(currMillis);
                    is_turning = true;
                    path[n] == 2;
                } else if (path[n] == 2) {
                    move_forward(currMillis);
                    n++;
                } else if (path[n] == 3) {
                    turn_right(currMillis);
                    is_turning = true;
                    path[n] == 2;
                } else if (path[n] == 4) {
                    turn_around(currMillis);
                    path[nodeIdx] = 4;
                    is_turning = true;
                    path[n] == 2;
                }

            }
        } else {

            // FSM for left hand algorithm
            switch (state) {
                case FORWARD_STATE:
                    if (irPins[0] == 1 && path[nodeIdx - 1] != 1 && path[nodeIdx - 1] != 3 && path[nodeIdx - 1] != 4) {
                        state = LEFT_TURN_STATE;
                        turn_left(currMillis);
                        path[nodeIdx] = 1;
                        is_turning = true;
                        update_idx();
                    } else if (irPins[2] == 1) {
                        move_forward(currMillis);
                        path[nodeIdx] = 2;
                        update_idx();
                    } else if (irPins[1] == 1) {
                        state = RIGHT_TURN_STATE;
                        turn_right(currMillis);
                        path[nodeIdx] = 3;
                        is_turning = true;
                        update_idx();
                    } else {
                        state = TURN_AROUND_STATE;
                        turn_around(currMillis);
                        path[nodeIdx] = 4;
                        is_turning = true;
                        update_idx();
                    }
                    break;

                case LEFT_TURN_STATE:
                    if (irPins[2] == 1) {
                        state = 1; // Transition back to move forward state
                        move_forward(currMillis);
                        path[nodeIdx] = 2;
                        update_idx();
                    } else {
                        state = 2; // Stay in left turn state
                    }
                    break;

                case RIGHT_TURN_STATE:
                    if (irPins[2] == 1) {
                        state = 1; // Transition back to move forward state
                        move_forward(currMillis);
                        path[nodeIdx] = 2;
                        update_idx();
                    } else {
                        state = 3; // Stay in turn right state
                    }
                    break;

                case TURN_AROUND_STATE:
                    if (irPins[2] == 1) {
                        state = 1; // Transition back to move forward state
                        move_forward(currMillis);
                        path[nodeIdx] = 2;
                        update_idx();
                    } else {
                        state = 4; // Stay in turn around state
                    }
                    break;
            }
            check_goal();
        }
    }
}

bool compareArrays(int arr1[], int size1, int arr2[], int size2) {
    if (size1 != size2) {
        return false;
    }
    for (int i = 0; i < size1; i++) {
        if (arr1[i] != arr2[i]) {
            return false;
        }
    }
    return true;
}


void check_goal() {
    int goal1[] = {2, 2, 3, 2, 2, 3, 2, 2, 3};
    int goal2[] = {2, 2, 1, 2, 2, 1, 2, 2, 1};
    int currPath[8] = {0};
    bool check1 = false;
    bool check2 = false;
    for (int i = 0; i < nodeIdx; i++) {
        currPath[i % 8] = path[i];
        if (i >= 7) {
            check1 = compareArrays(goal1, 8, currPath, 8);
            check2 = compareArrays(goal2, 8, currPath, 8);
        }
        if (check1 == true or check2 == true) {
            modifyPath();
            for (int i = 0; i < nodeIdx; i++) {
                if (path[i] != 0) {
                    Serial.print(path[i]);
                }
            }
            is_goal_reached = true;
        }
    }
}


void move_forward(float currMillis) {
    forward.start(CELL_DIMENSION * PULSES_PER_CM, FORWARD_SPEED * PULSES_PER_CM, 0, 96 * PULSES_PER_CM);
    prevMillis = currMillis;
// Serial.println("going forward");
}

void turn_left(float currMillis) {
    rotation.start(EIGHTH_TURN_CM * PULSES_PER_CM, TURN_LEFT_RIGHT * PULSES_PER_CM, 0, 96 * PULSES_PER_CM);
    prevMillis = currMillis;
// Serial.println("turning left");
}

void turn_right(float currMillis) {
    rotation.start(-EIGHTH_TURN_CM * PULSES_PER_CM, TURN_LEFT_RIGHT * PULSES_PER_CM, 0, 96 * PULSES_PER_CM);
    prevMillis = currMillis;
// Serial.println("turning right");
}

void turn_around(float currMillis) {
    rotation.start(2 * EIGHTH_TURN_CM * PULSES_PER_CM, TURN_180 * PULSES_PER_CM, 0, 96 * PULSES_PER_CM);
    prevMillis = currMillis;
// Serial.println("going backwards :(");
}

void is_it_finished(Profile motion) {
    while (motion.is_finished() == false) {

    }
}

void update_idx() {
    nodeIdx = (nodeIdx + 1) % PATH_SIZE;
}

// Calculate the median value of an array
float median(float arr[], int size) {
    if (size % 2 == 0) {
        // Average of the two middle values if there are an even number of elements
        return (arr[size / 2 - 1] + arr[size / 2]) / 2.0;
    } else {
        // Middle value if there are an odd number of elements
        return arr[size / 2];
    }
}

void getUltrasDistances() {
    float distances_left[5] = {0};
    float distances_right[5] = {0};

    // Take 10 measurements for each ultrasonic sensor
    for (int i = 0; i < 5; i++) {
        distances_left[i] = measureDistance(echoPin1, trigPin1);
        distances_right[i] = measureDistance(echoPin2, trigPin2);
    }

    // Bubble sort the arrays of distances
    int n = 5;
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (distances_left[j] > distances_left[j + 1]) {
                float temp = distances_left[j];
                distances_left[j] = distances_left[j + 1];
                distances_left[j + 1] = temp;
            }
            if (distances_right[j] > distances_right[j + 1]) {
                float temp = distances_right[j];
                distances_right[j] = distances_right[j + 1];
                distances_right[j + 1] = temp;
            }
        }
    }

    // Calculate the median distance for both left and right
    distance_left = median(distances_left, 5);
    distance_right = median(distances_right, 5);
}

int modifyPath() {
    int num_filled = 0;
    for (int i = 2; i < PATH_SIZE; i++) {
        if (path[i] == 3 && path[i - 1] == 4 && path[i - 2] == 1) {
            path[i] = 4;
            path[i - 1] = 0;
            path[i - 2] = 0;
            i -= 2;
        } else if (path[i] == 2 && path[i - 1] == 4 && path[i - 2] == 1) {
            path[i] = 3;
            path[i - 1] = 0;
            path[i - 2] = 0;
            i -= 2;
        } else if (path[i] == 1 && path[i - 1] == 4 && path[i - 2] == 1) {
            path[i] = 2;
            path[i - 1] = 0;
            path[i - 2] = 0;
            i -= 2;
        } else if (path[i] == 1 && path[i - 1] == 4 && path[i - 2] == 2) {
            path[i] = 3;
            path[i - 1] = 0;
            path[i - 2] = 0;
            i -= 2;
        } else if (path[i] == 2 && path[i - 1] == 4 && path[i - 2] == 2) {
            path[i] = 4;
            path[i - 1] = 0;
            path[i - 2] = 0;
            i -= 2;
        } else if (path[i] == 1 && path[i - 1] == 4 && path[i - 2] == 3) {
            path[i] = 4;
            path[i - 1] = 0;
            path[i - 2] = 0;
            i -= 2;
        }
    }

    for (int i = 0; i < PATH_SIZE; i++) {
        if (path[i] != 0) {
            num_filled++;
        }
    }

    return num_filled;
}

