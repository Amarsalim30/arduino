#include "robot_car_config.h"

// PID constants
#define KP 1.0
#define KI 0.0
#define KD 0.0

// Function prototypes
void initializePID();
int calculatePID(int currentError);
