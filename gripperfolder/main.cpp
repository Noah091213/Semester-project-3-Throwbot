#include "Gripper.h"

int main() {
    Gripper::connectGripper();
    Gripper::grip();
    Gripper::relase();
    Gripper::bye();
    return 0;
}
