# stepper_motor

## INSTALLATION
Clone this repository into your projects folder

# Add these lines to the CMakeLists.txt of your project
```sh
add_subdirectory(stepper_motor/src)
target_link_libraries(main stepper_motor)
```
# You can edit the Cmakefile as per your convinience
# include the library in your main.c program file eg.
```c
#include "src/motor/include/motor.h"

/* you can write the rest of the program*/
```

## Note
if multiple motors want to be controlled connect the step pins one after another eg. if two step pins want to be connected and if the first pin connected to GPIO1, the second pin should be connected to GPIO2. 

## Example program
```c
#include <stepper_motor.h>

int main(){
    stdio_init_all();
    set_sys_clock_khz(270000, true);

    int dir_pins[4] = {5, 7, 6, 8}; // direction pin can be connected in any order, if four motors are connected four dir pins should be given in a array.
    int step_per_mm[4] = {100, 100, 100, 100}; // you need to set for how many step a mm of distance is crossed.
    motor_reset_all_data(); // clears all the stored data.
    motor_set_pins(0, 3, dir_pins, false); // step pins 0, 1, 2, 3 are connected to four motor drivers.
    set_motor_step_per_mm(step_per_mm);
    // bool endstops[4] = {false, false, false, false};
    // int pins[4] = {0, 0, 0, 0};
    // set_endstops(endstops, pins);

    double jerk [4]= {0, 0, 10, 0}; // the unit of jerk is mm/sec^3, the values should be in the order of the motors, the values are set to 0 the function of jerk will be disabled
    double velocity[4] = {50, 10, 20, 10}; // unit: mm/sec
    double acceleration[4] = {0, 0, 100, 0}; // unit: mm/sec^2 if the values are set to 0 the acceleration mode will be turned off and only the velocity mode takes place
    double distance[4] = {100, 100, 100, 100};
    bool enable[4] = {true, false, true, false}; // only the true specified motor alone enabled if false the motor does not move.
    // return 0;
    /*
    the last 6th argument defines whether jerk mode want to enabled or not, and if the motor should be runned immediatly
    the last argument should be given as true
    */
    set_motors_steps(distance, velocity, acceleration, jerk, enable, false);
    motor_run(true); // true is given to wait until the motion completes or false to not to wait and move to the next line
}
```
