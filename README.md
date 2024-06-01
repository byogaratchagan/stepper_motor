# Stepper_motor

A stepper motor step signal generator/stepper motor speed control library for raspberry pi pico(in development)

# Features
- It can support to generate upto 32 step signals. (currently supports 10)
- It can support a maximum upto 16 driver (16 pins for step and 16 pins for dir).
- It supports the control of velocity, acceleration, jerk and distance.
- It can controll all the motors simultaneously.
- It is capable of producting a step signal frequency upto 100 khz(for one stepper motor) by the use of pio and dma control (the max operatable frequency depends on the number of motors connected).

## INSTALLATION
Clone this repository into your projects folder

# Add these lines to the CMakeLists.txt of your project
```cmake
add_subdirectory(stepper_motor/src)
target_link_libraries(main stepper_motor) # i have defined the executable name as main if your project uses different name please change it. 
```
# You can edit the Cmakefile as per your convinience
# include the library in your main.c program file eg.
```c
#include <stepper_motor.h>

/* you can write the rest of the program*/
```

## Note
If multiple motors want to be controlled connect the step pins one after another eg. if two step pins want to be connected and if the first pin connected to GPIO1, the second pin should be connected to GPIO2. 

And the RP2040 chip is need to be overclocked to 270 Mhz you can do this by adding this line to your code 
```c
set_sys_clock_khz(270000, true);
```
## Example program
```c
#include <stepper_motor.h>
#include <stdlib.h>
#include <stdio.h>

int main(){
    stdio_init_all();
    set_sys_clock_khz(270000, true); // overclocking is necessary to work with this library or else timing mismatch can occur.

    int dir_pins[4] = {5, 7, 6, 8}; // direction pin can be connected in any order, if four motors are connected four dir pins should be given in a array.
    int step_per_mm[4] = {100, 100, 100, 100}; // you need to set for how many step a mm of distance is crossed.

    motor_init(1); // sets the step signal min low time in microseconds
    motor_reset_all_data(); // clears all the stored data.
    motor_set_pins(0, 3, dir_pins, false); // step pins 0, 1, 2, 3 are connected to four motor drivers.
    set_motor_step_per_mm(step_per_mm);

    double jerk [4]= {0, 0, 10, 0}; // the unit of jerk is mm/sec^3, the values should be in the order of the motors, the values are set to 0 the function of jerk will be disabled
    double velocity[4] = {50, 10, 20, 10}; // unit: mm/sec
    double acceleration[4] = {0, 0, 100, 0}; // unit: mm/sec^2 if the values are set to 0 the acceleration mode will be turned off and only the velocity mode takes place
    double distance[4] = {100, 100, 100, 100};
    bool enable[4] = {true, true, true, true}; // only the true specified motor alone enabled if false the motor does not move.

    set_motors_mm(distance, velocity, acceleration, jerk, enable, false); // the last argument is set to true if the motor want to be runned immediatly
    motor_run(true); // true is given to wait until the motion completes or false to not to wait and move to the next line
}
```
