#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdio.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <stdlib.h>
#include <pico.h>
#include <hardware/irq.h>
#include <hardware/dma.h>
#include <hardware/pio.h>
#include <inttypes.h>
#include <math.h>
#include <hardware/clocks.h>


/*! motor_init
* \brief initiate the stepper motor library with the minimum step pin low time in microseconds, to find the min step low time see the datasheet of the stepper motor driver.
* \param min_low_time give the min step low time in microseconds.
*/
void motor_init(float min_low_time);

void set_motor_step_per_mm(uint *steps);

/*! motor_set_pins
* \brief motor_set_pins() - is a function that set the pins and initialize the required protocols.
* \param stp_strt_pin the starting pin from which the step pin is allocated
* \param stp_end_pin the ending pin of the step pin from the starting pin (NOTE: the pins between the starting pin to ending pin must not be used externally).
* \param dir_pins It is an array of dir pins that can be from any where in the pico board.
* \param dir_ctrl_ena If true then it enables the use of dir_pins, if false it disable the use of dir_pins.
* \sa motor_set_pins()
*/
void motor_set_pins(uint stp_strt_pin, uint stp_end_pin, uint *dir_pins, bool dir_ctrl_ena); // checked.

// function to clear all the loaded data.
void motor_reset_all_data(); // checked.

/*! set_motors_mm
* \brief The function loads all the required data to make a move,
* \param distance (array) An array of distance according the number of motor.
* \param velocity (array) Max velocity or speed of the motors.
* \param jerk_val (array) An array of jerk values according the number of motor.
* \param accl_val (array) Max acceleration of the motor.
* \param ena_axis (array) An array of enabled motors.
* \param strt_mot (bool)  True to start the move and wait, false to not to start the move.
* \sa motor_run()
*/
void set_motors_mm(double *distance, double *velocity, double *jerk_val, double *accl_val, bool *ena_axis, bool strt_mot); // checked.

/*! set_motors_steps
* \brief The function loads all the required data to make a move,
* \param no_steps (array) An array of number of steps according the number of motor.
* \param steps_per_second (array) Max velocity or speed of the motors.
* \param steps_per_second_square(array) Max acceleration of the motor.
* \param steps_per_second_cube(array) An array of jerk values according the number of motor.
* \param ena_axis (array) An array of enabled motors.
* \param strt_mot (bool) True to start the move and wait, false to not to start the move.
* \sa motor_run()
*/
void set_motors_steps(double *no_steps, double *steps_per_second, double *steps_per_second_square, double *steps_per_second_cube, bool *ena_axis, bool strt_mot); // checked


void motor_run(bool wait); // checked.

/*! is_motor_running
* \brief returns whether the motor is running. (Note: it only indicate according to the software, not hardware wise).
* \return A bool, true if the motor is running, false if the motor is not running.
* \sa is_motor_running()
*/
bool is_motor_running(); // checked.

void stop_running(); // checked.

void set_endstops(bool *bool_array, uint *pins); // unchecked.

void auto_home(bool *axis_bool, double *measurement); // unchecked.

void run_until(bool *axis_bool, double *velocity, bool start); // checked.

/*! r_run_until(), 
* \brief once this function is called the motor runs until stop_running() is called while running it stores the distance travelled in the distance_travelled argument as no of steps.
* \param axis_bool (array) which indicates which motor should do the job if false the motor does not take place in motion.
* \param distance_travelled (array) where the measurement will be stored, it will add 1 to the corresponding motor index after completing each step.
* \param velocity (array) velocity at which the motors run (mm/sec) 
* \param start (bool) to start the motor immediatly (note this function does not wait)
* \sa is_motor_running()
*/
void r_run_until(bool *axis_bool, uint64_t *distance_travelled, double *velocity, bool start); // checked.

// unaccessable function.
/*
void motor_accelerate(uint32_t *data);
void enable_motor_pio();
void motor_dma();
void motor_write_dma();
int64_t motor_fnd(int x);
void motor_unp(int z);
uint32_t motor_dm(int specific_axis);
uint32_t motor_nth_bit(uint32_t integer, int n);
void test_function();
void test_dma();
*/

#endif