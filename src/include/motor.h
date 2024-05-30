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


void set_motor_step_per_mm(uint *steps);

/*! motor_set_pins
* \brief motor_set_pins() - is a function that set the pins and initialize the required protocols.
* \param stp_strt_pin the starting pin from which the step pin is allocated
* \param stp_end_pin the ending pin of the step pin from the starting pin (NOTE: the pins between the starting pin to ending pin must not be used externally).
* \param dir_pins It is an array of dir pins that can be from any where in the pico board.
* \param dir_ctrl_ena If true then it enables the use of dir_pins, if false it disable the use of dir_pins.
* \sa motor_set_pins()
*/
void motor_set_pins(uint stp_strt_pin, uint stp_end_pin, uint *dir_pins, bool dir_ctrl_ena);

// function to clear all the loaded data.
void motor_reset_all_data();

/*! motor_add_data
* \brief The function loads all the required data to make a move,
* \param distance An array of distance according the number of motor.
* \param velocity Max velocity or speed of the motors.
* \param jerk_val An array of jerk values according the number of motor.
* \param accl_val Max acceleration of the motor.
* \param ena_axis An array of enabled motors.
* \param jerk_ena True to enable the jerk motion, false to enable the acceleration motion.
* \param strt_mot True to start the move, false to not to start the move.
* \sa motor_run()
*/
void set_motors(double *distance, double *velocity, double *jerk_val, double *accl_val, bool *ena_axis, bool jerk_ena, bool strt_mot);

/*! motor_run
* \brief the function starts the motor,
* \param wait if true, it waits till the motor finishes its move, else it will not wait.
* \sa motor_run()
*/
void motor_run(bool wait);

/*! is_motor_running
* \brief returns whether the motor is running. (Note: it only indicate according to the software, not hardware wise).
* \return A bool, true if the motor is running, false if the motor is not running.
* \sa is_motor_running()
*/
bool is_motor_running();

void testing();

void stop_running();

void set_endstops(bool *bool_array, uint *pins);

void auto_home(bool *axis_bool, double *measurement);

void run_until(bool *axis_bool, double *velocity, bool start);

void r_run_until(bool *axis_bool, int *distance_travelled, double *velocity, bool start);

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