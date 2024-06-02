#include "include/stepper_motor.h"
#include "include/stepper_motor_send_data.pio.h"
#include "inttypes.h"

#define MOTOR_PIO_FREQ 45000000

struct Acc_var{
    double distance[32];
    double velocity[32];
    double acceleration[32];
    uint64_t *measurement;

    bool EN_AXIS[32];
    bool JERK_EN;
    bool measure;

    // internel variable
    uint64_t possible_d[32];
    uint64_t coast_d[32];
    uint64_t decel_x[32];
    uint64_t accel_x[32];

    double JERK[32];
    uint64_t pt[32];
    uint num_axis;

    bool acceleration_end[32];
    bool acceleration_started;
    bool end_process;
    bool end;
    uint64_t min_p[32];
    uint64_t wait_p[32];
    uint64_t min;
    uint64_t cycle_time;
    uint sm;
    uint offset;
    bool pio_initiated_once;

    // motor pins
    bool dir_ctrl_ena;
    uint dir_pins[16];

    // dma datas
    uint dma_chan;
    uint32_t data[3];
    bool first_cancel;
    int i;

    // hardware data
    double motor_steps_per_mm[32];
    uint64_t motor_low_time;
    uint pio_freq;

    // endstops.
    bool endstops_bool[16];
    uint endstops_pins[16];

    // extra.
    bool run_until;
    bool homed[16];
    bool detected;
    bool mode;

    uint64_t velocity_delay[32];

} acc_var;

#define cstep(x, y) (acc_var.mode ? ((uint64_t)((double)x * (double)(acc_var.motor_steps_per_mm[y]))) : (uint64_t)x)

void motor_init(float min_low_time){
    acc_var.min = min_low_time * 45;  
}

void set_motor_step_per_mm(uint *steps){
    for (int x = 0; x < acc_var.num_axis; ++x){
        acc_var.motor_steps_per_mm[x] = steps[x];
    }
}

void set_all_to_zero(){
    acc_var.num_axis = 0;
    acc_var.cycle_time = 0;
    acc_var.data[0] = 0;
    acc_var.data[1] = 0;
    acc_var.data[2] = 0;

    acc_var.first_cancel = true;
    acc_var.JERK_EN = false;
    acc_var.acceleration_started = false;
    acc_var.end_process = true;
    acc_var.end = false;
    acc_var.dir_ctrl_ena = false;

    for (int x = 0; x < 32; ++x){
        acc_var.JERK[x] = 0;
        acc_var.velocity[x] = 0;
        acc_var.acceleration[x] = 0;
        acc_var.distance[x] = 0;
        acc_var.EN_AXIS[x] = false;
        acc_var.possible_d[x] = 0;
        acc_var.coast_d[x] = 0;
        acc_var.decel_x[x] = 0;
        acc_var.accel_x[x] = 0;
        acc_var.pt[x] = 0;
        acc_var.acceleration_end[x] = false;
        acc_var.min_p[x] = 0;
        acc_var.wait_p[x] = 0;
        if (x < 16) acc_var.dir_pins[x] = 0;
    }
}

void motor_set_pins(uint stp_strt_pin, uint stp_end_pin, uint *dir_pins, bool dir_ctrl_ena){
    set_all_to_zero();
    
    acc_var.num_axis = 0; 
    acc_var.num_axis = (stp_end_pin - stp_strt_pin) + (uint)1;
    
    for (int x = 0; x < acc_var.num_axis; ++x) acc_var.dir_pins[x] = dir_pins[x];

    acc_var.dir_ctrl_ena = dir_ctrl_ena;
    if (dir_ctrl_ena){
        for (int x = 0; x < acc_var.num_axis; x ++){
            if (gpio_get_function(dir_pins[x]) != GPIO_FUNC_SIO){
                gpio_init(dir_pins[x]);
            }
            gpio_set_dir(dir_pins[x], true);
        }
    }

    acc_var.offset = pio_add_program(pio0, &stepper_motor_send_data_program);
    stepper_motor_send_data_program_init(pio0, 0, acc_var.offset, stp_strt_pin, acc_var.num_axis, (float)clock_get_hz(clk_sys) / (MOTOR_PIO_FREQ * 6));

    pio_sm_set_enabled(pio0, 0, false);
    acc_var.pio_initiated_once = true;
    
}

static __inline uint64_t motor_fnd(int x){
    uint64_t t;
    uint64_t delay;
    // Accelerating.
    if (acc_var.accel_x[x] != acc_var.possible_d[x]){
        acc_var.accel_x[x] += 1;
        if (acc_var.JERK[x]) t = sqrt((double)(acc_var.accel_x[x]) / acc_var.acceleration[x]) * 45000000;
        else t = cbrt((double)(acc_var.accel_x[x]) / acc_var.JERK[x]) * 45000000; // change the jerk data type in functions and write the formula crctly

        delay = (t - acc_var.pt[x]);
        acc_var.pt[x] = t;
        return delay;
    }

    // Coasting.
    if ((acc_var.accel_x[x] == acc_var.possible_d[x]) && acc_var.coast_d[x]){
        delay = acc_var.velocity_delay[x];
        if (!acc_var.run_until) acc_var.coast_d[x] -= 1;
        else{
            if (acc_var.measure){
                acc_var.measurement[x] += 1;
            }
        }
        return delay;
    }

    // Decelerating.
    if ((acc_var.accel_x[x] == acc_var.possible_d[x]) && !acc_var.coast_d[x] && acc_var.decel_x[x]){
        acc_var.decel_x[x] -= 1;
        if (acc_var.JERK[x]) t = sqrt((double)(acc_var.decel_x[x]) / acc_var.acceleration[x]) * 45000000;
        else t = cbrt((double)(acc_var.decel_x[x]) / acc_var.JERK[x]) * 45000000;
        
        delay = (acc_var.pt[x] - t); 
        acc_var.pt[x] = t;
        return delay;
    }

    // Ending and resulting null.
    if ((acc_var.accel_x[x] == acc_var.possible_d[x]) && !acc_var.coast_d[x] && !acc_var.decel_x[x]){
        return -1;
    }
}

static __inline void motor_unp(int z){
    bool end = true;
    
    if (!acc_var.wait_p[z] && !acc_var.min_p[z]){
        uint64_t fn_delay = motor_fnd(z);
        if (fn_delay != -1){
            acc_var.min_p[z] = acc_var.min; // acc_var.min_p[z] = acc_var.min / acc_var.cycle_time; previous line.
            acc_var.wait_p[z] = fn_delay  - acc_var.min_p[z];
        }
        else {
            acc_var.wait_p[z] = 0;
            acc_var.min_p[z] = 0;
            acc_var.acceleration_end[z] = true;
        }
	}

    for (int y = 0; y < acc_var.num_axis; y++){
        if (!acc_var.acceleration_end[y]) return;
    }

    if (end) acc_var.end = true;
}

static __inline void motor_accelerate(uint32_t *data){
    // first find the value that is lesser than those in wait p
    // second it moves to min right the change the value;
    
    uint32_t pin_data = 0;
    uint64_t min_wp = 0;
    uint64_t min_mp = 0;

    bool first_wp = true;
    bool first_mp = true;

    uint32_t sending_delay = 0;

    register uint x = 0;
    
    // finding the minimum delay.
    for (x = 0; x < acc_var.num_axis; ++x){
        if (acc_var.EN_AXIS[x] && !acc_var.acceleration_end[x]){
            if (acc_var.wait_p[x]){
                pin_data = pin_data | (1 << x);
                if (first_wp){
                    min_wp = acc_var.wait_p[x];
                    first_wp = false;
                }
                else min_wp = MIN(min_wp ,acc_var.wait_p[x]);
            }

            else if (!acc_var.wait_p[x] && acc_var.min_p[x]){
                pin_data = pin_data & ~(1 << x);
                if (first_mp){
                    min_mp = acc_var.min_p[x];
                    first_mp = false;
                }
                else min_mp = MIN(min_mp ,acc_var.min_p[x]);
            }

            else{
                motor_unp(x);
                pin_data = pin_data | (1 << x);
                if (first_wp){
                    min_wp = acc_var.wait_p[x];
                    first_wp = false;
                }
                else min_wp = MIN(min_wp ,acc_var.wait_p[x]);
            }
        }
    }

    if (!acc_var.end){
        if (min_mp < min_wp){
            if (min_mp){
                sending_delay = min_mp;
                for (x = 0; x < acc_var.num_axis; ++x){
                    if (acc_var.wait_p[x]) acc_var.wait_p[x] -= min_mp;
                    else acc_var.min_p[x] -= min_mp;
                }
            }
            else if (!min_mp && min_wp){
                sending_delay = min_wp;
                for (x = 0; x < acc_var.num_axis; ++x){
                    if (acc_var.wait_p[x]) acc_var.wait_p[x] -= min_wp;
                    else acc_var.min_p[x] -= min_wp;
                }
            }
        } 
        if (min_mp >= min_wp){
            if (min_wp != 0){
                sending_delay = min_wp;
                for (x = 0; x < acc_var.num_axis; ++x){
                    if (acc_var.wait_p[x]) acc_var.wait_p[x] -= min_wp;
                    else acc_var.min_p[x] -= min_wp;
                }
            }
            else if (!min_wp && min_mp){
                sending_delay = min_mp;
                for (x = 0; x < acc_var.num_axis; ++x){
                    if (acc_var.wait_p[x]) acc_var.wait_p[x] -= min_mp;
                    else acc_var.min_p[x] -= min_mp;
                }
            }
        } 
    }
    else{
        pin_data = 0;
        sending_delay = 0;
    }

    data[2] = pin_data;
    data[1] = sending_delay;

    if (acc_var.end) data[0] = 1;
    else data[0] = 0; 
}

void motor_write_dma(){
    if (!acc_var.end){
        motor_accelerate(acc_var.data);
        dma_hw->ints0 = 1u << acc_var.dma_chan;
        dma_channel_set_read_addr(acc_var.dma_chan, acc_var.data, true);
    }
    else{
        if (acc_var.first_cancel){
            acc_var.data[0] = 1;
            acc_var.data[1] = 10;
            acc_var.data[2] = 0;
            acc_var.first_cancel = false;
            dma_hw->ints0 = 1u << acc_var.dma_chan;
            dma_channel_set_read_addr(acc_var.dma_chan, acc_var.data, true);
        }
        else{
            acc_var.acceleration_started = false;
            acc_var.end_process = true;
            pio_sm_set_enabled(pio0, 0, false);
            dma_channel_set_irq0_enabled(acc_var.dma_chan, false);
            dma_channel_cleanup(acc_var.dma_chan);
            irq_set_enabled(DMA_IRQ_0, false);
            irq_clear(DMA_IRQ_0);
        }
    }
}

void motor_dma(){
    if (!acc_var.pio_initiated_once){
        acc_var.dma_chan = dma_claim_unused_channel(true);
    }
    dma_channel_config c = dma_channel_get_default_config(acc_var.dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);

    dma_channel_configure(
        acc_var.dma_chan,
        &c,
        &pio0_hw->txf[0],
        NULL,
        3,
        false
    );

    dma_channel_set_irq0_enabled(acc_var.dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, motor_write_dma);
    irq_set_enabled(DMA_IRQ_0, true);
    motor_write_dma();
}

void motor_reset_all_data(){
    acc_var.first_cancel = true;
    acc_var.data[0] = 0;
    acc_var.data[1] = 0;
    acc_var.data[2] = 0;
    acc_var.end = false;
    acc_var.end_process = false;
    acc_var.acceleration_started = false;
    acc_var.cycle_time = 45000000 / MOTOR_PIO_FREQ;
    for (int x = 0; x < 32; ++x) acc_var.velocity[x] = 0;

    for (int x = 0; x < 32; ++x){
        acc_var.JERK[x] = 0;
        acc_var.acceleration[x] = 0;
        acc_var.velocity[x] = 0;
        acc_var.velocity_delay[x] = 0;
        acc_var.pt[x] = 0;
        acc_var.min_p[x] = 0;
        acc_var.wait_p[x] = 0;
        acc_var.distance[x] = 0;

        // internel variable
        acc_var.possible_d[x] = 0;
        acc_var.accel_x[x] = 0;
        acc_var.coast_d[x] = 0;
        acc_var.decel_x[x] = 0;

        acc_var.EN_AXIS[x] = false;
        acc_var.acceleration_end[x] = false;
    }
}

void motor_run(bool wait){
    pio_sm_clear_fifos(pio0, 0);
    pio_sm_set_enabled(pio0, 0, true);
    pio_sm_exec(pio0, 0, pio_encode_jmp(4 + acc_var.offset));
    motor_dma();
    acc_var.end_process = false;
    if (wait){
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        while (acc_var.end_process != true){
            tight_loop_contents();
        }
        pio_sm_clear_fifos(pio0, 0);
    }
}

void motor_add_data(double *distance, double *velocity, double *jerk_val, double *accl_val, bool *ena_axis, bool strt_mot){
    int x = 0;

    if (!acc_var.acceleration_started){
        motor_reset_all_data();

        for (x = 0; x < acc_var.num_axis; x ++){
            acc_var.EN_AXIS[x] = ena_axis[x];
            acc_var.velocity[x] = velocity[x];
            acc_var.acceleration[x] = accl_val[x];
            acc_var.JERK[x] = jerk_val[x];
        }

        for (x = 0; x < acc_var.num_axis; x ++){
            if (acc_var.EN_AXIS[x]){
                int raw_distance = distance[x];
                if (raw_distance < 0){
                    acc_var.distance[x] = (uint)(raw_distance * -1);
                    if (acc_var.dir_ctrl_ena) gpio_put(acc_var.dir_pins[x], false);
                }
                else{
                    acc_var.distance[x] = (uint)(raw_distance);
                    if (acc_var.dir_ctrl_ena) gpio_put(acc_var.dir_pins[x], true);
                }
            }
        }

        bool first = true;

        for (int x = 0; x < acc_var.num_axis; ++x){
            if (!acc_var.EN_AXIS[x]){
                acc_var.possible_d[x] = 0;
                acc_var.decel_x[x] = 0;
                acc_var.coast_d[x] = 0;
                acc_var.coast_d[x] = 0;
                continue;
            }
            if (acc_var.acceleration[x] == 0){
                acc_var.possible_d[x] = 0;
                acc_var.accel_x[x] = 0;
                acc_var.decel_x[x] = 0;
                acc_var.coast_d[x] = cstep(acc_var.distance[x], x);
                acc_var.velocity_delay[x] = (uint64_t)(45000000) / cstep(acc_var.velocity[x], x);
                continue;
            }
            if (acc_var.JERK[x] == 0){
                int min_d = acc_var.distance[x];
                int possible_dis = (int)(pow(acc_var.velocity[x], 2) / (2 * acc_var.acceleration[x]));
                if ((possible_dis) > (int)(min_d / 2)){ 
                    uint64_t cal_vel = (uint64_t)sqrt(((2 * acc_var.acceleration[x]) * (int)(min_d / 2)));
                    acc_var.velocity[x] = (double)cal_vel;
                    acc_var.possible_d[x] = (int)(cstep(min_d, x) / 2);
                    acc_var.decel_x[x] = (int)(cstep(min_d, x) / 2);
                    acc_var.coast_d[x] = 0;
                }
                else{
                    acc_var.possible_d[x] = cstep(possible_dis, x);
                    acc_var.decel_x[x] = cstep(possible_dis, x);
                    acc_var.coast_d[x] = (cstep(acc_var.distance[x], x) - (acc_var.possible_d[x] * 2));
                }
            }
            else{
                double time1 = sqrt((acc_var.velocity[x] / (double)acc_var.JERK[x]) * 2);
                int min_d = acc_var.distance[x];
                double min_dis = (double)((double)acc_var.JERK[x] * pow(time1, 3)) / (double)6;

                if (min_dis >= (int)(min_d / 2)){
                    double time = cbrt((((double)min_d / (double)2) / (double)acc_var.JERK[x]) * (double)6);
                    acc_var.velocity[x] = (uint)((acc_var.JERK[x] * pow(time, 2)) / 2);
                    double distance = ((acc_var.JERK[x] * pow(time, 3)) / (double)(6));
                    acc_var.possible_d[x] = cstep(distance, x);
                    acc_var.decel_x[x] = cstep(distance, x);
                    acc_var.coast_d[x] = cstep(acc_var.distance[x], x) - (cstep(distance, x) * 2);
                }
                else{
                    acc_var.possible_d[x] = cstep(min_dis, x);
                    acc_var.decel_x[x] = cstep(min_dis, x);
                    acc_var.coast_d[x] = cstep(acc_var.distance[x], x) - (cstep(min_dis, x) * 2);
                }
            }
            acc_var.velocity_delay[x] = (uint64_t)(45000000) / cstep(acc_var.velocity[x], x);
            acc_var.acceleration[x] = (double)cstep(acc_var.acceleration[x], x) * 0.5;
            acc_var.JERK[x] = (double)cstep(acc_var.JERK[x], x) * 6;
        }

        for (x = 0; x < acc_var.num_axis; ++x){
            motor_unp(x); // update next pulse.
        }

        acc_var.acceleration_started = true;

        if (strt_mot){
            motor_run(true);
        }
    }
}

bool is_motor_running(){
    return !acc_var.end_process;
}

void set_motors_mm(double *distance, double *velocity, double *accl_val, double *jerk_val, bool *ena_axis, bool strt_mot){
    acc_var.mode = true;
    acc_var.run_until = false;
    acc_var.measure = false;
    motor_add_data(distance, velocity, jerk_val, accl_val, ena_axis, strt_mot);
}

void set_motors_steps(double *no_steps, double *steps_per_second, double *steps_per_second_square, double *steps_per_second_cube, bool *ena_axis, bool strt_mot){
    acc_var.mode = false;
    acc_var.run_until = false;
    acc_var.measure = false;
    motor_add_data(no_steps, steps_per_second, steps_per_second_cube, steps_per_second_square, ena_axis, strt_mot);
}

void run_until(bool *axis_bool, double *velocity, bool start){
    uint data1 = 0;
    acc_var.run_until = true;
    acc_var.measure = false;
    double distance[16] = {};
    double jerk[16] = {};
    double acceleration[16] = {};
    for (int x = 0; x < acc_var.num_axis; ++x){
        if (axis_bool[x]){
            distance[x] = 1;
            acceleration[x] = 0;
            jerk[x] = 0;
        }
        else distance[x] = 0;
    }
    motor_add_data(distance, velocity, jerk, acceleration, axis_bool, false);
    if (start) motor_run(false);
}

void r_run_until(bool *axis_bool, uint64_t *distance_travelled, double *velocity, bool start){
    acc_var.run_until = true;
    acc_var.measure = true;
    acc_var.measurement = distance_travelled;
    double distance[16] = {};
    double jerk[16] = {};
    double acceleration[16] = {};
    for (int x = 0; x < acc_var.num_axis; ++x){
        if (axis_bool[x]){
            distance[x] = 1;
            jerk[x] = 0;
            acceleration[x] = 0;
        }
        else distance[x] = 0;
    }
    motor_add_data(distance, velocity, jerk, acceleration, axis_bool, false);
    if (start) motor_run(false);
}

void stop_running(){
    while (is_motor_running()){
        acc_var.run_until = false;
        acc_var.end = true;
        acc_var.acceleration_started = false;
    }
    pio_sm_clear_fifos(pio0, 0);
}

// In development code.
/*
void auto_home_irq(){
    stop_running();
    for (int x = 0; x < acc_var.num_axis; ++x){
        if (acc_var.endstops_bool[x]){
            bool data = false;
            for (int y = 0; y < 4; y++){
                data = gpio_get(acc_var.endstops_pins[x]);
                if (!data) break;
            }
            if (!data){
                if (!acc_var.detected){
                    acc_var.detected = true;
                    gpio_set_irq_enabled(acc_var.endstops_pins[x], GPIO_IRQ_EDGE_FALL, false);
                }
            }
        }
    }
    // write the code to run if not all homed
}

void auto_home(bool *axis_bool, double *measurement){
    // printf("entered\n");
    bool bool_array[16] = {};
    for (int x = 0; x < 16; ++x) acc_var.homed[x] = true;
    for (int x = 0; x < acc_var.num_axis; ++x){
        if (acc_var.endstops_bool[x]){
            if (axis_bool[x]){
                // printf("detected in axis_bool");
                if (gpio_get(acc_var.endstops_pins[x])){
                    acc_var.homed[x] = false;
                    bool_array[x] = true;
                    gpio_set_irq_enabled(acc_var.endstops_pins[x], GPIO_IRQ_EDGE_FALL, true);
                    // printf("irq eneabled on gpio: %d\n", acc_var.endstops_pins[x]);
                }
                else{
                    bool_array[x] = false;

                    double distance[16], velocity[16], acceleration[16], jerk[16];
                    distance[x] = -10, velocity[x] = 10, jerk[x] = 0;
                    
                    double velocity2[16];
                    velocity2[x] = 3;
                    
                    bool axis_bool2[16];
                    axis_bool2[x] = true;

                    motor_add_data(distance, velocity, acceleration, jerk, axis_bool2, true); // retract.
                    run_until(axis_bool2, velocity2, false);
                    motor_run(false);
                    while (gpio_get(acc_var.endstops_pins[x])){}
                    stop_running();
                    measurement[x] = 0;
                    acc_var.homed[x] = true;
                }
            }
        }
    }
    // printf("going running\n");
    double velocity[16] = {};
    uint64_t distance_travelled[16] = {};
    for (int x = 0; x < 16; ++x) velocity[x] = 100;
    r_run_until(bool_array, distance_travelled, velocity, false);
    motor_run(false);
    // printf("running\n");
    bool loop = true;
    
    while (loop){
        if (!acc_var.detected) continue;
        for (int x = 0; x < acc_var.num_axis; ++x){
            if (!acc_var.endstops_bool[x]) continue;
            if (!axis_bool[x]) continue;
            if (!gpio_get(acc_var.endstops_pins[x])){
                // printf("x: %d\n", x);
                double distance[16], velocity[16], jerk[16], acceleration[16];
                distance[x] = -10, velocity[x] = 10, acceleration[x] = 0;
                
                bool axis_bool2[16];
                axis_bool2[x] = true;

                double velocity2[16];
                velocity2[x] = 3;
                
                motor_add_data(distance, velocity, jerk, acceleration, axis_bool2, true);
                // printf("remove hand from the switch\n");
                sleep_ms(2000);
                // printf("gpio state = %d\n", gpio_get(acc_var.endstops_pins[x]));
                run_until(axis_bool2, velocity2, false);
                motor_run(false);
                // printf("click.\n");
                while (gpio_get(acc_var.endstops_pins[x])){}
                stop_running();
                // printf("executed run until\n");
                acc_var.homed[x] = true;

                bool not_all_homed = false;
                bool axis[16] = {};
                for (int x = 0; x < acc_var.num_axis; ++x){
                    if (acc_var.endstops_bool[x]){
                        if (!acc_var.homed[x]){
                            axis[x] = !acc_var.homed[x];
                            not_all_homed = true;
                        }
                    }
                }
                
                // printf("not all homed? %d\n", not_all_homed);
                if (!not_all_homed){
                    // printf("he gone here only then what??\n");
                    for (int x = 0; x < acc_var.num_axis; ++x){
                        if (acc_var.endstops_bool[x]) gpio_set_irq_enabled(acc_var.endstops_pins[x], GPIO_IRQ_EDGE_FALL, false);
                        // printf("measurement = %d\n", distance_travelled[x]);
                        measurement[x] = distance_travelled[x] / acc_var.motor_steps_per_mm[x];
                    }
                    loop = false;
                }
                else{
                    // printf("yeah, it gone back again.\n");
                    double velocity[16] = {};
                    for (int x = 0; x < 16; ++x) velocity[x] = 100;
                    gpio_set_irq_enabled(acc_var.endstops_pins[x], GPIO_IRQ_EDGE_FALL, true);
                    r_run_until(axis, distance_travelled, velocity, false);
                    acc_var.detected = false;
                }
            }
        }
    }
}

void set_endstops(bool *bool_array, uint *pins){
    bool first = true;
    for (int x = 0; x < 16; ++x){
        acc_var.endstops_bool[x] = false;
        acc_var.endstops_pins[x] = 0;
    }
    for (int x = 0; x < acc_var.num_axis; ++x){
        if (bool_array[x]){
            if (gpio_get_function(pins[x]) != GPIO_FUNC_SIO){
                gpio_init(pins[x]);
                gpio_set_dir(pins[x], false);
                gpio_pull_up(pins[x]);
                if (first){
                    gpio_set_irq_enabled_with_callback(acc_var.endstops_pins[x], GPIO_IRQ_EDGE_FALL, false, &auto_home_irq);
                    first = false;
                }
                else gpio_set_irq_enabled(acc_var.endstops_pins[x], GPIO_IRQ_EDGE_FALL, false);
            }
            acc_var.endstops_bool[x] = bool_array[x];
            acc_var.endstops_pins[x] = pins[x];
        }
    }
}
*/
/*
void test_dma(){
    // comment the down from here to 
    acc_var.sm = pio_claim_unused_sm(pio0, true);
    uint offset = pio_add_program(pio0, &motor_send_data_program);
    motor_send_data_program_init(pio0, 0, offset, 16, 28, 1, (float)clock_get_hz(clk_sys) / (MOTOR_PIO_FREQ));

    pio_sm_set_enabled(pio0, 0, true);

    motor_dma(true);
    // here
}

void test_function(){
    // motor_fnd.
    motor_reset_all_data();
    acc_var.end = false;
    acc_var.num_axis = 1;
    acc_var.JERK_EN = false;
    acc_var.acceleration = 1000;
    acc_var.velocity = 100;
    int min_d = 100;
    for (int x = 0; x < 4; ++x){
        acc_var.distance[x] = 100;
        acc_var.EN_AXIS[x] = true;
    }

    int possible_dis = pow(acc_var.velocity, 2) / 2 * acc_var.acceleration; 
    printf("possible_dis %d\n", possible_dis);
    printf("half_dis %d\n", (acc_var.distance[0] / 2));
    if ((possible_dis) > (int)(acc_var.distance[0] / 2)){ 
        int cal_vel = sqrt(((2 * acc_var.acceleration) * (int)(min_d / 2)));
        acc_var.velocity = cal_vel;
        for (int x = 0; x < acc_var.num_axis; ++x){
            if (acc_var.EN_AXIS[x]){
                acc_var.possible_d[x] = (int)(min_d / 2);
                acc_var.decel_x[x] = (int)(min_d / 2);
                acc_var.coast_d[x] = 0;
            }
            else{
                acc_var.possible_d[x] = 0;
                acc_var.decel_x[x] = 0;
                acc_var.coast_d[x] = 0;
            }
        }
    }

    else{
        for (int x = 0; x < acc_var.num_axis; x ++){
            if (acc_var.EN_AXIS[x]){
                acc_var.possible_d[x] = possible_dis;
                acc_var.decel_x[x] = possible_dis;
                acc_var.coast_d[x] = acc_var.distance[x] - (2 * acc_var.possible_d[x]);
            }
            else{
                acc_var.possible_d[x] = 0;
                acc_var.decel_x[x] = 0;
                acc_var.coast_d[x] = 0;
            }
        }
    }

    printf("n possible distance: %d\n", acc_var.possible_d[0]);
    printf("n decel distance: %d\n", acc_var.decel_x[0]);
    printf("n coast distnace: %d\n", acc_var.coast_d[0]);

    for (int x = 0; x < acc_var.num_axis; x ++){
        motor_unp(x);
    }

    // enable_motor_pio();
    acc_var.data[0] = 0;
    acc_var.data[1] = 0;
    // test_dma();
    motor_dma(true);

    // motor_dma(true);

    // coment -----
    while (acc_var.end != true){
        motor_accelerate_new(acc_var.data);
        printf("data: %d, delay: %d\n", acc_var.data[0], acc_var.data[1]);
    }
    // comment ----
    

    // tested!!!.
}

void test_write_dma(){
    // printf("end = %d\n", (int)acc_var.end);
    if (acc_var.end != true){
        printf("delay: %u, data: %u, end: %d\n", acc_var.data[1], acc_var.data[2], acc_var.data[0]);
        motor_accelerate(acc_var.data);
    }
    else{
        if (acc_var.first_cancel){
            printf("it continious!!! 282");
            acc_var.first_cancel = false;
        }
        else{
            dma_channel_set_irq0_enabled(acc_var.dma_chan, false);
            pio_sm_set_enabled(pio0, 0, false);
            printf("it continious!!! 286");
        }
    }

    dma_hw->ints0 = 1u << acc_var.dma_chan;
    dma_channel_set_read_addr(acc_var.dma_chan, acc_var.data, true);
}
*/
