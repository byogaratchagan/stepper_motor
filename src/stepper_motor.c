#include "include/stepper_motor.h"
#include "include/stepper_motor_send_data.pio.h"
#include "inttypes.h"

#define MOTOR_PIO_FREQ 45000000

double _distance[32];
double _velocity[32];
double _acceleration[32];
uint64_t *_measurement;

bool _EN_AXIS[32];
bool _JERK_EN;
bool _measure;

// internel variable
uint64_t _possible_d[32];
uint64_t _coast_d[32];
uint64_t _decel_x[32];
uint64_t _accel_x[32];

double _JERK[32];
uint64_t _pt[32];
uint _num_axis;

bool _acceleration_end[32];
bool _acceleration_started;
bool _end_process;
bool _end;
uint64_t _min_p[32];
uint64_t _wait_p[32];
uint64_t _min;
uint64_t _cycle_time;
uint _sm;
uint _offset;
bool _pio_initiated_once;

// motor pins
bool _dir_ctrl_ena;
uint _dir_pins[16];

// dma datas
uint _dma_chan;
uint32_t _data[3];
bool _first_cancel;
int _i;

// hardware data
double _motor_steps_per_mm[32];
uint64_t _motor_low_time;
uint _pio_freq;

// endstops.
bool _endstops_bool[16];
uint _endstops_pins[16];

// extra.
bool _run_until;
bool _homed[16];
bool _detected;
bool _mode;

uint64_t _velocity_delay[32];
uint64_t _acceleration_constant[32];
uint64_t _jerk_constant[32];

uint64_t t = 0;
uint64_t min_wp = 0;
uint64_t min_mp = 0;

#define cstep(x, y) (_mode ? ((uint64_t)((double)x * (double)(_motor_steps_per_mm[y]))) : (uint64_t)x)
#define _MIN(x, y) (x > y ? x: y)

void motor_init(float min_low_time){
    _min = min_low_time * 45;  
}

void set_motor_step_per_mm(uint *steps){
    for (int x = 0; x < _num_axis; ++x){
        _motor_steps_per_mm[x] = steps[x];
    }
}

void set_all_to_zero(){
    _num_axis = 0;
    _cycle_time = 0;
    _data[0] = 0;
    _data[1] = 0;
    _data[2] = 0;

    _first_cancel = true;
    _JERK_EN = false;
    _acceleration_started = false;
    _end_process = true;
    _end = false;
    _dir_ctrl_ena = false;

    for (int x = 0; x < 32; ++x){
        _JERK[x] = 0;
        _velocity[x] = 0;
        _acceleration[x] = 0;
        _distance[x] = 0;
        _EN_AXIS[x] = false;
        _possible_d[x] = 0;
        _coast_d[x] = 0;
        _decel_x[x] = 0;
        _accel_x[x] = 0;
        _pt[x] = 0;
        _acceleration_end[x] = false;
        _min_p[x] = 0;
        _wait_p[x] = 0;
        if (x < 16) _dir_pins[x] = 0;
    }
}

void motor_set_pins(uint stp_strt_pin, uint stp_end_pin, uint *dir_pins, bool dir_ctrl_ena){
    set_all_to_zero();
    
    _num_axis = 0; 
    _num_axis = (stp_end_pin - stp_strt_pin) + (uint)1;
    
    for (int x = 0; x < _num_axis; ++x) _dir_pins[x] = dir_pins[x];

    _dir_ctrl_ena = dir_ctrl_ena;
    if (dir_ctrl_ena){
        for (int x = 0; x < _num_axis; x ++){
            if (gpio_get_function(dir_pins[x]) != GPIO_FUNC_SIO){
                gpio_init(dir_pins[x]);
            }
            gpio_set_dir(dir_pins[x], true);
        }
    }

    _offset = pio_add_program(pio0, &stepper_motor_send_data_program);
    stepper_motor_send_data_program_init(pio0, 0, _offset, stp_strt_pin, _num_axis, (float)clock_get_hz(clk_sys) / (MOTOR_PIO_FREQ * 6));

    pio_sm_set_enabled(pio0, 0, false);
    _pio_initiated_once = true;
    
}

static __inline void motor_fnd(int x){
    uint64_t t;
    // Accelerating.
    if (_accel_x[x] != _possible_d[x]){
        _accel_x[x] += 1;
        if (!_JERK[x]) t = sqrtf((float)_accel_x[x]) * _acceleration_constant[x];
        else t = cbrtf((float)_accel_x[x]) * _jerk_constant[x]; // change the jerk data type in functions and write the formula crctly

        _min_p[x] = _min; // _min_p[z] = _min / _cycle_time; previous line.
        _wait_p[x] = (t - _pt[x])  - _min;
        _pt[x] = t;
    }

    // Coasting.
    else if (_coast_d[x]){
        _min_p[x] = _min; // _min_p[z] = _min / _cycle_time; previous line.
        _wait_p[x] = _velocity_delay[x];  - _min;
        if (!_run_until) _coast_d[x] -= 1;
        else{
            if (_measure){
                _measurement[x] += 1;
            }
        }
    }

    // Decelerating.
    else if (_decel_x[x]){
        _decel_x[x] -= 1;
        _min_p[x] = _min;

        if (!_JERK[x]) t = sqrtf((float)_decel_x[x]) * _acceleration_constant[x];
        else t = cbrtf((float)_decel_x[x]) * _jerk_constant[x];
        
         // _min_p[z] = _min / _cycle_time; previous line.
        _wait_p[x] = (_pt[x] - t);  - _min;
        _pt[x] = t;
    }

    // Ending and resulting null.
    else{
        _wait_p[x] = 0;
        _min_p[x] = 0;
        _acceleration_end[x] = true;
    }

    for (int y = 0; y < _num_axis; y++){
        if (!_acceleration_end[y]) return;
    }
    _end = true;
}

static __inline void motor_accelerate(){
    // first find the value that is lesser than those in wait p
    // second it moves to min right the change the value;
    _data[2] = 0;
    min_wp = 0;
    min_mp = 0;
    register uint x = 0;
    
    // finding the minimum delay.
    for (; x < _num_axis; ++x){
        if (_EN_AXIS[x] && !_acceleration_end[x]){
            if (_wait_p[x]){
                _data[2] = _data[2] | (1 << x);
                if (!min_wp) min_wp = _wait_p[x];
                else min_wp = _MIN(min_wp ,_wait_p[x]);
            }

            else if (_min_p[x]){
                _data[2] = _data[2] & ~(1 << x);
                if (!min_mp) min_mp = _min_p[x];
                else min_mp = _MIN(min_mp ,_min_p[x]);
            }

            else{
                t = 0;
                // Accelerating.
                if (_accel_x[x] != _possible_d[x]){
                    _accel_x[x] += 1;
                   
                    if (_JERK[x])  t = cbrtf((float)_accel_x[x]) * _jerk_constant[x];
                    else t = sqrtf((float)_accel_x[x]) * _acceleration_constant[x]; // change the jerk data type in functions and write the formula crctly

                    _min_p[x] = _min; // _min_p[z] = _min / _cycle_time; previous line.
                    _wait_p[x] = (t - _pt[x]) - _min;
                    _pt[x] = t;
                }

                // Coasting.
                else if (_coast_d[x]){
                    _min_p[x] = _min; // _min_p[z] = _min / _cycle_time; previous line.
                    _wait_p[x] = _velocity_delay[x] - _min;
                    if (!_run_until) _coast_d[x] -= 1;
                    else if (_measure) _measurement[x] += 1;
                }

                // Decelerating.
                else if (_decel_x[x]){
                    _decel_x[x] -= 1;
                    _min_p[x] = _min;

                    if (_JERK[x]) t = cbrtf((float)_decel_x[x]) * _jerk_constant[x];
                    else t = sqrtf((float)_decel_x[x]) * _acceleration_constant[x];
                    
                    // _min_p[z] = _min / _cycle_time; previous line.
                    _wait_p[x] = (_pt[x] - t) - _min;
                    _pt[x] = t;
                }

                // Ending and resulting null.
                else{
                    _wait_p[x] = 0;
                    _min_p[x] = 0;
                    _acceleration_end[x] = true;
                }

                _data[2] = _data[2] | (1 << x);
                if (!min_wp) min_wp = _wait_p[x];
                else min_wp = _MIN(min_wp ,_wait_p[x]);
            }
        }
    }

    _end = true;
    for (x = 0; x < _num_axis; ++x){
        if (!_acceleration_end[x]){
            _end = false;
            if (min_mp < min_wp){
                if (min_mp){
                    _data[1] = min_mp;
                    for (x = 0; x < _num_axis; ++x){
                        if (_wait_p[x]) _wait_p[x] -= min_mp;
                        else _min_p[x] -= min_mp;
                    }
                }
                else{
                    _data[1] = min_wp;
                    for (x = 0; x < _num_axis; ++x){
                        if (_wait_p[x]) _wait_p[x] -= min_wp;
                        else _min_p[x] -= min_wp;
                    }
                }
            } 
            else{
                if (min_wp){
                    _data[1] = min_wp;
                    for (x = 0; x < _num_axis; ++x){
                        if (_wait_p[x]) _wait_p[x] -= min_wp;
                        else _min_p[x] -= min_wp;
                    }
                }
                else{
                    _data[1] = min_mp;
                    for (x = 0; x < _num_axis; ++x){
                        if (_wait_p[x]) _wait_p[x] -= min_mp;
                        else _min_p[x] -= min_mp;
                    }
                }
            } 
            _data[0] = 0; 
            return;
        }
    }

    _data[2] = 0;
    _data[1] = 0;
    _data[0] = 1;
}

void motor_write_dma(){
    if (!_end){
        // first find the value that is lesser than those in wait p
        // second it moves to min right the change the value;
        _data[2] = 0;
        min_wp = 0;
        min_mp = 0;
        uint x = 0;
        
        // finding the minimum delay.
        for (; x < _num_axis; ++x){
            if (_EN_AXIS[x] && !_acceleration_end[x]){
                if (_wait_p[x]){
                    _data[2] |= (1 << x);
                    if (!min_wp) min_wp = _wait_p[x];
                    else min_wp = _MIN(min_wp ,_wait_p[x]);
                }

                else if (_min_p[x]){
                    _data[2] &= ~(1 << x);
                    if (!min_mp) min_mp = _min_p[x];
                    else min_mp = _MIN(min_mp ,_min_p[x]);
                }

                else{
                    t = 0;
                    // Accelerating.
                    if (_accel_x[x] != _possible_d[x]){
                        _accel_x[x] += 1;
                    
                        if (_JERK[x])  t = cbrtf((float)_accel_x[x]) * _jerk_constant[x];
                        else t = sqrtf((float)_accel_x[x]) * _acceleration_constant[x]; // change the jerk data type in functions and write the formula crctly

                        _min_p[x] = _min; // _min_p[z] = _min / _cycle_time; previous line.
                        _wait_p[x] = (t - _pt[x]) - _min;
                        _pt[x] = t;
                    }

                    // Coasting.
                    else if (_coast_d[x]){
                        _min_p[x] = _min; // _min_p[z] = _min / _cycle_time; previous line.
                        _wait_p[x] = _velocity_delay[x] - _min;
                        if (!_run_until) _coast_d[x] -= 1;
                        else if (_measure) _measurement[x] += 1;
                    }

                    // Decelerating.
                    else if (_decel_x[x]){
                        _decel_x[x] -= 1;
                        _min_p[x] = _min;

                        if (_JERK[x]) t = cbrtf((float)_decel_x[x]) * _jerk_constant[x];
                        else t = sqrtf((float)_decel_x[x]) * _acceleration_constant[x];
                        
                        // _min_p[z] = _min / _cycle_time; previous line.
                        _wait_p[x] = (_pt[x] - t) - _min;
                        _pt[x] = t;
                    }

                    // Ending and resulting null.
                    else{
                        _wait_p[x] = 0;
                        _min_p[x] = 0;
                        _acceleration_end[x] = true;
                    }

                    _data[2] |= (1 << x);
                    if (!min_wp) min_wp = _wait_p[x];
                    else min_wp = _MIN(min_wp ,_wait_p[x]);
                }
            }
        }

        _end = true;
        for (x = 0; x < _num_axis; ++x){
            if (!_acceleration_end[x]){
                _end = false;
                if (min_mp < min_wp){
                    if (min_mp){
                        _data[1] = min_mp;
                        for (x = 0; x < _num_axis; ++x){
                            if (_wait_p[x]) _wait_p[x] -= min_mp;
                            else _min_p[x] -= min_mp;
                        }
                    }
                    else{
                        _data[1] = min_wp;
                        for (x = 0; x < _num_axis; ++x){
                            if (_wait_p[x]) _wait_p[x] -= min_wp;
                            else _min_p[x] -= min_wp;
                        }
                    }
                } 
                else{
                    if (min_wp){
                        _data[1] = min_wp;
                        for (x = 0; x < _num_axis; ++x){
                            if (_wait_p[x]) _wait_p[x] -= min_wp;
                            else _min_p[x] -= min_wp;
                        }
                    }
                    else{
                        _data[1] = min_mp;
                        for (x = 0; x < _num_axis; ++x){
                            if (_wait_p[x]) _wait_p[x] -= min_mp;
                            else _min_p[x] -= min_mp;
                        }
                    }
                } 
                _data[0] = 0; 
                dma_hw->ints0 = 1u << _dma_chan;
                dma_channel_set_read_addr(_dma_chan, _data, true);
                return;
            }
        }

        _data[2] = 0;
        _data[1] = 0;
        _data[0] = 1;
        dma_hw->ints0 = 1u << _dma_chan;
        dma_channel_set_read_addr(_dma_chan, _data, true);
    }
    else{
        if (_first_cancel){
            _data[0] = 1;
            _data[1] = 10;
            _data[2] = 0;
            _first_cancel = false;
            dma_hw->ints0 = 1u << _dma_chan;
            dma_channel_set_read_addr(_dma_chan, _data, true);
        }
        else{
            _acceleration_started = false;
            _end_process = true;
            pio_sm_set_enabled(pio0, 0, false);
            dma_channel_set_irq0_enabled(_dma_chan, false);
            dma_channel_cleanup(_dma_chan);
            irq_set_enabled(DMA_IRQ_0, false);
            irq_clear(DMA_IRQ_0);
        }
    }
}

void motor_dma(){
    if (!_pio_initiated_once){
        _dma_chan = dma_claim_unused_channel(true);
    }
    dma_channel_config c = dma_channel_get_default_config(_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);

    dma_channel_configure(
        _dma_chan,
        &c,
        &pio0_hw->txf[0],
        NULL,
        3,
        false
    );

    dma_channel_set_irq0_enabled(_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, motor_write_dma);
    irq_set_enabled(DMA_IRQ_0, true);
    motor_write_dma();
}

void motor_reset_all_data(){
    _first_cancel = true;
    _data[0] = 0;
    _data[1] = 0;
    _data[2] = 0;
    _end = false;
    _end_process = false;
    _acceleration_started = false;
    _cycle_time = 45000000 / MOTOR_PIO_FREQ;
    for (int x = 0; x < 32; ++x) _velocity[x] = 0;

    for (int x = 0; x < 32; ++x){
        _JERK[x] = 0;
        _acceleration[x] = 0;
        _velocity[x] = 0;
        _velocity_delay[x] = 0;
        _pt[x] = 0;
        _min_p[x] = 0;
        _wait_p[x] = 0;
        _distance[x] = 0;

        // internel variable
        _possible_d[x] = 0;
        _accel_x[x] = 0;
        _coast_d[x] = 0;
        _decel_x[x] = 0;

        _EN_AXIS[x] = false;
        _acceleration_end[x] = false;
    }
}

void motor_run(bool wait){
    pio_sm_clear_fifos(pio0, 0);
    pio_sm_set_enabled(pio0, 0, true);
    pio_sm_exec(pio0, 0, pio_encode_jmp(4 + _offset));
    motor_dma();
    _end_process = false;
    if (wait){
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        while (_end_process != true){
            tight_loop_contents();
        }
        pio_sm_clear_fifos(pio0, 0);
    }
}

void motor_add_data(double *distance, double *velocity, double *jerk_val, double *accl_val, bool *ena_axis, bool strt_mot){
    int x = 0;

    if (!_acceleration_started){
        motor_reset_all_data();

        for (x = 0; x < _num_axis; x ++){
            _EN_AXIS[x] = ena_axis[x];
            _velocity[x] = velocity[x];
            _acceleration[x] = accl_val[x];
            _JERK[x] = jerk_val[x];
        }

        for (x = 0; x < _num_axis; x ++){
            if (_EN_AXIS[x]){
                double raw_distance = distance[x];
                if (raw_distance < 0){
                    _distance[x] = (uint)(raw_distance * -1);
                    if (_dir_ctrl_ena) gpio_put(_dir_pins[x], false);
                }
                else{
                    _distance[x] = (uint)(raw_distance);
                    if (_dir_ctrl_ena) gpio_put(_dir_pins[x], true);
                }
            }
        }

        bool first = true;

        for (int x = 0; x < _num_axis; ++x){
            if (!_EN_AXIS[x]){
                _possible_d[x] = 0;
                _decel_x[x] = 0;
                _coast_d[x] = 0;
                _coast_d[x] = 0;
                continue;
            }
            if (_acceleration[x] == 0){
                _possible_d[x] = 0;
                _accel_x[x] = 0;
                _decel_x[x] = 0;
                _coast_d[x] = cstep(_distance[x], x);
                _velocity_delay[x] = (uint64_t)(45000000) / cstep(_velocity[x], x);
                continue;
            }
            if (_JERK[x] == 0){
                double min_d = _distance[x];
                double possible_dis = pow(_velocity[x], 2) / (2 * _acceleration[x]);
                if ((possible_dis) > (int)(min_d / 2)){ 
                    double cal_vel = sqrt(_acceleration[x] * min_d);
                    _velocity[x] = (double)cal_vel;
                    _possible_d[x] = (int)(cstep(min_d, x) / 2);
                    _decel_x[x] = (int)(cstep(min_d, x) / 2);
                    _coast_d[x] = 0;
                }
                else{
                    _possible_d[x] = cstep(possible_dis, x);
                    _decel_x[x] = cstep(possible_dis, x);
                    _coast_d[x] = (cstep(_distance[x], x) - (_possible_d[x] * 2));
                }
            }
            else{
                double time1 = sqrt((_velocity[x] / (double)_JERK[x]) * 2);
                int min_d = _distance[x];
                double min_dis = (double)((double)_JERK[x] * pow(time1, 3)) / (double)6;

                if (min_dis >= (int)(min_d / 2)){
                    double time = cbrt((((double)min_d / (double)2) / (double)_JERK[x]) * (double)6);
                    _velocity[x] = (_JERK[x] * pow(time, 2)) / (double)2;
                    double distance = ((_JERK[x] * pow(time, 3)) / (double)(6));
                    _possible_d[x] = cstep(distance, x);
                    _decel_x[x] = cstep(distance, x);
                    _coast_d[x] = cstep(_distance[x], x) - (cstep(distance, x) * 2);
                }
                else{
                    _possible_d[x] = cstep(min_dis, x);
                    _decel_x[x] = cstep(min_dis, x);
                    _coast_d[x] = cstep(_distance[x], x) - (cstep(min_dis, x) * 2);
                }
            }
            _velocity_delay[x] = (uint64_t)(45000000) / cstep(_velocity[x], x);
            _acceleration[x] = (double)cstep(_acceleration[x], x) * 0.5;
            _JERK[x] = (double)cstep(_JERK[x], x) / 6;
            _acceleration_constant[x] = (1 / sqrt(_acceleration[x])) * 45000000;
            _jerk_constant[x] = (1 / cbrt(_JERK[x])) * 45000000;
        }

        for (x = 0; x < _num_axis; ++x){
            motor_fnd(x); // update next pulse.
        }

        _acceleration_started = true;

        if (strt_mot){
            motor_run(true);
        }
    }
}

bool is_motor_running(){
    return !_end_process;
}

void set_motors_mm(double *distance, double *velocity, double *accl_val, double *jerk_val, bool *ena_axis, bool strt_mot){
    _mode = true;
    _run_until = false;
    _measure = false;
    motor_add_data(distance, velocity, jerk_val, accl_val, ena_axis, strt_mot);
}

void set_motors_steps(double *no_steps, double *steps_per_second, double *steps_per_second_square, double *steps_per_second_cube, bool *ena_axis, bool strt_mot){
    _mode = false;
    _run_until = false;
    _measure = false;
    motor_add_data(no_steps, steps_per_second, steps_per_second_cube, steps_per_second_square, ena_axis, strt_mot);
}

void run_until(bool *axis_bool, double *velocity, bool start){
    uint data1 = 0;
    _run_until = true;
    _measure = false;
    double distance[16] = {};
    double jerk[16] = {};
    double acceleration[16] = {};
    for (int x = 0; x < _num_axis; ++x){
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
    _run_until = true;
    _measure = true;
    _measurement = distance_travelled;
    double distance[16] = {};
    double jerk[16] = {};
    double acceleration[16] = {};
    for (int x = 0; x < _num_axis; ++x){
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
        _run_until = false;
        _end = true;
        _acceleration_started = false;
    }
    pio_sm_clear_fifos(pio0, 0);
}

// In development code.
/*
void auto_home_irq(){
    stop_running();
    for (int x = 0; x < _num_axis; ++x){
        if (_endstops_bool[x]){
            bool data = false;
            for (int y = 0; y < 4; y++){
                data = gpio_get(_endstops_pins[x]);
                if (!data) break;
            }
            if (!data){
                if (!_detected){
                    _detected = true;
                    gpio_set_irq_enabled(_endstops_pins[x], GPIO_IRQ_EDGE_FALL, false);
                }
            }
        }
    }
    // write the code to run if not all homed
}

void auto_home(bool *axis_bool, double *measurement){
    // printf("entered\n");
    bool bool_array[16] = {};
    for (int x = 0; x < 16; ++x) _homed[x] = true;
    for (int x = 0; x < _num_axis; ++x){
        if (_endstops_bool[x]){
            if (axis_bool[x]){
                // printf("detected in axis_bool");
                if (gpio_get(_endstops_pins[x])){
                    _homed[x] = false;
                    bool_array[x] = true;
                    gpio_set_irq_enabled(_endstops_pins[x], GPIO_IRQ_EDGE_FALL, true);
                    // printf("irq eneabled on gpio: %d\n", _endstops_pins[x]);
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
                    while (gpio_get(_endstops_pins[x])){}
                    stop_running();
                    measurement[x] = 0;
                    _homed[x] = true;
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
        if (!_detected) continue;
        for (int x = 0; x < _num_axis; ++x){
            if (!_endstops_bool[x]) continue;
            if (!axis_bool[x]) continue;
            if (!gpio_get(_endstops_pins[x])){
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
                // printf("gpio state = %d\n", gpio_get(_endstops_pins[x]));
                run_until(axis_bool2, velocity2, false);
                motor_run(false);
                // printf("click.\n");
                while (gpio_get(_endstops_pins[x])){}
                stop_running();
                // printf("executed run until\n");
                _homed[x] = true;

                bool not_all_homed = false;
                bool axis[16] = {};
                for (int x = 0; x < _num_axis; ++x){
                    if (_endstops_bool[x]){
                        if (!_homed[x]){
                            axis[x] = !_homed[x];
                            not_all_homed = true;
                        }
                    }
                }
                
                // printf("not all homed? %d\n", not_all_homed);
                if (!not_all_homed){
                    // printf("he gone here only then what??\n");
                    for (int x = 0; x < _num_axis; ++x){
                        if (_endstops_bool[x]) gpio_set_irq_enabled(_endstops_pins[x], GPIO_IRQ_EDGE_FALL, false);
                        // printf("measurement = %d\n", distance_travelled[x]);
                        measurement[x] = distance_travelled[x] / _motor_steps_per_mm[x];
                    }
                    loop = false;
                }
                else{
                    // printf("yeah, it gone back again.\n");
                    double velocity[16] = {};
                    for (int x = 0; x < 16; ++x) velocity[x] = 100;
                    gpio_set_irq_enabled(_endstops_pins[x], GPIO_IRQ_EDGE_FALL, true);
                    r_run_until(axis, distance_travelled, velocity, false);
                    _detected = false;
                }
            }
        }
    }
}

void set_endstops(bool *bool_array, uint *pins){
    bool first = true;
    for (int x = 0; x < 16; ++x){
        _endstops_bool[x] = false;
        _endstops_pins[x] = 0;
    }
    for (int x = 0; x < _num_axis; ++x){
        if (bool_array[x]){
            if (gpio_get_function(pins[x]) != GPIO_FUNC_SIO){
                gpio_init(pins[x]);
                gpio_set_dir(pins[x], false);
                gpio_pull_up(pins[x]);
                if (first){
                    gpio_set_irq_enabled_with_callback(_endstops_pins[x], GPIO_IRQ_EDGE_FALL, false, &auto_home_irq);
                    first = false;
                }
                else gpio_set_irq_enabled(_endstops_pins[x], GPIO_IRQ_EDGE_FALL, false);
            }
            _endstops_bool[x] = bool_array[x];
            _endstops_pins[x] = pins[x];
        }
    }
}
*/
/*
void test_dma(){
    // comment the down from here to 
    _sm = pio_claim_unused_sm(pio0, true);
    uint offset = pio_add_program(pio0, &motor_send_data_program);
    motor_send_data_program_init(pio0, 0, offset, 16, 28, 1, (float)clock_get_hz(clk_sys) / (MOTOR_PIO_FREQ));

    pio_sm_set_enabled(pio0, 0, true);

    motor_dma(true);
    // here
}

void test_function(){
    // motor_fnd.
    motor_reset_all_data();
    _end = false;
    _num_axis = 1;
    _JERK_EN = false;
    _acceleration = 1000;
    _velocity = 100;
    int min_d = 100;
    for (int x = 0; x < 4; ++x){
        _distance[x] = 100;
        _EN_AXIS[x] = true;
    }

    int possible_dis = pow(_velocity, 2) / 2 * _acceleration; 
    printf("possible_dis %d\n", possible_dis);
    printf("half_dis %d\n", (_distance[0] / 2));
    if ((possible_dis) > (int)(_distance[0] / 2)){ 
        int cal_vel = sqrt(((2 * _acceleration) * (int)(min_d / 2)));
        _velocity = cal_vel;
        for (int x = 0; x < _num_axis; ++x){
            if (_EN_AXIS[x]){
                _possible_d[x] = (int)(min_d / 2);
                _decel_x[x] = (int)(min_d / 2);
                _coast_d[x] = 0;
            }
            else{
                _possible_d[x] = 0;
                _decel_x[x] = 0;
                _coast_d[x] = 0;
            }
        }
    }

    else{
        for (int x = 0; x < _num_axis; x ++){
            if (_EN_AXIS[x]){
                _possible_d[x] = possible_dis;
                _decel_x[x] = possible_dis;
                _coast_d[x] = _distance[x] - (2 * _possible_d[x]);
            }
            else{
                _possible_d[x] = 0;
                _decel_x[x] = 0;
                _coast_d[x] = 0;
            }
        }
    }

    printf("n possible distance: %d\n", _possible_d[0]);
    printf("n decel distance: %d\n", _decel_x[0]);
    printf("n coast distnace: %d\n", _coast_d[0]);

    for (int x = 0; x < _num_axis; x ++){
        motor_unp(x);
    }

    // enable_motor_pio();
    _data[0] = 0;
    _data[1] = 0;
    // test_dma();
    motor_dma(true);

    // motor_dma(true);

    // coment -----
    while (_end != true){
        motor_accelerate_new(_data);
        printf("data: %d, delay: %d\n", _data[0], _data[1]);
    }
    // comment ----
    

    // tested!!!.
}

void test_write_dma(){
    // printf("end = %d\n", (int)_end);
    if (_end != true){
        printf("delay: %u, data: %u, end: %d\n", _data[1], _data[2], _data[0]);
        motor_accelerate(_data);
    }
    else{
        if (_first_cancel){
            printf("it continious!!! 282");
            _first_cancel = false;
        }
        else{
            dma_channel_set_irq0_enabled(_dma_chan, false);
            pio_sm_set_enabled(pio0, 0, false);
            printf("it continious!!! 286");
        }
    }

    dma_hw->ints0 = 1u << _dma_chan;
    dma_channel_set_read_addr(_dma_chan, _data, true);
}
*/
