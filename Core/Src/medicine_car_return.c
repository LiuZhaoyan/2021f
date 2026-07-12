#include "medicine_car_return.h"

#include "medicine_car_config.h"
#include "medicine_car_detect.h"
#include "medicine_car_platform.h"

static uint8_t return_stack[MED_CAR_RETURN_STACK_SIZE];
static uint8_t return_depth;

static uint8_t fork_turn_left(void)
{
    return search_line_rotating(MED_CAR_TURN_LEFT_LEFT_PWM,
                                MED_CAR_TURN_LEFT_RIGHT_PWM,
                                MED_CAR_TURN_MIN_MS,
                                MED_CAR_FORK_TURN_TIMEOUT_MS,
                                is_line_left);
}

static uint8_t fork_turn_right(void)
{
    return search_line_rotating(MED_CAR_TURN_RIGHT_LEFT_PWM,
                                MED_CAR_TURN_RIGHT_RIGHT_PWM,
                                MED_CAR_TURN_MIN_MS,
                                MED_CAR_FORK_TURN_TIMEOUT_MS,
                                is_line_right);
}

static uint8_t reverse_turn(uint8_t forward_dir)
{
    if (forward_dir == RETURN_DIR_LEFT) {
        return sensor_turn_right();
    } else {
        return sensor_turn_left();
    }
}

static uint8_t fork_reverse_turn(uint8_t forward_dir)
{
    if (forward_dir == RETURN_DIR_LEFT) {
        return fork_turn_right();
    } else {
        return fork_turn_left();
    }
}

void Return_Init(void)
{
    return_depth = 0U;
}

void Return_Push(uint8_t direction)
{
    if (return_depth < (uint8_t)MED_CAR_RETURN_STACK_SIZE) {
        return_stack[return_depth] = direction;
        return_depth++;
    }
}

uint8_t Return_IsEmpty(void)
{
    return (return_depth == 0U) ? 1U : 0U;
}

static uint8_t return_pop(void)
{
    uint8_t dir;

    if (return_depth == 0U) {
        return RETURN_DIR_LEFT;
    }
    return_depth--;
    dir = return_stack[return_depth];
    return dir;
}

void Return_Execute(uint16_t home_distance, int pwm)
{
    uint8_t retry;

    sensor_diaotou();

    while (!Return_IsEmpty()) {
        uint8_t cross_found;

        cross_found = xunxian(home_distance, pwm);

        if (cross_found != 0U) {
            uint8_t dir = return_pop();

            // move_forward_timed(MED_CAR_CROSS_ADVANCE_MS,
            //                    MED_CAR_CROSS_ADVANCE_PWM);
            reverse_turn(dir);
        } else if (is_fork()) {
            uint8_t dir = return_pop();

            // move_forward_timed(MED_CAR_CROSS_ADVANCE_MS,
            //                    MED_CAR_CROSS_ADVANCE_PWM);
            fork_reverse_turn(dir);
        } else {
            stop(1);
            return;
        }
    }

    for (retry = 0U; retry < (uint8_t)MED_CAR_RETURN_CROSS_RETRY_MAX; retry++) {
        uint8_t cross_found;

        cross_found = xunxian(home_distance, pwm);
        if (cross_found == 0U) {
            return;
        }
        // move_forward_timed(MED_CAR_CROSS_ADVANCE_MS,
        //                    MED_CAR_CROSS_ADVANCE_PWM);
    }

    stop(1);
}
