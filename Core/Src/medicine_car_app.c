#include "medicine_car_app.h"

#include "medicine_car_config.h"
#include "medicine_car_platform.h"

static uint8_t route_running;

static void wait_delivery_done(void)
{
    uint32_t waited = 0U;

    while ((MedicineCar_ReadDrugPresent() == 0U) &&
           (waited < MED_CAR_DELIVERY_WAIT_TIMEOUT_MS)) {
        MedicineCar_SetRedLed(1U);
        Load(0, 0);
        delay_ms(20U);
        waited += 20U;
    }

    MedicineCar_SetRedLed(0U);
}

static void finish_at_home(uint16_t distance, int pwm)
{
    zhao_bai(distance, pwm);
    Load(0, 0);
    MedicineCar_SetGreenLed(1U);
}

static void deliver_left(uint16_t return_cross_distance, uint16_t home_distance, int pwm)
{
    turn_left();
    xunxian(MED_CAR_DISTANCE_MID, pwm);
    wait_delivery_done();
    diaotou();
    xunxian(return_cross_distance, pwm);
    turn_right();
    finish_at_home(home_distance, 5000);
}

static void deliver_right(uint16_t return_cross_distance, uint16_t home_distance, int pwm)
{
    turn_right();
    xunxian(MED_CAR_DISTANCE_MID, pwm);
    wait_delivery_done();
    diaotou();
    xunxian(return_cross_distance, pwm);
    turn_left();
    finish_at_home(home_distance, 5000);
}

static void shibie(void)
{
    NumBuff[0] = getnum();
    NumBuff[1] = getnum();
    XBuff[0] = 1;
    XBuff[1] = 2;
}

static void shibie_1(void)
{
    NumBuff[0] = getnum();
    NumBuff[1] = getnum();
    XBuff[0] = 2;
    XBuff[1] = 1;
}

static void shibie_34(void)
{
    shibie_1();
}

static void find_1(void)
{
    xunxian(MED_CAR_DISTANCE_FIRST_CHECK, 4000);
    deliver_left(MED_CAR_DISTANCE_MID, MED_CAR_DISTANCE_HOME_WHITE, 4000);
}

static void find_2(void)
{
    xunxian(MED_CAR_DISTANCE_FIRST_CHECK, 4000);
    deliver_right(MED_CAR_DISTANCE_MID, MED_CAR_DISTANCE_HOME_WHITE, 4000);
}

static uint8_t deliver_if_matched(uint16_t return_cross_distance,
                                  uint16_t home_distance,
                                  int pwm)
{
    if (aim == NumBuff[0]) {
        if (XBuff[0] == 1) {
            deliver_left(return_cross_distance, home_distance, pwm);
        } else {
            deliver_right(return_cross_distance, home_distance, pwm);
        }
        return 1U;
    }

    if (aim == NumBuff[1]) {
        if (XBuff[1] == 1) {
            deliver_left(return_cross_distance, home_distance, pwm);
        } else {
            deliver_right(return_cross_distance, home_distance, pwm);
        }
        return 1U;
    }

    return 0U;
}

static void run3_8(void)
{
    xunxian(MED_CAR_DISTANCE_SECOND_CHECK, 4000);
    shibie_1();

    if (deliver_if_matched(MED_CAR_DISTANCE_MID,
                           MED_CAR_DISTANCE_SECOND_CHECK,
                           4000) != 0U) {
        return;
    }

    xunxian(14500U, 4000);
    shibie();
    xunxian(4000U, 4000);

    if (deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS3,
                           MED_CAR_DISTANCE_THIRD_CHECK,
                           4000) != 0U) {
        return;
    }

    xunxian(MED_CAR_DISTANCE_THIRD_CHECK, 4000);
    shibie_1();
    (void)deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS4,
                             MED_CAR_DISTANCE_THIRD_CHECK,
                             4000);
}

static void fahui(void)
{
    xunxian(MED_CAR_DISTANCE_SECOND_CHECK, 5000);

    do {
        shibie_34();
    } while (NumBuff[0] == NumBuff[1]);

    if (deliver_if_matched(MED_CAR_DISTANCE_MID,
                           MED_CAR_DISTANCE_SECOND_CHECK,
                           5000) != 0U) {
        return;
    }

    shibie();
    zhao_bai(3000U, 4000);
    xunxian(4000U, 4000);

    if (deliver_if_matched(MED_CAR_DISTANCE_RETURN_CROSS3,
                           MED_CAR_DISTANCE_THIRD_CHECK,
                           4000) != 0U) {
        return;
    }

    run3_8();
}

void MedicineCar_Init(void)
{
    MedicineCarPlatform_Init();
    route_running = 0U;
    Run_Flag = 0;
    aim = 0;
    Number = 0;
    MedicineCar_SetGreenLed(0U);
}

void MedicineCar_Step(void)
{
    MedicineCarPlatform_Service();

    if ((MedicineCar_ReadDrugPresent() != 0U) && (N != 0)) {
        aim = N;
    }

    if ((Run_Flag == 0) || (route_running != 0U)) {
        Read_Speed();
        delay_ms(MED_CAR_STEP_IDLE_DELAY_MS);
        return;
    }

    route_running = 1U;
    Run_Flag = 0;

    if (aim == 0) {
        Load(0, 0);
    } else if (aim == 1) {
        find_1();
    } else if (aim == 2) {
        find_2();
    } else {
        fahui();
    }

    route_running = 0U;
}

void MedicineCar_RequestRun(uint8_t target_room)
{
    aim = (int)target_room;
    Run_Flag = 1;
}

void MedicineCar_SetRecognizedNumber(uint8_t number)
{
    N = (int)number;
}
