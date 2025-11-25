#include "stepper_motor_angle.h"

#define STEPS_PER_REV  200
#define STEP_DELAY_MS  5

void moveStepperToAngle(float target_deg)
{
    static int32_t current_step = 0;
    static uint8_t seq_index = 0;

    const uint8_t seq[4] = {
        0x18,
        0x0A,
        0x06,
        0x14
    };

    while (target_deg < 0.0f)      target_deg += 360.0f;
    while (target_deg >= 360.0f)   target_deg -= 360.0f;

    float steps_f = (target_deg / 360.0f) * (float)STEPS_PER_REV;
    int32_t target_step = (int32_t)(steps_f + 0.5f);

    int32_t current_norm = current_step % STEPS_PER_REV;
    if (current_norm < 0)
        current_norm += STEPS_PER_REV;

    int32_t delta = target_step - current_norm;

    if (delta > (STEPS_PER_REV / 2))       delta -= STEPS_PER_REV;
    else if (delta < -(STEPS_PER_REV / 2)) delta += STEPS_PER_REV;

    int32_t steps_to_move = (delta >= 0) ? delta : -delta;

    for (int32_t i = 0; i < steps_to_move; i++)
    {
        if (delta >= 0)
        {
            seq_index = (seq_index + 1) & 0x03;
            current_step++;
        }
        else
        {
            seq_index = (seq_index + 3) & 0x03;
            current_step--;
        }

        shift595Write(seq[seq_index]);
        delay_ms(STEP_DELAY_MS);
    }
}
