/*
 * FOC.c - library for implementing a FOC controller
 * Created by Yandong Liu, 20250510
*/

#include "FOC.h"    
#include "defines.h"
#include "FastMath.h"
#include "math.h"

extern void writePwm(uint32_t timer, int32_t duty);

extern volatile int16_t adc_data[64];
extern volatile int16_t adc_os[3];

volatile const uint8_t SVPWM_PermuataionMatrix[6][3] = {
	{ 1, 2, 0 },
	{ 3, 1, 0 },
	{ 0, 1, 2 },
	{ 0, 3, 1 },
	{ 2, 0, 1 },
	{ 1, 0, 3 }
};

void FOC_update(volatile FOC_data* self) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0);
    // FOC
    self->temp_it = self->motor_PhysPosition + self->Encoder_os;
    // compensate for the time delay if motor speed is high
    if (fabsf(self->motor_speed) > ((300.0f/60.0f)*1e-6f*N_STEP_ENCODER * 0.02f)){
        self->temp_it += (TIM2->CNT - self->motor_lastMeasTime + (25 << self->F_sw))*self->motor_speed;
    }
    self->temp_it *= N_POLES;
    self->temp_it %= N_STEP_ENCODER;
    self->motor_ElecPosition = (float)(self->temp_it) / (float)N_STEP_ENCODER;
    // motor_ElecPosition = 0.0f;
    self->sin_elec_position = sinf(self->motor_ElecPosition * PIx2);
    self->cos_elec_position = cosf(self->motor_ElecPosition * PIx2);
    // sin_elec_position = lookupTblf(sin_LookupX, sin_LookupY, math_LookupSize, motor_ElecPosition);
    // cos_elec_position = lookupTblf(sin_LookupX, cos_LookupY, math_LookupSize, motor_ElecPosition);
    // sin_elec_position = flookupTbll(sin_LookupXl, sin_LookupY, math_LookupSize, temp_it);
    // cos_elec_position = flookupTbll(sin_LookupXl, cos_LookupY, math_LookupSize, temp_it);
    // Clarke transform
    self->I_a = self->U_current * 0.66666667f - self->V_current * 0.33333333f - self->W_current * 0.33333333f;
    self->I_b = sqrt3_1o * (self->V_current - self->W_current);
    // Park transform
    self->I_d = self->I_a * self->cos_elec_position + self->I_b * self->sin_elec_position;
    self->I_q = self->I_b * self->cos_elec_position - self->I_a * self->sin_elec_position;
    self->I_d_avg += (self->I_d - self->I_d_avg) * 0.001f;
    self->I_q_avg += (self->I_q - self->I_q_avg) * 0.001f;
    // PI controllers on Q and D
    float RpmSafetyMult = (fabsf(self->motor_speed) > MAX_SPEED * 0.9) ? 
        (MAX_SPEED - fabsf(self->motor_speed)) / MAX_SPEED * 10.0f : 1.0f;
    self->I_d_err = self->TargetFieldWk * RpmSafetyMult - self->I_d;
    self->I_q_err = self->TargetCurrent * RpmSafetyMult - self->I_q;
    self->integ_d += self->I_d_err * self->Ki_Id * 25e-6f * ((float)(1U << self->F_sw));
    self->integ_d = (self->integ_d > MAX_CMD_D) ? MAX_CMD_D : self->integ_d;
    self->integ_d = (self->integ_d < MIN_CMD_D) ? MIN_CMD_D : self->integ_d;
    self->cmd_d = self->I_d_err * self->Kp_Id + self->integ_d;
    self->integ_q += self->I_q_err * self->Ki_Iq * 25e-6f * ((float)(1U << self->F_sw));
    self->integ_q = (self->integ_q > MAX_CMD_Q) ? MAX_CMD_Q : self->integ_q;
    self->integ_q = (self->integ_q < MIN_CMD_Q) ? MIN_CMD_Q : self->integ_q;
    self->cmd_q = self->I_q_err * self->Kp_Iq + self->integ_q;
    // Inverse Park transform
    self->cmd_a = self->cmd_d * self->cos_elec_position - self->cmd_q * self->sin_elec_position;
    self->cmd_b = self->cmd_q * self->cos_elec_position + self->cmd_d * self->sin_elec_position;
    // Inverse Clarke transform
    self->duty_u = self->cmd_a;
    self->duty_v = (self->cmd_a * -0.5f + 0.8660254037844386f * self->cmd_b);
    self->duty_w = (self->cmd_a * -0.5f - 0.8660254037844386f * self->cmd_b);
    writePwm(U_TIMER, self->duty_u * -32000.0f + 32000);
    writePwm(V_TIMER, self->duty_v * -32000.0f + 32000);
    writePwm(W_TIMER, self->duty_w * -32000.0f + 32000);

    // SVPWM generation
    // SVPWM_mag = hypotf(cmd_b, cmd_a);
    // SVPWM_ang = atan2f(cmd_b, cmd_a);
    // SVPWM_ang += PI;
    // SVPWM_sector = (uint8_t)(SVPWM_ang * PI_3o);
    // SVPWM_beta = SVPWM_ang - PIo3 * SVPWM_sector;
    // SVPWM_Tb1 = SVPWM_mag * sinf(PIo3 - SVPWM_beta);
    // SVPWM_Tb2 = SVPWM_mag * sinf(SVPWM_beta);
    // SVPWM_Ti[0] = (1.0f - SVPWM_Tb1 - SVPWM_Tb2) * 0.5f;
    // SVPWM_Ti[1] = SVPWM_Tb1 + SVPWM_Tb2 + SVPWM_Ti[0];
    // SVPWM_Ti[2] = SVPWM_Tb2 + SVPWM_Ti[0];
    // SVPWM_Ti[3] = SVPWM_Tb1 + SVPWM_Ti[0];
    // duty_u = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][0]];
    // duty_v = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][1]];
    // duty_w = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][2]];
    // writePwm(U_TIMER, duty_u * 64000.0f);
    // writePwm(V_TIMER, duty_v * 64000.0f);
    // writePwm(W_TIMER, duty_w * 64000.0f);
    // if (sendCANBus_flag == 0) sendCANBus_flag = 1;
    
    // TxHeaderIT.BitRateSwitch = FDCAN_BRS_OFF;
    // TxHeaderIT.DataLength = FDCAN_DLC_BYTES_8;
    // TxHeaderIT.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    // TxHeaderIT.FDFormat = FDCAN_CLASSIC_CAN;
    // TxHeaderIT.Identifier = CAN_DEBUG_ID;
    // TxHeaderIT.IdType = FDCAN_EXTENDED_ID;
    // TxHeaderIT.MessageMarker = 0;
    // TxHeaderIT.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    // TxHeaderIT.TxFrameType = FDCAN_DATA_FRAME;
    // uint32_t testtest = 0x0UL;
    // TxDataIT[0] = (uint8_t)0x00;
    // TxDataIT[1] = (uint8_t)0x00;
    // TxDataIT[2] = (uint8_t)0x00;
    // TxDataIT[3] = (uint8_t)0x00;
    // // memcpy(&TxDataIT[0], &testtest, 4);
    // uint16_t temp = motor_ElecPosition * 10000.0f;
    // memcpy(&TxDataIT[4], &temp, 2);
    // uint16_t temp2 = motor_PhysPosition;
    // memcpy(&TxDataIT[6], &temp2, 2);
    // HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeaderIT, TxDataIT);
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);
}
