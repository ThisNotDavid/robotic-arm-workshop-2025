#include "stm32f1xx_hal.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;

/* servo timer/channel bindings */
#define SERVO0_HTIM &htim1
#define SERVO1_HTIM &htim1
#define SERVO2_HTIM &htim4
#define SERVO3_HTIM &htim4
#define SERVO4_HTIM &htim4
#define SERVO5_HTIM &htim4

#define SERVO0_TIM_CHANNEL TIM_CHANNEL_4
#define SERVO1_TIM_CHANNEL TIM_CHANNEL_1
#define SERVO2_TIM_CHANNEL TIM_CHANNEL_1
#define SERVO3_TIM_CHANNEL TIM_CHANNEL_2
#define SERVO4_TIM_CHANNEL TIM_CHANNEL_3
#define SERVO5_TIM_CHANNEL TIM_CHANNEL_4

/* geometry (cm) */
#define FOREARM_LENGTH 12.0
#define ARM_LENGTH     12.0

/* servo pulse settings (microseconds).
   Set these per your servo's spec. */
#define SERVO_MIN_US  500u
#define SERVO_MAX_US 2500u

/* Helper: clamp double to [a,b] */
static inline double clamp_d(double v, double a, double b) {
    if (v < a) return a;
    if (v > b) return b;
    return v;
}

static inline uint32_t angle_to_pulse_us(double angle, uint32_t min_us, uint32_t max_us) {
    angle = clamp_d(angle, 0.0, 180.0);
    return (uint32_t)(min_us + (angle / 180.0) * (double)(max_us - min_us));
}

static inline uint32_t pulse_us_to_ticks(uint32_t pulse_us) {
    return (pulse_us * 1000 / 20000); // 1 tick == 20 us
}

void setServoAngle(TIM_HandleTypeDef *htim, uint32_t timChannel, double servoAngle) {
    uint32_t pulse_us = angle_to_pulse_us(servoAngle, SERVO_MIN_US, SERVO_MAX_US);
    uint32_t ticks = pulse_us_to_ticks(pulse_us);
    __HAL_TIM_SET_COMPARE(htim, timChannel, ticks);
}

/* move using three DOF: base(b), shoulder(a1), elbow(a2) in degrees */
void moveToAngle(double servoAngle0, double servoAngle1, double servoAngle2) {
    setServoAngle(SERVO0_HTIM, SERVO0_TIM_CHANNEL, servoAngle0);
    setServoAngle(SERVO1_HTIM, SERVO1_TIM_CHANNEL, servoAngle1);
    setServoAngle(SERVO2_HTIM, SERVO2_TIM_CHANNEL, servoAngle2);
}

void setBaseAngle(double angle) {
	angle = clamp_d(angle, 0, 180);
	setServoAngle(SERVO0_HTIM, SERVO0_TIM_CHANNEL, angle);
}

void setClawAngle(double angle) {
	angle = clamp_d(angle, 0, 90);
	setServoAngle(SERVO5_HTIM, SERVO5_TIM_CHANNEL, angle);
}

void moveToPos(double x, double y, double z) {
	double h = sqrt(x*x + y*y);
	double b = atan2(y, x) * 180 / M_PI;
	double l = sqrt(z*z + h*h);
	double phi = atan2(z, h) * 180 / M_PI;

	if (l > FOREARM_LENGTH + ARM_LENGTH)	return;

	double num1 = FOREARM_LENGTH*FOREARM_LENGTH + l*l - ARM_LENGTH*ARM_LENGTH;
	double den1 = 2 * FOREARM_LENGTH * l;
	double cos_theta1 = clamp_d(num1/den1, -1.0, 1.0);
	double theta1 = acos(cos_theta1) * 180 / M_PI;

	double num2 = FOREARM_LENGTH*FOREARM_LENGTH + ARM_LENGTH*ARM_LENGTH - l*l;
	double den2 = 2 * FOREARM_LENGTH * ARM_LENGTH;
	double cos_theta2 = clamp_d(num2/den2, -1.0, 1.0);
	double theta2 = acos(cos_theta2) * 180 / M_PI;

	double a1 = theta1 + phi;
	double a2 = 360 - 135 - theta2;

	double a3 = 180.0 - theta1 - theta2 - phi;

	b  = clamp_d(b, 0.0, 180.0);
	a1 = clamp_d(a1, 0.0, 180.0);
	a2 = clamp_d(a2, 0.0, 180.0);
	a3 = clamp_d(a3, 0.0, 180.0);

	moveToAngle(b, a1, a2);
	setServoAngle(SERVO4_HTIM, SERVO4_TIM_CHANNEL, a3);
}

/* start PWM and set initial positions */
void Servo_Init(void) {
    HAL_TIM_PWM_Start(SERVO0_HTIM, SERVO0_TIM_CHANNEL);
    HAL_TIM_PWM_Start(SERVO1_HTIM, SERVO1_TIM_CHANNEL);
    HAL_TIM_PWM_Start(SERVO2_HTIM, SERVO2_TIM_CHANNEL);
    HAL_TIM_PWM_Start(SERVO3_HTIM, SERVO3_TIM_CHANNEL);
    HAL_TIM_PWM_Start(SERVO4_HTIM, SERVO4_TIM_CHANNEL);
    HAL_TIM_PWM_Start(SERVO5_HTIM, SERVO5_TIM_CHANNEL);

    /* set neutral / safe poses */
    setServoAngle(SERVO3_HTIM, SERVO3_TIM_CHANNEL, 90.0);
    setServoAngle(SERVO4_HTIM, SERVO4_TIM_CHANNEL, 90.0);
    setServoAngle(SERVO5_HTIM, SERVO5_TIM_CHANNEL,  0.0);

    moveToAngle(90.0, 90.0, 45.0);	// stand straight up

    // Sending Init message to Robotic Arm
    uint8_t txData0[] = "Getting ready...\r\n";
    uint8_t txData1[] = "Done. Joystick control enabled.\r\n";
    HAL_UART_Transmit(&huart1, txData0, sizeof(txData0)-1, HAL_MAX_DELAY);
    HAL_Delay(5000);
    HAL_UART_Transmit(&huart1, txData1, sizeof(txData1)-1, HAL_MAX_DELAY);
}

void checkAngle(void) {
	for (uint8_t baseAngle = 0; baseAngle <= 180; baseAngle += 45) {
		moveToAngle(baseAngle, 90, 45);
		HAL_Delay(2000);
	}
	HAL_Delay(3000);

	for (uint8_t a1 = 90; a1 <= 180; a1 += 10) {
		moveToAngle(90, a1, 45);
		HAL_Delay(500);
	}
	HAL_Delay(2000);

	for (uint8_t a2 = 0; a2 <= 180; a2 += 45) {
		moveToAngle(90, 90, a2);
		HAL_Delay(2000);
	}
	HAL_Delay(2000);
}


// archived

//void moveToPos(double baseAngle, double x, double y) {
//	double a1 = 0, a2 = 0;
//	double h = sqrt(x*x + y*y);
//	double phi = atan2(y, x) * 180.0 / M_PI;
//
//	double num1 = FOREARM_LENGTH*FOREARM_LENGTH + h*h - ARM_LENGTH;
//	double den1 = 2 * FOREARM_LENGTH * h;
//	double theta1 = acos(num1/den1) * 180.0 / M_PI;
//
//	double num2 = FOREARM_LENGTH*FOREARM_LENGTH + ARM_LENGTH*ARM_LENGTH - h*h;
//	double den2 = 2 * FOREARM_LENGTH * ARM_LENGTH;
//	double theta2 = acos(num2/den2) * 180.0 / M_PI;
//
//	a1 = theta1 + phi;
//	a2 = 360.0 - 135.0 - theta2;
//
//	moveToAngle(baseAngle, a1, a2);
//}
