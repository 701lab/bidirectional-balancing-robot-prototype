// $branch$


// For global variables declaration
#define VAR_DECLS

#include "implementation.h"
#include "device.h"


/*
	TODO: Добавить изменение знака угла комплиментарного фильтра при повороте робота (когда происходит переход от -180 к 180 и обратно).
			Сейчас данный переход из-за фильтра происзодит в несколько итераций напрямую от одног значения к другому, а должен перескакивать
	TODO:
	TODO:
	TODO:


 */

// ********************************** //
// ****** Motor 1 declarations ****** //
// ********************************** //
motor motor1 =
			{
					.encoder_constant = 937.2f,
					.max_duty_cycle_coefficient = PWM_PRECISION
			};

speed_control motor1_speed_cotroller =
			{
					.current_integral = 0.0f,
					.controller_output_limitation_value = PWM_PRECISION,
					.previous_encoder_counter_value = 0,
					.previous_speed_mistake = 0.0f,
					.target_speed = 0.0f,
					.regulator_control_signal = 0.0f,
					.current_speed = 0.0f
			};

position_control motor1_position_controller =
			{
					.current_position = 0.0f,
					.previous_encoder_counter_value = 0.0f,
					.regulator_control_signal = 0.0f,
					.target_position = 0.0f,
			};

// ********************************** //
// ****** Motor 2 declarations ****** //
// ********************************** //
motor motor2 =
			{
					.encoder_constant = 937.2f,
					.max_duty_cycle_coefficient = PWM_PRECISION
			};

speed_control motor2_speed_cotroller =
			{
					.current_integral = 0.0f,
					.controller_output_limitation_value = PWM_PRECISION,
					.previous_encoder_counter_value = 0,
					.previous_speed_mistake = 0.0f,
					.target_speed = 0.0f,
					.regulator_control_signal = 0.0f,
					.current_speed = 0.0f
			};

position_control motor2_position_controller =
			{
					.current_position = 0.0f,
					.previous_encoder_counter_value = 0.0f,
					.regulator_control_signal = 0.0f,
					.target_position = 0.0f,
			};


// *********************************** //
// ****** ICM-20600 declaration ****** //
// *********************************** //
// Пока оставлю, не уверен, чт одля данного проекта мне это нужно, но все же
icm_20600 robot_imu = {.device_was_initialized = 0};
int16_t icm_raw_data[7] = { 0, 0, 0, 0, 0, 0, 0 };
float icm_processed_data[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
int16_t gyro_calib_values[3] = {0, 0, 0};

float x_z_plane_angle = 0.0f;

// *********************************** //
// ****** NRF24L01+ declaration ****** //
// *********************************** //
nrf24l01p robot_nrf24 = {.device_was_initialized = 0};

//uint8_t nrf24_rx_address[5] = {0xAA,0xBB,0xCC,0xEE,0x15};	// green LEDs car
uint8_t nrf24_rx_address[5] = {0xAA,0xBB,0xCC,0xEE,0x25};	// blue LEDs car
uint16_t nrf_input_data[5] = {0, 0, 0, 0, 0};

uint32_t nrf24_data_has_been_captured = 0;
uint32_t nrf24_safety_counter = 0;

uint32_t someVar = 0;

// Balancing robot stuff

void handle_angle_reg(icm_20600 *icm_instance, int16_t icm_data[], float integration_period);
void handle_speed_reg(float average_motors_speed);
void icm_add_data_to_filter(icm_20600 * icm_instance, int32_t filter_array[7]);
void clear_int_array(int32_t input_array[], uint32_t array_length);

int32_t icm_filtering_data [7] = {0, 0, 0, 0, 0, 0, 0};
// Angle regulator
float actual_zero_angle = 90.5f; // deg
float angle_loop_task = 90.5f;	// deg
// !! Важно видеть этот момент, так как сейчас я не использую всей скороости
//float angle_loop_max_output = 2.0f;	// RRM
float angle_loop_max_output = 7.0f;	// RRM
float angle_current_value = 0.0f;	// deg
float angle_loop_mistake;
float angle_loop_previous_mistake = 0.0f;
float angle_loop_integral = 0.0f;
float angle_loop_control_signal = 0.0f;
float angle_regulator_kp = 0.5f;//0.3f;
float angle_regulator_ki = 5.0f;//2.8f;
float angle_regulator_kd = 0.00f;//0.0;
float balancing_fault = 0;

float angle_loop_p_part = 0.0f;
float angle_loop_i_part = 0.0f;
float angle_loop_d_part = 0.0f;

float robot_rotation_task = 0.0f;
float rotation_mistake = 0.0f;
float rotation_integral = 0.0f;

// Speed regulator
//float rotation_task = 0.0f;
float speed_reg_mistake;
float speed_reg_task = 0.0f;
//float speed_reg_max_output = 10.0f;
float speed_reg_ki = 1.0f; //0.5f;
float speed_reg_kp = 2.2f;	//1.5f;
float speed_reg_integral = 0;
float speed_reg_control_signal;

float speed_loop_p_part = 0.0f;
float speed_loop_i_part = 0.0f;

int16_t previousEncoderTicks = 0;
float current_m1_speed;
float current_m2_speed;
float angle_first_value = 0.0f;


// Мусор для отладки
int16_t raw_filtered_data_arrary[7];
float accelerometer_based_angle;
float gyroscope_based_angle;

int main(void)
{
	// ****** Motor 1 initialization ****** //
	motor1.motor_disable = gpioc6_low;
	motor1.motor_enable = gpioc6_high;
	motor1.set_pwm_duty_cycle = set_motor1_pwm;
	motor1.get_encoder_counter_value = get_motor1_encoder_value;
	motor1.speed_controller = &motor1_speed_cotroller;
	motor1.position_controller = &motor1_position_controller;
	motor1_speed_cotroller.kp = 200.0f;
	motor1_speed_cotroller.ki = 5000.0f;
	motor1_position_controller.kp = 1.0f;
	motor1_position_controller.position_precision = 8.0f/motor1.encoder_constant;

	// ****** Motor 2 initialization ****** //
	motor2.motor_disable = gpioc6_low;
	motor2.motor_enable = gpioc6_high;
	motor2.set_pwm_duty_cycle = set_motor2_pwm;
	motor2.get_encoder_counter_value = get_motor2_encoder_value;
	motor2.speed_controller = &motor2_speed_cotroller;
	motor2.position_controller = &motor2_position_controller;
	motor2_speed_cotroller.kp = 200.0f;
	motor2_speed_cotroller.ki = 5000.0f;
	motor2_position_controller.kp = 1.0f;
	motor2_position_controller.position_precision = motor1_position_controller.position_precision;

	// ****** ICM-20600 initialization ****** //
	robot_imu.cs_high = gpiob12_high;
	robot_imu.cs_low = gpiob12_low;
	robot_imu.send_one_byte = spi2_write_single_byte;
	robot_imu.gyro_full_scale_setup = icm_gyro_500dps;
	robot_imu.accel_full_scale_setup = icm_accel_2g;
	robot_imu.enable_temperature_sensor = 0;
	robot_imu.gyro_calibration_coefficients[icm_x] = -35;
	robot_imu.gyro_calibration_coefficients[icm_y] = 412;
	robot_imu.gyro_calibration_coefficients[icm_z] = -84;
	robot_imu.complementary_filter_coefficient = 0.1f;

	// ****** NRF24L01+ initialization ****** //
	robot_nrf24.ce_high = gpiob0_high;
	robot_nrf24.ce_low = gpiob0_low;
	robot_nrf24.csn_high = gpiob1_high;
	robot_nrf24.csn_low = gpiob1_low;
	robot_nrf24.spi_write_byte = spi1_write_single_byte;
	robot_nrf24.frequency_channel = 45;
	robot_nrf24.payload_size_in_bytes = 10;
	robot_nrf24.power_output = nrf24_pa_high;
	robot_nrf24.data_rate = nrf24_1_mbps;

	// Initialization of MCU peripherals
	full_device_setup(yes, yes);

	// Board is online LED
	// Для отладки (проверки того, что новый код действительно был скомпилирована и прошит) время от времени буду менять, какой из светодиодов горт.
	GPIOD->ODR |= 0x08;
//	GPIOD->ODR |= 0x04;
//	GPIOD->ODR |= 0x02;
//	GPIOD->ODR |= 0x01;

	// Enables both motors
	motor1.motor_enable();

	delay_in_milliseconds(200);

	// NRF24L01+ device setup
	add_to_mistakes_log(nrf24_basic_init(&robot_nrf24));
	add_to_mistakes_log(nrf24_enable_pipe1(&robot_nrf24, nrf24_rx_address));
	add_to_mistakes_log(nrf24_enable_interrupts(&robot_nrf24, yes, no, no));
	add_to_mistakes_log(nrf24_rx_mode(&robot_nrf24));

	// icm-20600 initialization
	add_to_mistakes_log(icm_20600_basic_init(&robot_imu));

	// If necessary imu can be calibrated
//	imu_gyro_calibration(&robot_imu, gyro_calib_values);

	icm_20600_get_proccesed_data(&robot_imu, icm_processed_data);

	angle_current_value = atan2(-1 * icm_processed_data[icm_accelerometer_x], icm_processed_data[icm_accelerometer_z]) * 57.296f;
	angle_first_value = angle_current_value;

	while(1)
	{
		// Place for some dummy code
	}
}


// ****** Control loop handler ****** //
// Control loop handles by the SysTick timer

// *** Speed controller setup variables ***//
uint32_t speed_loop_call_counter = 0;

#define SPEED_LOOP_FREQUENCY				20	// Times per second. Must be not bigger then SYSTICK_FREQUENCY.
#define SPEED_LOOP_COUNTER_MAX_VALUE 		( SYSTICK_FREQUENCY / SPEED_LOOP_FREQUENCY )	// Times.
#define SPEED_LOOP_PERIOD					( 1.0f / (float)(SPEED_LOOP_FREQUENCY) ) // Seconds.

// ****** Balancing robot control loops ****** //
// Учитыва, что все контура должны отличаться друг от друга по частоте вызовов минимум в два раза, и низкую точность имеющихся энкодеров придется запускать систему управления на довольно низкой частоте
// Возможно, если такая система покажет себя не лучшим образом, можно будет попробовать запустить систему управления без управления по скорости двигателей и соответственно ускорить остальные контуры

uint32_t angle_loop_call_counter = 0;

#define ANGLE_LOOP_FREQUENCY				40	// Times per second. Must be not bigger then SYSTICK_FREQUENCY.
#define ANGLE_LOOP_COUNTER_MAX_VALUE 		( SYSTICK_FREQUENCY / ANGLE_LOOP_FREQUENCY )	// Times.
#define ANGLE_LOOP_PERIOD					( 1.0f / (float)(ANGLE_LOOP_FREQUENCY) )// Seconds.

uint32_t angle_speed_loop_call_counter = 0;

#define ANGLE_SPEED_LOOP_FREQUENCY			20
#define ANGLE_SPEED_LOOP_COUNTER_MAX_VALUE	( SYSTICK_FREQUENCY / ANGLE_SPEED_LOOP_FREQUENCY )
#define ANGLE_SPEED_LOOP_PERIOD				( 1.0f / (float)(ANGLE_SPEED_LOOP_FREQUENCY) )

void SysTick_Handler()
{

	// Каждую итерацию вызывать измерения данных у ICM
	icm_add_data_to_filter(&robot_imu, icm_filtering_data);

	// *** Speed control handling *** //
	speed_loop_call_counter += 1;
	if ( speed_loop_call_counter == SPEED_LOOP_COUNTER_MAX_VALUE )	// 20 times per second
	{
		speed_loop_call_counter = 0;
		current_m1_speed = motors_get_speed_by_incements(&motor1, SPEED_LOOP_PERIOD);
		current_m2_speed = motors_get_speed_by_incements(&motor2, SPEED_LOOP_PERIOD);

		rotation_mistake = robot_rotation_task;// - (current_m2_speed - current_m1_speed) / 0.18f ; // где 0.18 - длина оси в метрах/*тут должна бытьдлина оси*/
		if (rotation_mistake > 2 )
		{
			rotation_mistake = 2;
		}
		if ( rotation_mistake < -2)
		{
			rotation_mistake = -2;
		}

//		rotation_mistake_integral += rotation_mistake
		// Код специально для балансирующего робота. Интегрируем ошибку по скорости
		speed_reg_integral += (speed_reg_task - (current_m1_speed + current_m2_speed) / 2.0f) * SPEED_LOOP_PERIOD;
	}

	angle_loop_call_counter += 1;
	if ( angle_loop_call_counter == ANGLE_LOOP_COUNTER_MAX_VALUE )
	{
		angle_loop_call_counter = 0;
//		int16_t raw_filtered_data[7];

		// data averaging
		for(int i = 0; i < 6; i++)
		{
			raw_filtered_data_arrary[i] = icm_filtering_data[i] / ANGLE_LOOP_COUNTER_MAX_VALUE;
		}

		handle_angle_reg(&robot_imu, raw_filtered_data_arrary, ANGLE_LOOP_PERIOD); // Даст новое задание для двигателей

		clear_int_array(icm_filtering_data, 7);
	}

	angle_speed_loop_call_counter += 1;
	if( angle_speed_loop_call_counter == ANGLE_SPEED_LOOP_COUNTER_MAX_VALUE)
	{
		angle_speed_loop_call_counter = 0;

		handle_speed_reg ( (current_m1_speed + current_m2_speed) / 2.0f );

	}

	// *** Nrf24l01+ safety clock *** //
	if(nrf24_data_has_been_captured == 1)
	{
		nrf24_data_has_been_captured = 0;
		nrf24_safety_counter = 0;
	}
	else
	{
		nrf24_safety_counter += 1;
		if(nrf24_safety_counter == SYSTICK_FREQUENCY) // Exactly one second delay
		{
			// Stop and show that data is not capturing any more
			GPIOD->ODR &= ~0x01;
			motor1.speed_controller->target_speed = 0.0f;
			motor2.speed_controller->target_speed = 0.0f;
		}
	}
}
// **************************************** //


// Место для черновых функций, специально для проекта балансирующего робота.

// Данная функция должна принимать данные от IMU и давать задания движения на двигатели
void handle_angle_reg(icm_20600 *icm_instance, int16_t icm_data[], float integration_period)
{

	// ICM data at input is actually raw, so it should be first converted into related angle based scales

//	float icm_processed_data[7];
	icm_20600_procces_raw_data(icm_instance, icm_data, icm_processed_data);

	// Calculation of robot angle
	// как сейчас проводятся вычисления: комплиментарный угол = (угол по акселерометру)*коэффициент + (приращение угла по гироскопу)*(1- коэффициант)
	// Данный подход является совершенно неправильным, так как уменьшает реальный имеющийся угол в отношении коэффеициента деления. То есть реальный угол на выходе всегда был равен примерно реальный угол * коэффицент фильтра.
	// что на самом деле совершенно неправильно. Вместо этого надо складывать угол по акселерометру и (приращение кгла по гироскопу + прошлый угол)
	accelerometer_based_angle = atan2(-1 * icm_processed_data[icm_accelerometer_x], icm_processed_data[icm_accelerometer_z]) * 57.296f; // Angle in degrees
	gyroscope_based_angle = angle_current_value + (icm_instance->previous_gyro_y + icm_processed_data[icm_gyroscope_y])/2.0f * integration_period;	// Разность между последним извесным значением угла и трапециидальным интегралом скорости вращения
	icm_instance->previous_gyro_y = icm_processed_data[icm_gyroscope_y];
	if(gyroscope_based_angle > 180.0f)
	{
		gyroscope_based_angle = -360 + gyroscope_based_angle;
	}
	if (gyroscope_based_angle < -180.0f)
	{
		gyroscope_based_angle = 360 + gyroscope_based_angle;
	}

	if ( (gyroscope_based_angle > 0.0f && accelerometer_based_angle < 0.0f) || (gyroscope_based_angle < 0.0f && accelerometer_based_angle > 0.0f) )
	{
		gyroscope_based_angle = accelerometer_based_angle;
	}

	// Проверкае на переход между -180 и 180 и соответственно смену знака

	angle_current_value = accelerometer_based_angle * icm_instance->complementary_filter_coefficient + gyroscope_based_angle * (1.0f - icm_instance->complementary_filter_coefficient);
//	angle_current_value = -1 * ( accelerometer_based_angle * icm_instance->complementary_filter_coefficient /*+ проверка знака */ - gyroscope_based_angle * (1.0f - icm_instance->complementary_filter_coefficient));

	angle_loop_mistake = angle_loop_task - angle_current_value;

	// If angle is too big too balance stop motors
	if (angle_loop_mistake > 20.0f || angle_loop_mistake < -20.0f)
	{
		balancing_fault = 1;
//		motor_reset(&motor1);
//		motor_reset(&motor2);
		// пока тестирую без регуляторов скорости
		motor1.set_pwm_duty_cycle(0);
		motor2.set_pwm_duty_cycle(0);
		angle_loop_integral = 0.0f;
		angle_loop_task = actual_zero_angle;
		return;
	}
	else
	{
		balancing_fault = 0;
	}

	if (!((angle_loop_control_signal > angle_loop_max_output && angle_loop_mistake < 0.0f) || (angle_loop_control_signal < -angle_loop_max_output && angle_loop_mistake > 0.0f)))
	{
		// Если значение интеграла не находится в насыщении, интегрируем
		angle_loop_integral += (angle_loop_mistake + angle_loop_previous_mistake) / 2.0f * integration_period;
	}
	angle_loop_previous_mistake = angle_loop_mistake;

	// PID controller without filter on D-part

	angle_loop_d_part = icm_processed_data[icm_gyroscope_y] * angle_regulator_kd;
	angle_loop_p_part = angle_loop_mistake * angle_regulator_kp;
	angle_loop_i_part = angle_loop_integral * angle_regulator_ki;

	angle_loop_control_signal = -1* (angle_loop_p_part + angle_loop_i_part + angle_loop_d_part);

//	motor1.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f );
//	motor2.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f );

	// Попытки заставить робота поворачивать на месте

	if( rotation_mistake > 0)
	{
		if (angle_loop_control_signal > 0)
		{
			motor1.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f);
			motor2.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f + rotation_mistake * 300.0f);

		}
		else
		{
			motor1.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f - rotation_mistake * 300.0f);
			motor2.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f);
		}
	}
	else if (rotation_mistake < 0)
	{
		if(angle_loop_control_signal > 0)
		{
			motor1.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f - rotation_mistake * 300.0f);
			motor2.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f);
		}
		else
		{
			motor1.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f);
			motor2.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f + rotation_mistake * 300.0f);
		}
	}
	else
	{
		motor1.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f);
		motor2.set_pwm_duty_cycle(angle_loop_control_signal * 300.0f);
	}



	return;
}

// Надо ввести какой-нибудь флаг, который бы показывал, что робот находится в плохом состоянии для успешного балансирования.
void handle_speed_reg(float average_motors_speed)
{

	if (balancing_fault)
	{
		speed_reg_integral = 0.0f;
		speed_reg_mistake = 0.0f;
		return;
	}

	speed_reg_mistake = speed_reg_task - average_motors_speed;

	speed_loop_i_part = speed_reg_integral * speed_reg_ki;
	speed_loop_p_part = speed_reg_mistake * speed_reg_kp;

	if (speed_loop_i_part < -3)
	{
		speed_reg_control_signal = speed_loop_p_part - 3;
	}
	else if ( speed_loop_i_part > 3)
	{
		speed_reg_control_signal = speed_loop_p_part + 3;
	}
	else
	{
		speed_reg_control_signal = speed_loop_p_part + speed_loop_i_part;
	}

	angle_loop_task = actual_zero_angle + speed_reg_control_signal;
//	if (speed_reg_control_signal < 10.0f && speed_reg_control_signal > -10.0f)
//	{
//		angle_loop_task = actual_zero_angle + speed_reg_control_signal;
//	}

}


// ****** NRf24l01+ IRQ handler ****** //
void EXTI2_3_IRQHandler()
{
	// Clear interrupt flag
	EXTI->FPR1 |= 0x04;

	// Get new data
	add_to_mistakes_log(nrf24_read_message(&robot_nrf24, nrf_input_data, 10));
	GPIOD->ODR |= 0x01;

	nrf24_data_has_been_captured = 1;
	robot_rotation_task = 0.0f;
	speed_reg_task = 0.0f;

	// Check for buttons press
//	if((nrf_input_data[4] & 0x14) == 0x14){ // means, that top right button is unpressed
//		// Do nothing
//	}
//	else if((nrf_input_data[4] & 0x04) == 0){
//		left_motor_boost = 0.3f;
//		right_motor_boost = 0.3f;
//	}
//	else{
//		left_motor_boost = 0.6f;
//		right_motor_boost = 0.6f;
//	}

	// Evaluate the speed tasks
	if(nrf_input_data[2] < 1000 /*means it up*/ && nrf_input_data[1] < 3000 && nrf_input_data[1] > 1000) // Forward
	{
		speed_reg_task = 1.5f;
		speed_reg_integral = 0.0f;
	}
	else if(nrf_input_data[2] < 1000 /*means it up*/ && nrf_input_data[1] < 1000)	// Forward left
	{
		speed_reg_task = 1.0f;
		robot_rotation_task = 0.5f;
		speed_reg_integral = 0.0f;
	}
	else if(nrf_input_data[2] > 1000 && nrf_input_data[2] < 3000 && nrf_input_data[1] < 1000)	// Turn left
	{
		robot_rotation_task = 1.0f;
		speed_reg_integral = 0.0f;
	}
	else if(nrf_input_data[2] > 3000 && nrf_input_data[1] < 1000)	// Backward left
	{
		speed_reg_task = -1.0f;
		robot_rotation_task = -0.5f;
		speed_reg_integral = 0.0f;
	}
	else if(nrf_input_data[2] > 3000  && nrf_input_data[1] < 3000 && nrf_input_data[1] > 1000)	// Backward
	{
		speed_reg_task = -1.5f;
		speed_reg_integral = 0.0f;
	}
	else if(nrf_input_data[2] > 3000 && nrf_input_data[1] > 3000)	// Backward right
	{
		speed_reg_task = -1.0f;
		robot_rotation_task = 0.5f;
		speed_reg_integral = 0.0f;
	}
	else if(nrf_input_data[2] < 1000 && nrf_input_data[1] > 3000)	// Forward right
	{
		speed_reg_task = 1.0f;
		robot_rotation_task = -0.5f;
		speed_reg_integral = 0.0f;
	}
	else if(nrf_input_data[2] > 1000 && nrf_input_data[2] < 3000 && nrf_input_data[1] > 3000)	// Turn right
	{
		robot_rotation_task = -1.0f;
		speed_reg_integral = 0.0f;
	}
}
// **************************************** //




void icm_add_data_to_filter(icm_20600 * icm_instance, int32_t filter_array[7])
{
	int16_t temp[7];
	icm_20600_get_raw_data(icm_instance, temp);
	filter_array[0] += temp[0];
	filter_array[1] += temp[1];
	filter_array[2] += temp[2];
	filter_array[3] += temp[3];
	filter_array[4] += temp[4];
	filter_array[5] += temp[5];
	filter_array[6] += temp[6];
}

void clear_int_array(int32_t input_array[], uint32_t array_length)
{
	for(uint32_t i = 0; i < array_length; ++i)
	{
		input_array[i] = 0;
	}
}

// EOF

