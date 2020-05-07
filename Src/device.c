#include "device.h"


void device_self_diagnosticks(icm_20600 * icm_instance, nrf24l01p *nrf24_instance, motor * first_motor_instance, motor * second_motor_instance)
{

	GPIOD->ODR |= 0x08;

	// Icm-20600 diagnostics
	add_to_mistakes_log(icm_20600_check_if_alive(icm_instance));

	// NRF24 diagnostics
	add_to_mistakes_log(nrf24_check_if_alive(nrf24_instance));

	// Motor1 rotation check
	uint32_t motors_rotation_test_resalt = motors_rotation_deiraction_test(first_motor_instance);
	switch(motors_rotation_test_resalt){
	case 2: add_to_mistakes_log(M1_IS_NOT_CONNECTED); break;
	case 1: add_to_mistakes_log(M1_DIRECTION_MISMATCH); break;
	case 0: break;
	default: break;
	}


	delay_in_milliseconds(500);

	// Motor 2 rotation check
	motors_rotation_test_resalt = motors_rotation_deiraction_test(second_motor_instance);
	switch(motors_rotation_test_resalt){
	case 2: add_to_mistakes_log(M2_IS_NOT_CONNECTED); break;
	case 1: add_to_mistakes_log(M2_DIRECTION_MISMATCH); break;
	case 0: break;
	default: break;
	}
}


/*
	@brief Gets 100 gyroscope measurements and counts average
 */

void imu_gyro_calibration(icm_20600 *icm_instance, int16_t calibration_coeficients[3])
{
	// Delete previous calibration results to get proper values
	icm_instance->gyro_calibration_coefficients[icm_x] = 0;
	icm_instance->gyro_calibration_coefficients[icm_y] = 0;
	icm_instance->gyro_calibration_coefficients[icm_z] = 0;

	int16_t raw_imu_data[7] = {0, 0, 0, 0, 0, 0, 0};

	int32_t gyroscope_raw_sums[3] = {0, 0, 0};

	for (uint32_t i = 0; i < 256; i++)
	{
		delay_in_milliseconds(25); // 40 times per second
		icm_20600_get_raw_data(icm_instance, raw_imu_data);
		gyroscope_raw_sums[icm_x] += raw_imu_data[icm_gyroscope_x];
		gyroscope_raw_sums[icm_y] += raw_imu_data[icm_gyroscope_y];
		gyroscope_raw_sums[icm_z] += raw_imu_data[icm_gyroscope_z];
	}

	calibration_coeficients[icm_x] = (int16_t)(gyroscope_raw_sums[icm_x] / 256);
	calibration_coeficients[icm_y] = (int16_t)(gyroscope_raw_sums[icm_y] / 256);
	calibration_coeficients[icm_z] = (int16_t)(gyroscope_raw_sums[icm_z] / 256);
}


// !!!!!!!!!!!!!!!!!!!!!!!!!!  //
// Danger zone
// !!!!!!!!!!!!!!!!!!!!!!!!!!! //

// Система входит в режим балансирвоания из горизонтального режима только если отклонение от номинального для баланисирования угла меньше 15 по модулю.
// Система входит в горизонтальное режим из режима балансирования Если отклонеение угла от номинального для балансирования больше 20 по модулю.
void choose_state(float current_angle_value, uint32_t *current_system_state)
{
	// 90 градусов - угол балансирования в прямом направдения

	switch (*current_system_state){
		case blancing_state:	// условно от 70 до 110 градусов
		{
			if ( current_angle_value < 70.0f && current_angle_value > -75.0f )
			{
				*current_system_state = upper_horizon_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= upper_horizon_state << 2;
			}
			else if ( current_angle_value > 110.0f || current_angle_value < -105.0f )
			{
				*current_system_state = lower_horizon_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= lower_horizon_state << 2;
			}
			else if ( current_angle_value <= -75.0f && current_angle_value >= -105.0f )
			{
				*current_system_state = up_side_down_balancing_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= up_side_down_balancing_state << 2;
			}

			break;
		}
		case upper_horizon_state: // условно от -75 до 75 грудусов чтобы при переходе было поле задержки
		{
			if (current_angle_value >= 75.0f && current_angle_value <= 105.0f)
			{
				*current_system_state = blancing_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= blancing_state << 2;
			}
			else if ( current_angle_value > 105.0f || current_angle_value < -105.0f )
			{
				*current_system_state = lower_horizon_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= lower_horizon_state << 2;
			}
			else if ( current_angle_value <= -75.0f && current_angle_value >= -105 )
			{
				*current_system_state = up_side_down_balancing_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= up_side_down_balancing_state << 2;
			}

			break;
		}
		case lower_horizon_state: // Условно от -180 до -105 и от 105 до 180
		{
			if (current_angle_value >= 75.0f && current_angle_value <= 105.0f)
			{
				*current_system_state = blancing_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= blancing_state << 2;
			}
			else if ( current_angle_value < 75.0f && current_angle_value > -75.0f )
			{
				*current_system_state = upper_horizon_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= upper_horizon_state << 2;
			}
			else if ( current_angle_value <= -75.0f && current_angle_value >= -105.0f )
			{
				*current_system_state = up_side_down_balancing_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= up_side_down_balancing_state << 2;
			}

			break;
		}
		case up_side_down_balancing_state:
		{
			if (current_angle_value >= 75.0f && current_angle_value <= 105.0f)
			{
				*current_system_state = blancing_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= blancing_state << 2;
			}
			else if ( current_angle_value < 75.0f && current_angle_value > -70.0f )
			{
				*current_system_state = upper_horizon_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= upper_horizon_state << 2;
			}
			else if ( current_angle_value > 105.0f || current_angle_value < -110.0f )
			{
				*current_system_state = lower_horizon_state;
				GPIOD->ODR &= ~0x0C;
				GPIOD->ODR |= lower_horizon_state << 2;
			}

			break;
		}
		default: break;
	}
}

