#define DT_DRV_COMPAT adi_adxl343

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "adxl343.h"

LOG_MODULE_REGISTER(ADXL343, CONFIG_SENSOR_LOG_LEVEL);

struct adxl343_data {
    uint8_t channel_states;
};

struct adxl343_dev_config {
    struct i2c_dt_spec i2c;
};

static int adxl343_sample_fetch(const struct device *dev, 
                                enum sensor_channel chan) {
	
    return 0;
}

static int adxl343_channel_get(const struct device *dev, 
                               enum sensor_channel chan, 
                               struct sensor_value *val) {

    return 0;
}

static int adxl343_attr_set(const struct device *dev, 
                            enum sensor_channel chan, 
                            enum sensor_attribute attr, 
                            const struct sensor_value *val) {

	const struct adxl343_dev_config *config = dev->config;

	switch(chan) {
		case ADXL343_INITIALIZE : {
			// starting the self-test procedure 
			// steps:
			// 0). make sure that the ADXl343 is flat along a surface. This will be useful when calculating offsets
			// 1). must be in normal power operation, set LOW_POWER bit to 0 in BW_RATE register,
			// 	set ODR to 400Hz, code 1100 in BW_RATE register
			// 2). set measure bit (D3), no link, no AUTO_SLEEP, no sleep in POWER_CTL register
			// 3). set FIFO_MODE bits of FIFO_CTL register (0x38) to 00 to bypass FIFO,
			//	trigger bit to 0
			// 4). set Range bits to 16g, FULL_RES to 0, justify to 0 in DATA_FORMAT register 0x31
			// 5). wait cycles to sample roughly 40 measurements for each axis to average
			// 6). enable self-test by setting SELF_TEST bit in DATA_FORMAT register 0x31
			// 7). wait cycles and sample roughly 40 measurements for each axis to average
			// 8). if self-test passes, perform offset calibration

			// hardcoded values incoming!
			// i'm running the adxl343 at 3.3V
			// in the datasheet, at +/- 16g, 10-bit resolution 2.5V, the accepted LSB values for 
			// Xst = Xst_on - Xst_off
			// Yst = Yst_on - Yst_off
			// Zst = Zst_on - Zst_off
			// are :
			// 6 < Xst < 67
			// -67 < Yst < -6
			// 10 < Zst < 110
			// however, the output scale factor at 2.5V is 1, and at 3.3V, it's 1.77 for X, Y and 1.47 for Z 
			// my guess is that scale factor means to DIVIDE the output by the scale factor
			// this is because the lower the Vs, the higher the limits and lower the scale factors
			// can also verify that if I set Vs to 2.5V, self-test passes which means that it must be divide
			// this is because at 3.3V, Zst was roughly 80 and 80 * 1.47 = 117.6 > 110
			// but at 2.5V, Zst is 43, well within the bounds of 10 and 110.

			uint8_t bw_rate_val = 0b00001100;
			int i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_BW_RATE_REG, bw_rate_val);
			if(i2c_operation_error) {
				LOG_ERR("trouble with i2c while writing to ADXL BW RATE \r\n");
				return -EIO;
			}

			uint8_t power_ctl_val = 0b00001000;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_POWER_CTL_REG, power_ctl_val);
			if(i2c_operation_error) {
				LOG_ERR("trouble with i2c while writing to ADXL Power CTL \r\n");
				return -EIO;
			}

			uint8_t fifo_ctl_val = 0b00000000;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_FIFO_CTL_REG, fifo_ctl_val);
			if(i2c_operation_error) {
				LOG_ERR("trouble with i2c while writing to ADXL FIFO CTL \r\n");
				return -EIO;
			}

			uint8_t data_format_val = 0b00000011;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_DATA_FORMAT_REG, data_format_val);
			if(i2c_operation_error) {
				LOG_ERR("trouble with i2c while writing to ADXL FIFO CTL \r\n");
				return -EIO;
			}

			uint8_t number_of_samples_to_acquire = 40;
			int16_t x_axis_accel_avg = 0;
			int16_t y_axis_accel_avg = 0;
			int16_t z_axis_accel_avg = 0;

			uint8_t axis_data[6];
			
			uint8_t number_of_delay_cycles = 0;
			while(number_of_delay_cycles < number_of_samples_to_acquire) {
				i2c_operation_error = i2c_burst_read_dt(&config->i2c, ADXL343_DATAX0_REG, axis_data, 6);
				if(i2c_operation_error) {
					LOG_ERR("trouble with i2c while reading from to ADXL DATAX0 while spinning \r\n");
					return -EIO;
				}
				number_of_delay_cycles++;
			}

			uint8_t number_of_sample_cycles = 0;
			while(number_of_sample_cycles < number_of_samples_to_acquire) {
				i2c_operation_error = i2c_burst_read_dt(&config->i2c, ADXL343_DATAX0_REG, axis_data, 6);
				if(i2c_operation_error) {
					LOG_ERR("trouble with i2c while reading from to ADXL DATAX0 while sampling pre self-test \r\n");
					return -EIO;
				}
				number_of_sample_cycles++;
				
				int16_t x_axis_data = axis_data[1];
				x_axis_data = x_axis_data << 8;
				x_axis_data += axis_data[0];
				x_axis_accel_avg += x_axis_data;

				int16_t y_axis_data = axis_data[3];
				y_axis_data = y_axis_data << 8;
				y_axis_data += axis_data[2];
				y_axis_accel_avg += y_axis_data;

				int16_t z_axis_data = axis_data[5];
				z_axis_data = z_axis_data << 8;
				z_axis_data += axis_data[4];
				z_axis_accel_avg += z_axis_data;
			}

			x_axis_accel_avg /= 40;
			y_axis_accel_avg /= 40;
			z_axis_accel_avg /= 40;

			// Starting self-test now
			data_format_val = 0b10000011;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_DATA_FORMAT_REG, data_format_val);
			if(i2c_operation_error) {
				LOG_ERR("trouble with starting self-test \r\n");
				return -EIO;
			}

			int16_t self_test_x_axis_accel_avg = 0;
			int16_t self_test_y_axis_accel_avg = 0;
			int16_t self_test_z_axis_accel_avg = 0;

			number_of_delay_cycles = 0;
			uint8_t number_of_self_test_delay_cycles = 4;
			while(number_of_delay_cycles < number_of_self_test_delay_cycles) {
				i2c_operation_error = i2c_burst_read_dt(&config->i2c, ADXL343_DATAX0_REG, axis_data, 6);
				if(i2c_operation_error) {
					LOG_ERR("trouble with i2c while reading from to ADXL DATAX0 while spinning pre selftest \r\n");
					return -EIO;
				}
				number_of_delay_cycles++;
			}

			number_of_sample_cycles = 0;
			while(number_of_sample_cycles < number_of_samples_to_acquire) {
				i2c_operation_error = i2c_burst_read_dt(&config->i2c, ADXL343_DATAX0_REG, axis_data, 6);
				if(i2c_operation_error) {
					LOG_ERR("trouble with i2c while reading from to ADXL DATAX0 while sampling during self-test \r\n");
					return -EIO;
				}
				number_of_sample_cycles++;
				
				int16_t x_axis_data = axis_data[1];
				x_axis_data = x_axis_data << 8;
				x_axis_data += axis_data[0];
				self_test_x_axis_accel_avg += x_axis_data;

				int16_t y_axis_data = axis_data[3];
				y_axis_data = y_axis_data << 8;
				y_axis_data += axis_data[2];
				self_test_y_axis_accel_avg += y_axis_data;

				int16_t z_axis_data = axis_data[5];
				z_axis_data = z_axis_data << 8;
				z_axis_data += axis_data[4];
				self_test_z_axis_accel_avg += z_axis_data;
			}

			self_test_x_axis_accel_avg /= 40;
			self_test_y_axis_accel_avg /= 40;
			self_test_z_axis_accel_avg /= 40;

			printk("self test on - x: %d, y: %d, z: %d \r\n", self_test_x_axis_accel_avg,
															  self_test_y_axis_accel_avg,
															  self_test_z_axis_accel_avg);

			printk("self test off - x: %d, y: %d, z: %d \r\n", x_axis_accel_avg,
															   y_axis_accel_avg,
															   z_axis_accel_avg);

			printk("self test values - x: %d, y: %d, z: %d \r\n", (self_test_x_axis_accel_avg - x_axis_accel_avg),
			                                                      (self_test_y_axis_accel_avg - y_axis_accel_avg),
																  (self_test_z_axis_accel_avg - z_axis_accel_avg));
			
			float x_st = self_test_x_axis_accel_avg - x_axis_accel_avg;
			float y_st = self_test_y_axis_accel_avg - y_axis_accel_avg;
			float z_st = self_test_z_axis_accel_avg - z_axis_accel_avg;

			float x_st_scaled = x_st / 1.77;
			float y_st_scaled = y_st / 1.77;
			float z_st_scaled = z_st / 1.47;
			
			bool x_st_satisfied = ((x_st_scaled > 6.0) && (x_st_scaled < 67.0));
			bool y_st_satisfied = ((y_st_scaled > -67.0) && (y_st_scaled < 6.0));
			bool z_st_satisfied = ((z_st_scaled > 10.0) && (z_st_scaled < 110.0));

			if(x_st_satisfied && y_st_satisfied && z_st_satisfied) {
				printk("self-test passed! \r\n");
			} else {
				printk("self-test failed \r\n");
				return -EIO;
			}

			// Stopping self-test now
			data_format_val = 0b00000011;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_DATA_FORMAT_REG, data_format_val);
			if(i2c_operation_error) {
				LOG_ERR("trouble with stopping self-test \r\n");
				return -EIO;
			}

			// time to perform offset calibration
			// Note that 16G @ 10-bit resolution is actually the LEAST sensitive to small changes
			// since the ADXL was flat upon a surface, we can reuse the average x,y,z values
			// Xactual = Xmeas - X0g
			// Yactual = Ymeas - Y0g
			// Z0g = Z1g - Sz
			// Zactual = Zmeas - Z0g
			// Since we're sticking to 16g @ 10-bit resolution, the sensitivity is 32 lsb/g
			// this means that Sz = 32
			// each axis offset register is 15.6mg/lsb
			// offset(in lsb) * scale factor of 31.2mg/lsb = value in mg
			// divide this value by 15.6mg to get LSB for offset register
			// this means offset * 31.2/15.6 = lsb for offset register

			float x0g = -1 * (x_axis_accel_avg);
			float y0g = -1 * (y_axis_accel_avg);
			float z0g = -1 * (z_axis_accel_avg - 32.0);

			int8_t x0g_scaled = ceilf(x0g * 2.0);
			int8_t y0g_scaled = ceilf(y0g * 2.0);
			int8_t z0g_scaled = ceilf(z0g * 2.0);

			printk("setting offsets x: %d, y: %d, z: %d \r\n", x0g_scaled,
															   y0g_scaled,
															   z0g_scaled);
			
			uint8_t offset_data[3] = {x0g_scaled, y0g_scaled, z0g_scaled};
			i2c_operation_error = i2c_burst_write_dt(&config->i2c, ADXL343_OFSX_REG, offset_data, 3);
			if(i2c_operation_error) {
				printk("having trouble setting axis offsets \r\n");
				return -EIO;
			} else {
				printk("successfully set axis offsets \r\n");
			}

			#ifdef DEBUG
			int16_t normalized_x_axis_data_avg = 0;
			int16_t normalized_y_axis_data_avg = 0;
			int16_t normalized_z_axis_data_avg = 0;

			number_of_sample_cycles = 0;
			while(number_of_sample_cycles < number_of_samples_to_acquire) {
				i2c_operation_error = i2c_burst_read_dt(&config->i2c, ADXL343_DATAX0, axis_data, 6);
				if(i2c_operation_error) {
					LOG_ERR("trouble with i2c while reading from to ADXL DATAX0 while sampling during self-test \r\n");
					return -EIO;
				}
				number_of_sample_cycles++;
				
				int16_t x_axis_data = axis_data[1];
				x_axis_data = x_axis_data << 8;
				x_axis_data += axis_data[0];
				normalized_x_axis_data_avg += x_axis_data;

				int16_t y_axis_data = axis_data[3];
				y_axis_data = y_axis_data << 8;
				y_axis_data += axis_data[2];
				normalized_y_axis_data_avg += y_axis_data;

				int16_t z_axis_data = axis_data[5];
				z_axis_data = z_axis_data << 8;
				z_axis_data += axis_data[4];
				normalized_z_axis_data_avg += z_axis_data;
				
				if(x_axis_data != 0 || y_axis_data != 0) {
					printk("x: %d, y: %d, z: %d \r\n", x_axis_data, y_axis_data, z_axis_data);
				}
			}

			printk("normalized x: %d, y: %d, z: %d \r\n", (normalized_x_axis_data_avg/40),
														  (normalized_y_axis_data_avg/40),
														  (normalized_z_axis_data_avg/40));
			#endif

			// set-up double tap detection 
			// TO TRY:
			// 1). enter low power mode here by setting LOW_POWER bit in BW_RATE register (0x2C),
			//		and get an interrupt
			// 2). enter standby mode and try to get an interrupt by clearing bit D3 of POWER_CTL register 0x2D
			//
			// Note: typical values are on page 27
			// Note: I'm referencing some found here: https://forums.adafruit.com/viewtopic.php?f=22&t=155318
			// 1). disable all interrupt in INT_ENABLE register 0x2E
			// 2). clear bit D5 of INT_MAP register(0x2F) to map double-tap to INT1 pin
			// 3). Set THRESH_TAP register (0x1D) with magnitude of tap event
			// 4). Set DUR register (0x21) with duration of a tap event
			// 5). Set LATENT register (0x22) with duration of delay to second tap
			// 6). Set WINDOW register (0x23) with time to detect second tap
			// 7). Set TAP_AXES register (0x2A) with D3 cleared to not avoid suppressing double tap,
			//	and set bits D2-D0 to enable x,y,z axis participation
			// 8). Enable double tap interrupt by setting bit D5 of INT_ENABLE register (0x2E)
			
			uint8_t int_source_byte = 0;
			i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, ADXL343_INT_SOURCE_REG, &int_source_byte);
			if(i2c_operation_error) {
				printk("having trouble clearing double tap interrupt at init \r\n");
				return -EIO;
			} else {
				printk("cleared double tap interrupt at init\r\n");
			}

			uint8_t disable_int_byte = 0b00000000;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_INT_ENABLE_REG, disable_int_byte);
			if(i2c_operation_error) {
				printk("having trouble disabling adxl343 interrupts \r\n");
				return -EIO;
			} else {
				printk("disabled adxl343 interrupts \r\n");
			}

			uint8_t map_double_tap_to_int1_byte = 0b00000000;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_INT_MAP_REG, map_double_tap_to_int1_byte);
			if(i2c_operation_error) {
				printk("having trouble mapping adxl343 interrupts to int1 \r\n");
				return -EIO;
			} else {
				printk("mapped adxl343 interrupts to int1\r\n");
			}

			uint8_t thresh_tap_byte = 20;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_THRESH_TAP_REG, thresh_tap_byte);
			if(i2c_operation_error) {
				printk("having trouble setting adxl343 thresh tap \r\n");
				return -EIO;
			} else {
				printk("set adxl343 thresh tap\r\n");
			}

			uint8_t dur_byte = 300;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_DUR_REG, dur_byte);
			if(i2c_operation_error) {
				printk("having trouble setting adxl343 duration \r\n");
				return -EIO;
			} else {
				printk("set adxl343 duration\r\n");
			}

			uint8_t latent_byte = 100;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_LATENT_REG, latent_byte);
			if(i2c_operation_error) {
				printk("having trouble setting adxl343 latent \r\n");
				return -EIO;
			} else {
				printk("set adxl343 latent\r\n");
			}
			
			uint8_t window_byte = 400;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_WINDOW_REG, window_byte);
			if(i2c_operation_error) {
				printk("having trouble setting adxl343 window \r\n");
				return -EIO;
			} else {
				printk("set adxl343 window\r\n");
			}

			uint8_t tap_axes_byte = 0b00000111;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_TAP_AXES_REG, tap_axes_byte);
			if(i2c_operation_error) {
				printk("having trouble setting adxl343 tap axes \r\n");
				return -EIO;
			} else {
				printk("set adxl343 tap axes\r\n");
			}

			uint8_t enable_int_byte = 0b00100000;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, ADXL343_INT_ENABLE_REG, enable_int_byte);
			if(i2c_operation_error) {
				printk("having trouble enabling adxl343 interrupts \r\n");
				return -EIO;
			} else {
				printk("enabled adxl343 interrupts \r\n");
			}

			break;
		}
		case INTERRUPT : {
			// Clear double-tap interrupt by reading INT_SOURCE register (0x30)
			uint8_t int_source_byte = 0;
			int i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, ADXL343_INT_SOURCE_REG, &int_source_byte);
			if(i2c_operation_error) {
				printk("having trouble clearing double tap interrupt!\r\n");
				return -EIO;
			} else {
				printk("cleared double tap interrupt \r\n");
			}
			
			break;
		}
	}
	
    return 0;
}

int adxl343_set_initial_registers(const struct device *dev) {

	// read the DEVID register (0x00) to make sure that the DEVID is 0xE5
	const struct adxl343_dev_config *config = dev->config;

	uint8_t id = 0;
	int i2c_operation_error = 0;
	i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, ADXL343_DEVID_REG, &id);
	if(i2c_operation_error) {
		LOG_ERR("can't read ADXL device id! \r\n");
		return -EIO;
	}
	if(id != 0xE5) {
		return -ENODEV;
	}

	return 0;
}

int adxl343_init(const struct device *dev) {

    const struct adxl343_dev_config *config = dev->config;
    
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("failure with i2c bus \r\n");
		return -ENODEV;
	}

    /// initialize sensor here

    if(adxl343_set_initial_registers(dev) < 0) {
        LOG_ERR("failure initializing adxl343 \r\n");
        return -EINVAL;
    }

	return 0;
}

static const struct sensor_driver_api adxl343_api_funcs = {
	.sample_fetch = adxl343_sample_fetch,
	.channel_get = adxl343_channel_get,
	.attr_set = adxl343_attr_set,
};

#define ADXL343_DEVICE_INIT(inst)					\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			      adxl343_init,				        \
			      NULL,					            \
			      &adxl343_data_##inst,			    \
			      &adxl343_config_##inst,			\
			      POST_KERNEL,				        \
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &adxl343_api_funcs);

#define ADXL343_CONFIG(inst)								\
    {                                                       \
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
    }                                                       \

#define ADXL343_DEFINE(inst)                                        \
    static struct adxl343_data adxl343_data_##inst;                 \
    static const struct adxl343_dev_config adxl343_config_##inst =	\
        ADXL343_CONFIG(inst);				                        \
    ADXL343_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(ADXL343_DEFINE)
