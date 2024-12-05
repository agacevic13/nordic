/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

static struct sensor_value prev_acc[3] = {0};
static struct sensor_value prev_gyr[3] = {0};
/*MIN_CHANGE can change in  */
#define MIN_CHANGE 4

extern int bmi160_acc_slope_config(const struct device *dev, enum sensor_attribute attr, const struct sensor_value *val);

static void print_bmi160_data(const struct device *bmi160)
{
	struct sensor_value acc[3];
	struct sensor_value gyr[3];

	if (sensor_channel_get(bmi160, SENSOR_CHAN_GYRO_XYZ, gyr) < 0) {
		printf("Cannot read bmg160 gyro channels.\n");
		return;
	}
	if (sensor_channel_get(bmi160, SENSOR_CHAN_GYRO_XYZ, acc) < 0) {
		printf("Cannot read bmg160 acc channels.\n");
		return;
	}
		sensor_channel_get(bmi160, SENSOR_CHAN_ACCEL_XYZ, acc);
		sensor_channel_get(bmi160, SENSOR_CHAN_GYRO_XYZ, gyr);

		printf("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
		       acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2,
		       acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2,
		       gyr[1].val1, gyr[1].val2,
		       gyr[2].val1, gyr[2].val2);
}
// static void print_significant_change(const struct device *bmi160)
// {
//     struct sensor_value acc[3], gyr[3];
//     bool significant_change = false;

//     /* Fetch data from sensor */
//     if (sensor_sample_fetch(bmi160) < 0) {
//         printf("Sensor sample fetch failed.\n");
//         return;
//     }

//     /* Get acceleration and gyroscope data */
//     sensor_channel_get(bmi160, SENSOR_CHAN_ACCEL_XYZ, acc);
//     sensor_channel_get(bmi160, SENSOR_CHAN_GYRO_XYZ, gyr);

//     /* Check for significant change in acceleration */
//     for (int i = 0; i < 3; i++) {
//         if (abs(acc[i].val1 - prev_acc[i].val1) >= MIN_CHANGE) {
//             significant_change = true;
//         }
//     }

//     /* Check for significant change in gyroscope */
//     for (int i = 0; i < 3; i++) {
//         if (abs(gyr[i].val1 - prev_gyr[i].val1) >= MIN_CHANGE) {
//             significant_change = true;
//         }
//     }

//     /* If no significant change, exit */
//     if (!significant_change) {
//         return;
//     }

   
    
//     printf("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
//            "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
//            acc[0].val1, acc[0].val2,
//            acc[1].val1, acc[1].val2,
//            acc[2].val1, acc[2].val2,
//            gyr[0].val1, gyr[0].val2,
//            gyr[1].val1, gyr[1].val2,
//            gyr[2].val1, gyr[2].val2);

//     /* Update previous values */
//     for (int i = 0; i < 3; i++) {
//         prev_acc[i] = acc[i];
//         prev_gyr[i] = gyr[i];
//     }
// }

static void trigger_handler(const struct device *bmi160,
			    const struct sensor_trigger *trigger)
{
	if (trigger->type != SENSOR_TRIG_DATA_READY &&
	    trigger->type != SENSOR_TRIG_DELTA) {
		printf("%p: trigger handler: unknown trigger type.\n", bmi160);
		return;
	}

	if (sensor_sample_fetch(bmi160) < 0) {
		printf("Gyro sample update error.\n");
	}
   //print_significant_change(bmi160);
	print_bmi160_data(bmi160);
}


int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bmi160);
	struct sensor_value acc[3], gyr[3];
	struct sensor_value full_scale, sampling_freq, oversampling, threshold;

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return 0;
	}

	printf("Device %p name is %s\n", dev, dev->name);

	/* Setting scale in G, due to loss of precision if the SI unit m/s^2
	 * is used
	 */
	full_scale.val1 = 2;            /* G */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100;       /* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 0;          /* Normal mode */
	oversampling.val2 = 0;

    threshold.val1 = 2;
	threshold.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling);


	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &threshold);

	if(sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &threshold) != 0){
		printk("Failed to configure slope");
		return 0;
	}
	else{
		printk("suceed to configure slope");
        
	}
	
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change to 0.0Hz before changing
	 * other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sampling_freq);

	/* Setting scale in degrees/s to match the sensor scale */
	full_scale.val1 = 500;          /* dps */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100;       /* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 0;          /* Normal mode */
	oversampling.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling);
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change sampling frequency to
	 * 0.0Hz before changing other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sampling_freq);
     struct sensor_trigger trig;
	 

	trig.type = SENSOR_TRIG_DELTA;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    sensor_trigger_set(dev, &trig, trigger_handler);
	
		
	 
	return 0;
}
