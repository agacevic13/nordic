#include <stdio.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>


#define GNSS_SAMPLE_PERIODIC_INTERVAL	120
#define GNSS_SAMPLE_PERIODIC_TIMEOUT	70
#define PI 3.14159265358979323846
#define EARTH_RADIUS_METERS (6371.0 * 1000.0)
#define SINGLE_FIX_TIMEOUT_SEC 10
#define SINGLE_FIX_INTERVAL		0

void modem_lib_initialize(void);
int gps_start(void);
int get_position(void);


