#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>

LOG_MODULE_REGISTER(gnss_sample, CONFIG_GNSS_SAMPLE_LOG_LEVEL);

#define GNSS_SAMPLE_PERIODIC_INTERVAL	120
#define GNSS_SAMPLE_PERIODIC_TIMEOUT	120
#define PI 3.14159265358979323846
#define EARTH_RADIUS_METERS (6371.0 * 1000.0)

static const char update_indicator[] = {'\\', '|', '/', '-'};

static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static uint64_t fix_timestamp;

/* Reference position. */
static bool ref_used;
static double ref_latitude;
static double ref_longitude;

static K_SEM_DEFINE(pvt_data_sem, 0, 1);

static struct k_poll_event events[1] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&pvt_data_sem, 0),
};

/* Returns the distance between two coordinates in meters. The distance is calculated using the
 * haversine formula.*/
static double distance_calculate(double lat1, double lon1, double lat2, double lon2)
{
	double d_lat_rad = (lat2 - lat1) * PI / 180.0;
	double d_lon_rad = (lon2 - lon1) * PI / 180.0;

	double lat1_rad = lat1 * PI / 180.0;
	double lat2_rad = lat2 * PI / 180.0;

	double a = pow(sin(d_lat_rad / 2), 2) +
		   pow(sin(d_lon_rad / 2), 2) *
		   cos(lat1_rad) * cos(lat2_rad);

	double c = 2 * asin(sqrt(a));

	return EARTH_RADIUS_METERS * c;
}

static void print_distance_from_reference(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	if (!ref_used) 
	{
		return;
	}
	double distance = distance_calculate(pvt_data->latitude, pvt_data->longitude, ref_latitude, ref_longitude);
	printf("\nDistance from reference: %.01f\n", distance);
}

static void gnss_event_handler(int event)
{
	int retval;

	switch (event) {
	case NRF_MODEM_GNSS_EVT_PVT:
		retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
		if (retval == 0) {
			k_sem_give(&pvt_data_sem);
		}
		break;
	default:
		break;
	}
}
static int gnss_init_and_start(void)
{
#if defined(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE) || defined(CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND)
	/* Enable GNSS. */
	if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS) != 0) {
		LOG_ERR("Failed to activate GNSS functional mode");
		return -1;
	}
#endif /* CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE || CONFIG_GNSS_SAMPLE_LTE_ON_DEMAND */

	/* Configure GNSS. */
	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
		LOG_ERR("Failed to set GNSS event handler");
		return -1;
	}
	/* This use case flag should always be set. */
	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;

	if (IS_ENABLED(CONFIG_GNSS_SAMPLE_LOW_ACCURACY)) {
		use_case |= NRF_MODEM_GNSS_USE_CASE_LOW_ACCURACY;
	}

	if (nrf_modem_gnss_use_case_set(use_case) != 0) {
		LOG_WRN("Failed to set GNSS use case");
	}
	/* FOR PERIODIC TRACKING */
	uint16_t fix_retry = GNSS_SAMPLE_PERIODIC_TIMEOUT;
	uint16_t fix_interval = GNSS_SAMPLE_PERIODIC_INTERVAL;

	if (nrf_modem_gnss_fix_retry_set(fix_retry) != 0) {
		LOG_ERR("Failed to set GNSS fix retry");
		return -1;
	}

	if (nrf_modem_gnss_fix_interval_set(fix_interval) != 0) {
		LOG_ERR("Failed to set GNSS fix interval");
		return -1;
	}

	if (nrf_modem_gnss_start() != 0) {
		LOG_ERR("Failed to start GNSS");
		return -1;
	}

	return 0;
}

static void print_satellite_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	uint8_t tracked   = 0;
	uint8_t in_fix    = 0;
	uint8_t unhealthy = 0;

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i) {
		if (pvt_data->sv[i].sv > 0) {
			tracked++;

			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
				in_fix++;
			}

			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY) {
				unhealthy++;
			}
		}
	}

	printf("Tracking: %2d Using: %2d Unhealthy: %d\n", tracked, in_fix, unhealthy);
}

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	printf("Latitude:       %.06f\n", pvt_data->latitude);
	printf("Longitude:      %.06f\n", pvt_data->longitude);
	printf("Altitude:       %.01f m\n", (double)pvt_data->altitude);
	printf("Accuracy:       %.01f m\n", (double)pvt_data->accuracy);
	printf("Speed:          %.01f m/s\n", (double)pvt_data->speed);
	printf("Speed accuracy: %.01f m/s\n", (double)pvt_data->speed_accuracy);
	printf("Heading:        %.01f deg\n", (double)pvt_data->heading);
	printf("Date:           %04u-%02u-%02u\n",
	       pvt_data->datetime.year,
	       pvt_data->datetime.month,
	       pvt_data->datetime.day);
	printf("Time (UTC):     %02u:%02u:%02u.%03u\n",
	       pvt_data->datetime.hour,
	       pvt_data->datetime.minute,
	       pvt_data->datetime.seconds,
	       pvt_data->datetime.ms);
	printf("PDOP:           %.01f\n", (double)pvt_data->pdop);
	printf("HDOP:           %.01f\n", (double)pvt_data->hdop);
	printf("VDOP:           %.01f\n", (double)pvt_data->vdop);
	printf("TDOP:           %.01f\n", (double)pvt_data->tdop);
}

int main(void)
{
	int err;
	uint8_t cnt = 0;

	LOG_INF("Starting GNSS sample");

	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("Modem library initialization failed, error: %d", err);
		return err;
	}
	if (gnss_init_and_start() != 0) {
		LOG_ERR("Failed to initialize and start GNSS");
		return -1;
	}

	fix_timestamp = k_uptime_get();

	for (;;) {
		(void)k_poll(events, 2, K_FOREVER);

		if (events[0].state == K_POLL_STATE_SEM_AVAILABLE &&
		    k_sem_take(events[0].sem, K_NO_WAIT) == 0) {

			/* PVT and NMEA output mode. */
			printf("\033[1;1H");
			printf("\033[2J");
			print_satellite_stats(&last_pvt);

			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED) {
				printf("GNSS operation blocked by LTE\n");
			}
			if (last_pvt.flags &
				NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) {
				printf("Insufficient GNSS time windows\n");
			}
			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_SLEEP_BETWEEN_PVT) {
				printf("Sleep period(s) between PVT notifications\n");
			}
			printf("-----------------------------------\n");

			if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
				fix_timestamp = k_uptime_get();
				print_fix_data(&last_pvt);
				print_distance_from_reference(&last_pvt);
			} 
			else {
				printf("Seconds since last fix: %d\n",
						(uint32_t)((k_uptime_get() - fix_timestamp) / 1000));
				cnt++;
				printf("Searching [%c]\n", update_indicator[cnt%4]);
			}
		}
	}
	return 0;
}
