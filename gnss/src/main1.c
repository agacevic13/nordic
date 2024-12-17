#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <modem/lte_lc.h>
#include <stdint.h>
#include <zephyr/zbus/zbus.h>

//LOG_MODULE_DECLARE(zbus, CONFIG_ZBUS_LOG_LEVEL);

LOG_MODULE_REGISTER(gnss_sample, CONFIG_GNSS_SAMPLE_LOG_LEVEL);

#define GNSS_SAMPLE_PERIODIC_INTERVAL	120
#define GNSS_SAMPLE_PERIODIC_TIMEOUT	120
#define PI 3.14159265358979323846
#define EARTH_RADIUS_METERS (6371.0 * 1000.0)

struct nrf_modem_gnss_pvt_data_frame *pvt_data;

K_MUTEX_DEFINE(last_pvt_mutex);

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

//define channel
ZBUS_CHAN_DEFINE(acc_data_channel,
                struct nrf_modem_gnss_pvt_data_frame,
                NULL,
                NULL,
                ZBUS_OBSERVERS(bar_sub),
                ZBUS_MSG_INIT(0)
);
//define subscriber
ZBUS_SUBSCRIBER_DEFINE(bar_sub, 4);
//define subscriber task
static void subscriber_task(void)
{
    const struct zbus_channel *chan;

    while(!zbus_sub_wait(&bar_sub, &chan, K_FOREVER))
    {
        struct nrf_modem_gnss_pvt_data_frame *pvt_data = &last_pvt;

        if(&acc_data_channel == chan)
        {
            k_mutex_lock(&last_pvt_mutex, K_FOREVER);
            zbus_chan_read(&acc_data_channel, &last_pvt, K_MSEC(500));

            LOG_INF("From subscriber -> Latitude: %0.06f, Longitude: %0.06f, Altitude: %0.01f", pvt_data->latitude, pvt_data->longitude, pvt_data->altitude);

            k_mutex_unlock(&last_pvt_mutex);
        }
    }
}

K_THREAD_DEFINE(subscriber_task_id, CONFIG_MAIN_STACK_SIZE, subscriber_task, NULL, NULL, NULL, 3, 0, 0);


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
	LOG_INF("\nDistance from reference: %.01f\n", distance);
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

	LOG_INF("Tracking: %2d Using: %2d Unhealthy: %d\n", tracked, in_fix, unhealthy);
}

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	LOG_INF("Latitude:       %.06f\n", pvt_data->latitude);
	LOG_INF("Longitude:      %.06f\n", pvt_data->longitude);
	LOG_INF("Altitude:       %.01f m\n", (double)pvt_data->altitude);
	LOG_INF("Accuracy:       %.01f m\n", (double)pvt_data->accuracy);
	LOG_INF("Speed:          %.01f m/s\n", (double)pvt_data->speed);
	LOG_INF("Speed accuracy: %.01f m/s\n", (double)pvt_data->speed_accuracy);
	LOG_INF("Heading:        %.01f deg\n", (double)pvt_data->heading);
	LOG_INF("Date:           %04u-%02u-%02u\n",
	       pvt_data->datetime.year,
	       pvt_data->datetime.month,
	       pvt_data->datetime.day);
	LOG_INF("Time (UTC):     %02u:%02u:%02u.%03u\n",
	       pvt_data->datetime.hour,
	       pvt_data->datetime.minute,
	       pvt_data->datetime.seconds,
	       pvt_data->datetime.ms);
	LOG_INF("PDOP:           %.01f\n", (double)pvt_data->pdop);
	LOG_INF("HDOP:           %.01f\n", (double)pvt_data->hdop);
	LOG_INF("VDOP:           %.01f\n", (double)pvt_data->vdop);
	LOG_INF("TDOP:           %.01f\n", (double)pvt_data->tdop);
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

	for (;;) 
	{
		(void)k_poll(events, 2, K_FOREVER);
        LOG_INF("gets here 238");
		if (events[0].state == K_POLL_STATE_SEM_AVAILABLE && k_sem_take(events[0].sem, K_NO_WAIT) == 0) 
		{
				/* PVT and NMEA output mode. */
				LOG_INF("\033[1;1H");
				LOG_INF("\033[2J");
				print_satellite_stats(&last_pvt);

				if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED) 
				{
					LOG_INF("GNSS operation blocked by LTE\n");
				}
				if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) 
				{
					LOG_INF("Insufficient GNSS time windows\n");
				}
				if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_SLEEP_BETWEEN_PVT) 
				{
					LOG_INF("Sleep period(s) between PVT notifications\n");
				}
				//LOG_INF("gets here 258");
				LOG_INF("-----------------------------------\n");

				if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) 
				{
					LOG_WRN("gets here 260");
					fix_timestamp = k_uptime_get();
					//LOG_INF("gets here 266");
					print_fix_data(&last_pvt);
					//LOG_INF("gets here 266");
					print_distance_from_reference(&last_pvt);

					LOG_WRN("gets here 266");
					k_mutex_lock(&last_pvt_mutex, K_FOREVER);
					zbus_chan_pub(&acc_data_channel, &last_pvt, K_SECONDS(1));
					k_mutex_unlock(&last_pvt_mutex);
					LOG_WRN("gets here 270");
				} 
				else 
				{
					//LOG_INF("gets here 261");
					LOG_INF("Seconds since last fix: %d\n",
							(uint32_t)((k_uptime_get() - fix_timestamp) / 1000));
					cnt++;
					LOG_INF("Searching [%c]\n", update_indicator[cnt%4]);
				}
		}
	}
	return 0;
}