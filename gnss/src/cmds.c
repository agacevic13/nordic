#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cmds);

static int reset(const struct shell *sh, size_t argc, char **argv) {
	//sys_reboot(SYS_REBOOT_COLD);
	return 0;
}

extern int get_position(void);
static int get_position_cmd(const struct shell *sh, size_t argc, char **argv) {
	LOG_INF("requesting a single fix");
	get_position();
	return 0;
}

static int set_nwkId(const struct shell *sh, size_t argc, char **argv) {
	if (argc != 2 && argc != 1) {
		LOG_ERR("Invalid nwk id command...");
		return -1;
	}

	char nwk_id[5];
	nwk_id[4] = 0;
	// PawrBroadcaster_GetNwkId(nwk_id, 4);
	LOG_INF("Current nwk id: %s", nwk_id);

	if (argc == 1) {
		return 0;
	}

	LOG_INF("Requested nwk id: %s, len: %d", argv[1], strlen(argv[1]));
	// PawrBroadcaster_SetNwkId(argv[1], strlen(argv[1]));

	// PawrBroadcaster_GetNwkId(nwk_id, 4);
	LOG_INF("New nwk id: %s", nwk_id);

	return 0;
}

static int set_version(const struct shell *sh, size_t argc, char **argv) {
	if (argc != 2 && argc != 1) {
		LOG_ERR("Invalid nwk id command...");
		return -1;
	}

	char version[16];
	version[15] = 0;
	// PawrBroadcaster_GetVersion(version, sizeof(version));
	LOG_INF("Current version: %s", version);

	if (argc == 1) {
		return 0;
	}

	LOG_INF("Requested version: %s, len: %d", argv[1], strlen(argv[1]));
	// PawrBroadcaster_SetVersion(argv[1], strlen(argv[1]));

	// PawrBroadcaster_GetVersion(version, sizeof(version));
	LOG_INF("New version: %s", version);

	return 0;
}

static int set_phy(const struct shell *sh, size_t argc, char **argv) {
	if (argc == 1) {
		LOG_INF("Current phy: %s", true ? "coded" : "1m");
		return 0;
	}
	else if (argc != 2) {
		LOG_ERR("Invalid version command...");
	}

	LOG_INF("Current phy: %s", true ? "coded" : "1m");
	LOG_INF("Requested phy: %s", atoi(argv[1]) == 1 ? "coded" : "1m");
	// PawrBroadcaster_SetPhy(atoi(argv[1]));
	LOG_INF("New phy: %s", true ? "coded" : "1m");

	return 0;
}

static int set_pollRate(const struct shell *sh, size_t argc, char **argv) {
	if (argc != 2) {
		LOG_ERR("Invalid arguments");
		return -EINVAL;
	}

    // Convert strings to integers using atoi
    uint16_t pollRate = (uint16_t)atoi(argv[1]);
	// PawrBroadcaster_SetPollingRate(pollRate);
	return 0;
}

static int set_params(const struct shell *sh, size_t argc, char **argv) {
	if (argc != 5) {
		LOG_ERR("Invalid arguments");
		return -EINVAL;
	}

    // Convert strings to integers using atoi
    uint8_t subeventInterval = (uint8_t)atoi(argv[1]);
    uint8_t rspSlotDelay = (uint8_t)atoi(argv[2]);
    uint8_t rspSlotSpacing = (uint8_t)atoi(argv[3]);
    uint8_t nRspSlots = (uint8_t)atoi(argv[4]);

	// PawrBroadcaster_SetPawrParams(subeventInterval, rspSlotDelay, rspSlotSpacing, nRspSlots);
	return 0;
}

static int set_timeout(const struct shell *sh, size_t argc, char **argv) {
	if (argc != 2) {
		LOG_ERR("Invalid arguments");
		return -EINVAL;
	}

    // Convert strings to integers using atoi
    uint32_t timeout = (uint16_t)atoi(argv[1]);
	// PawrBroadcaster_SetSlotTimeoutInSeconds(timeout);
	return 0;
}

SHELL_CMD_REGISTER(reset, NULL, "Reset the chip", reset);
SHELL_CMD_REGISTER(get_position, NULL, "Start broadcaster", get_position_cmd);
SHELL_CMD_REGISTER(nwkid, NULL, "Set nwk_id. Requires 2 arguments, argv[1] length must be 4. Default value: 1234.", set_nwkId);
SHELL_CMD_REGISTER(version, NULL, "Modifies version of the broadcaster. Example: ", set_version);
SHELL_CMD_REGISTER(phy, NULL, "Modifies phy of the broadcaster. Example: ", set_phy);
SHELL_CMD_REGISTER(pollRate, NULL, "Set poll rate. Example: ", set_pollRate);
SHELL_CMD_REGISTER(params, NULL, "Modifies adv parameters. Example: ", set_params);
SHELL_CMD_REGISTER(timeout, NULL, "Modifies adv parameters. Example: ", set_timeout);
