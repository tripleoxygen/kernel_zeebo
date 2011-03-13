#ifndef _MICROP_NG_H
#define _MICROP_NG_H

#include <linux/device.h>

/**
 * @brief platform-specific microp data
 * comp_versions is the array of compatible versions, i.e., [0x787, 0x600]
 * n_comp_versions is the length of comp_versions array
 * clients is the array of clients
 * nclients is the length of clients array
 */
struct microp_platform_data {
	unsigned char version_reg;
	uint16_t *comp_versions;
	int n_comp_versions;
	struct platform_device** clients;
	int nclients;
};

/**
 * @param client the pointer to the i2c client. It is passed to microp clients
 * as drvdata
 * @param sendbuf the buffer containing data to be transmitted
 * @param len the length of data to write
 * @return zero in case of success, negative error code otherwise
 */
extern int microp_ng_write(struct i2c_client *client, uint8_t* sendbuf, int len);

/**
 * @param client the pointer to the i2c client. It is passed to microp clients
 * as drvdata
 * @param id the address on the i2c device to receive data
 * @param buf the buffer to receive data
 * @param len the length of data to read
 * @return zero in case of success, negative error code otherwise
 */
extern int microp_ng_read(struct i2c_client *client, uint8_t id, uint8_t *buf, int len);

#endif
