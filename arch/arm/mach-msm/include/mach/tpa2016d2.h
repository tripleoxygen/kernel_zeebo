/*
 * Definitions for tpa2016d2 speaker amp chip.
 */
#ifndef TPA2016D2_H
#define TPA2016D2_H

#include <linux/ioctl.h>

#define TPA2016D2_I2C_NAME "tpa2016d2"

#define IC_REG		0x1
#define ATK_REG		0x2
#define REL_REG		0x3
#define HOLD_REG	0x4
#define FIXED_GAIN_REG	0x5
#define AGC_REG1	0x6
#define AGC_REG2	0x7


/* Registers map
 *
 *  Address | Name                  | Bit 7      | Bit 6      | Bit 5      | Bit 4      | Bit 3      | Bit 2      | Bit 1      | Bit 0      |
 *  --------+-----------------------+------------+------------+------------+------------+------------+------------+------------+------------+
 *  0x1     | IC FUNCTION CONTROL   | SPKR_EN_R  | SPKR_EN_L  | SWS        | FAULT_R    | FAULT_L    | Thermal    | UNUSED     | NG_EN      |
 *  0x2     | AGC ATTACK CONTROL    | Unused     | Unused     | ATK_time[5]| ATK_time[4]| ATK_time[3]| ATK_time[2]| ATK_time[1]| ATK_time[0]|
 *  0x3     | AGC RELEASE CONTROL   | Unused     | Unused     | REL_time[5]| REL_time[4]| REL_time[3]| REL_time[2]| REL_time[1]| REL_time[0]|
 *  0x4     | AGC HOLD TIME CONTROL | Unused     | Unused     | HLD_time[5]| HLD_time[4]| HLD_time[3]| HLD_time[2]| HLD_time[1]| HLD_time[0]|
 *  0x5     | AGC FIXED GAIN CONTROL| Unused     | Unused     | Fix_gain[5]| Fix_gain[4]| Fix_gain[3]| Fix_gain[2]| Fix_gain[1]| Fix_gain[0]|
 *  0x6     | AGC CONTROL 1         | Out Lim Dis| NoisGateTh1| NoisGateTh0| Out limlev4| Out limlev3| Out limlev2| Out limlev1| Out limlev0|
 *  0x7     | AGC CONTROL 2         | Max Gain[3]| Max Gain[2]| Max Gain[1]| Max Gain[0]| Unused     | Unused     | CompRatio1 | CompRatio0 |
 *
 */

#define SPK_EN_L 	1<<6
#define SPK_EN_R 	1<<7
#define SWS_BIT                     0x40
#define NOISE_GATE_ENABLE_BIT       0x01


#define AGC_ATTACK_BITS             0x3F
#define AGC_RELEASE_BITS            0x3F
#define AGC_HOLDTIME_BITS           0x3F
#define AGC_FIXEDGAIN_BITS          0x3F
/* AGC CONTROL 1 */
#define OUT_LIMITER_DISABLE         0x80
#define NOISE_GATE_THRESOLD_BITS    0x60
#define OUT_LIMITER_LEVEL_BITS      0x1F
/* AGC CONTROL 2 */
#define MAX_GAIN_BITS               0xF0
#define COMPRESSION_RATIO_BITS      0x03


struct tpa2016d2_platform_data {
	uint32_t gpio_tpa2016_spk_en;
};

#define TPA2016_IOCTL_MAGIC 'a'
#define TPA2016_SET_POWER   _IOW(TPA2016_IOCTL_MAGIC, 0x01,	unsigned)
#define TPA2016_SET_CONFIG  _IOW(TPA2016_IOCTL_MAGIC, 0x02,	unsigned char*)
#define TPA2016_READ_CONFIG _IOW(TPA2016_IOCTL_MAGIC, 0x03, unsigned char*)

/* Exported functions */
void tpa2016d2_set_power(bool bOn);
void tpa2016d2_read_register(uint8_t reg, uint8_t* arg);
void tpa2016d2_set_register(uint8_t reg, uint8_t arg);

#endif

