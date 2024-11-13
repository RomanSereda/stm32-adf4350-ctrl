#ifndef INC_ADF4350_H_
#define INC_ADF4350_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define ADF4350_REG0	0
#define ADF4350_REG1	1
#define ADF4350_REG2	2
#define ADF4350_REG3	3
#define ADF4350_REG4	4
#define ADF4350_REG5	5

#define ADF4350_REG0_FRACT(x)					(((x) & 0xFFF) << 3)
#define ADF4350_REG0_INT(x)						(((x) & 0xFFFF) << 15)

#define ADF4350_REG1_MOD(x)						(((x) & 0xFFF) << 3)
#define ADF4350_REG1_PHASE(x)					(((x) & 0xFFF) << 15)
#define ADF4350_REG1_PRESCALER					(1 << 27)

#define ADF4350_REG2_COUNTER_RESET_EN			(1 << 3)
#define ADF4350_REG2_CP_THREESTATE_EN			(1 << 4)
#define ADF4350_REG2_POWER_DOWN_EN				(1 << 5)
#define ADF4350_REG2_PD_POLARITY_POS			(1 << 6)
#define ADF4350_REG2_LDP_6ns					(1 << 7)
#define ADF4350_REG2_LDP_10ns					(0 << 7)
#define ADF4350_REG2_LDF_FRACT_N				(0 << 8)
#define ADF4350_REG2_LDF_INT_N					(1 << 8)
#define ADF4350_REG2_CHARGE_PUMP_CURR_uA(x)		(((((x)-312) / 312) & 0xF) << 9)
#define ADF4350_REG2_DOUBLE_BUFF_EN				(1 << 13)
#define ADF4350_REG2_10BIT_R_CNT(x)				((x) << 14)
#define ADF4350_REG2_RDIV2_EN					(1 << 24)
#define ADF4350_REG2_RMULT2_EN					(1 << 25)
#define ADF4350_REG2_MUXOUT(x)					((x) << 26)
#define ADF4350_REG2_NOISE_MODE(x)				(((x) & 0x3) << 29)

#define ADF4350_REG3_12BIT_CLKDIV(x)			((x) << 3)
#define ADF4350_REG3_12BIT_CLKDIV_MODE(x)		((x) << 16)
#define ADF4350_REG3_12BIT_CSR_EN				(1 << 18)
#define ADF4351_REG3_CHARGE_CANCELLATION_EN		(1 << 21)
#define ADF4351_REG3_ANTI_BACKLASH_3ns_EN		(1 << 22)
#define ADF4351_REG3_BAND_SEL_CLOCK_MODE_HIGH	(1 << 23)

#define ADF4350_REG4_OUTPUT_PWR(x)				((x) << 3)
#define ADF4350_REG4_RF_OUT_EN					(1 << 5)
#define ADF4350_REG4_AUX_OUTPUT_PWR(x)			((x) << 6)
#define ADF4350_REG4_AUX_OUTPUT_EN				(1 << 8)
#define ADF4350_REG4_AUX_OUTPUT_FUND			(1 << 9)
#define ADF4350_REG4_AUX_OUTPUT_DIV				(0 << 9)
#define ADF4350_REG4_MUTE_TILL_LOCK_EN			(1 << 10)
#define ADF4350_REG4_VCO_PWRDOWN_EN				(1 << 11)
#define ADF4350_REG4_8BIT_BAND_SEL_CLKDIV(x)	((x) << 12)
#define ADF4350_REG4_RF_DIV_SEL(x)				((x) << 20)
#define ADF4350_REG4_FEEDBACK_DIVIDED			(0 << 23)
#define ADF4350_REG4_FEEDBACK_FUND				(1 << 23)

#define ADF4350_REG5_LD_PIN_MODE_LOW			(0 << 22)
#define ADF4350_REG5_LD_PIN_MODE_DIGITAL		(1 << 22)
#define ADF4350_REG5_LD_PIN_MODE_HIGH			(3 << 22)

#define ADF4350_MAX_OUT_FREQ		4400000000ULL /* Hz */
#define ADF4350_MIN_OUT_FREQ		34375000 /* Hz */
#define ADF4350_MIN_VCO_FREQ		2200000000ULL /* Hz */
#define ADF4350_MAX_FREQ_45_PRESC	3000000000ULL /* Hz */
#define ADF4350_MAX_FREQ_PFD		32000000 /* Hz */
#define ADF4350_MAX_BANDSEL_CLK		125000 /* Hz */
#define ADF4350_MAX_FREQ_REFIN		250000000 /* Hz */
#define ADF4350_MAX_MODULUS			4095
#define ADF4350_MAX_R_CNT			1023

struct adf4350_platform_data {
	uint32_t	clkin;
	uint32_t	channel_spacing;
	uint64_t	power_up_frequency;

	uint16_t	ref_div_factor; /* 10-bit R counter */
	uint8_t	    ref_doubler_en;
	uint8_t	    ref_div2_en;

	uint32_t    r2_user_settings;
	uint32_t    r3_user_settings;
	uint32_t    r4_user_settings;
	int32_t	    gpio_lock_detect;
};

typedef struct {
	uint32_t	clkin;
	uint32_t	channel_spacing;
	uint32_t	power_up_frequency;
	uint32_t	reference_div_factor;
	uint8_t		reference_doubler_enable;
	uint8_t		reference_div2_enable;

	uint8_t		phase_detector_polarity_positive_enable;
	uint8_t		lock_detect_precision_6ns_enable;
	uint8_t		lock_detect_function_integer_n_enable;
	uint32_t	charge_pump_current;
	uint32_t	muxout_select;
	uint8_t		low_spur_mode_enable;

	uint8_t		cycle_slip_reduction_enable;
	uint8_t		charge_cancellation_enable;
	uint8_t		anti_backlash_3ns_enable;
	uint8_t		band_select_clock_mode_high_enable;
	uint32_t	clk_divider_12bit;
	uint32_t	clk_divider_mode;

	uint8_t		aux_output_enable;
	uint8_t		aux_output_fundamental_enable;
	uint8_t		mute_till_lock_enable;
	uint32_t	output_power;
	uint32_t	aux_output_power;
} adf4350_init_param;

typedef struct {
	SPI_HandleTypeDef* hspi;
	struct adf4350_platform_data *pdata;
	uint32_t	clkin;
	uint32_t	chspc;
	uint32_t	fpfd;
	uint32_t	min_out_freq;
	uint32_t	r0_fract;
	uint32_t	r0_int;
	uint32_t	r1_mod;
	uint32_t	r4_rf_div_sel;
	uint32_t	regs[6];
	uint32_t	regs_hw[6];
	uint32_t 	val;
} adf4350_dev;

int32_t adf4350_setup(adf4350_dev **device, SPI_HandleTypeDef* hspi, adf4350_init_param init_param);

int64_t adf4350_frequency(adf4350_dev *dev, int64_t Hz);

int32_t adf4350_frequency_resolution(adf4350_dev *dev, int32_t Hz);

int64_t adf4350_refin_frequency(adf4350_dev *dev, int64_t Hz);

int32_t adf4350_powerdown(adf4350_dev *dev, int32_t pwd);

#endif /* INC_ADF4350_H_ */
