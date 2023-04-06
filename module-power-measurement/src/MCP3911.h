//
// Created by dominik on 4/6/23.
//

#ifndef MODULE_POWER_MEASUREMENT_MCP3911_H
#define MODULE_POWER_MEASUREMENT_MCP3911_H

#include <inttypes.h>
#include <new>
#include <spi.h>

extern io::spi_t<io::spi_type_t::MASTER, io::spi_host_mode_t::NORMAL> spi;

namespace dev {
		class MCP3911 {
			public:
				enum reg_t {
						REG_CHANNEL_0   = 0x00,
						REG_CHANNEL_1   = 0x03,
						REG_MOD         = 0x06,
						REG_PHASE       = 0x07,
						REG_GAIN        = 0x09,
						REG_STATUSCOM   = 0x0a,
						REG_CONFIG      = 0x0c,
						REG_CONFIG2     = 0x0c,
						REG_OFFCAL_CH0  = 0x0e,
						REG_GAINCAL_CH0 = 0x11,
						REG_OFFCAL_CH1  = 0x14,
						REG_GAINCAL_CH1 = 0x17,
						REG_VREFCAL     = 0x1a
				};

				enum gain_t { GAINX1, GAINX2, GAINX4, GAINX8, GAINX16, GAINX32 };
				enum boost_t { BOOSTX05, BOOSTX066, BOOSTX1, BOOSTX2 };
				enum oversampling_t { O32, O64, O128, O256, O512, O1024, O2048, O4096 };
				enum prescale_t { MCLK1, MCLK2, MCLK4, MCLK8 };
				enum looping_t { REGISTER, GROUP, TYPE, ALL };
				enum dr_mode_t { LAGGING, CH0, CH1, CH_BOTH };
				enum dither_t { OFF, MIN, MED, MAX };

				enum channel_t { A, B };

				using ch_data_t = uint8_t[3];

				// conf_mod_t (0x06)
				using conf_mod_t = struct __attribute__( ( __packed__ ) ) {
						union {
								uint8_t reg;
								struct __attribute__( ( __packed__ ) ) {
										uint8_t ch0 : 4;
										uint8_t ch1 : 4;  // msb
								};
						};
				};

				// conf_phase_t (0x07,0x08)
				using conf_phase_t = struct __attribute__( ( __packed__ ) ) { uint8_t b[2]; };

				// conf_gain_t (0x09)
				using conf_gain_t = struct __attribute__( ( __packed__ ) ) {
						gain_t  ch0 : 3;
						gain_t  ch1 : 3;
						boost_t boost : 2;  // msb
				};

				// conf_gain_t (0x0a, 0x0b)
				using conf_status_t = struct __attribute__( ( __packed__ ) ) {
						bool      ch0_not_ready : 1;
						bool      ch1_not_ready : 1;
						dr_mode_t dr_mode : 2;
						bool      dr_pull_up : 1;
						bool : 1;
						bool ch0_modout : 1;  // p. 33
						bool ch1_modout : 1;  // msb

						bool : 1;
						bool gain_cal_24bit : 1;
						bool off_cal_24bit : 1;
						bool ch0_24bit_mode : 1;
						bool ch1_24bit_mode : 1;
						bool write_reg_incr : 1;
						long read_reg_incr : 2;  // msb
				};

				// config_t (0x0c, 0x0d)
				using config_t = struct __attribute__( ( __packed__ ) ) {
						bool           az_on_hi_speed : 1;  // p. 33
						dither_t       dither : 2;
						oversampling_t osr : 3;
						prescale_t     prescale : 2;  // msb 15
						bool : 1;
						bool clkext : 1;
						bool vrefext : 1;
						bool : 1;
						bool ch0_shtdn_mode : 1;
						bool ch1_shtdn_mode : 1;
						bool reset_ch0 : 1;
						bool reset_ch1 : 1;  // msb 7
				};

				// conf_off_cal_t (0x0e, 0x14)
				using conf_off_cal_t = struct __attribute__( ( __packed__ ) ) {
						uint8_t b[3];
				};

				// conf_gain_cal_t (0x011, 0x17)
				using conf_gain_cal_t = struct __attribute__( ( __packed__ ) ) {
						uint8_t b[3];
				};

				// conf_reg_map_t (27) (0x1b)
				using conf_reg_map_t = struct __attribute__( ( __packed__ ) ) {
						ch_data_t    ch[2];        // 0x00 3,3
						conf_mod_t   mod;          // 0x06 1
						conf_phase_t phase;        // 0x07 2
						conf_gain_t  gain;         // 0x09 1
						conf_status_t status;       // 0x0a 2
						config_t    config;       // 0x0c 2
						conf_off_cal_t offcal_ch0;   // 0x0e 3
						conf_gain_cal_t gaincal_ch0;  // 0x11 3
						conf_off_cal_t offcal_ch1;   // 0x14 3
						conf_gain_cal_t gaincal_ch1;  // 0x17 3
						uint8_t      vrefcal;      // 0x1a 1
				};

				conf_reg_map_t* config;

				double v_ref = 1.2f;

				void begin(void);

				void spi_transfer(uint8_t mode, uint8_t addr, uint8_t *buff, size_t count );

				inline void reset();



				bool reg_read(uint8_t addr, looping_t g, uint8_t count=0);

				bool reg_write(uint8_t addr, looping_t g, uint8_t count=0);

				void get_value(double *result, uint8_t channel);

			private:

				static uint32_t msb2l(void *src, uint8_t count);
				static void l2msb(uint32_t value, void *tgt, uint8_t count);

				io::pin_t<6, io::type_::DIGITAL> cs_ {1};
		};
}  // namespace dev


#endif  // MODULE_POWER_MEASUREMENT_MCP3911_H
