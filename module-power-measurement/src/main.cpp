#define F_CPU 20'000'000ull
#include <avr/delay.h>

#include <io.h>
#include <spi.h>
#include <usart.h>

//#include <SPI.h>

#include "MCP3911.h"


//constexpr int OUTPUT = 1, INPUT = 0;

io::spi_t<io::spi_type_t::MASTER, io::spi_host_mode_t::NORMAL> spi{4'000'000, io::spi_bit_order_t::MSBFIRST, io::spi_transfer_mode_t::MODE0};

int main() {

		//init();

		_PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0);

		io::usart<io::usart_mode_t::FULL_DUPLEX, 0> usart{9600};

		dev::MCP3911 adc;
		dev::MCP3911::conf_reg_map_t config;

		adc.begin();

		{   using namespace dev;

				// set register map ptr
				adc.config = &config;

				adc.reg_read(MCP3911::REG_STATUSCOM, MCP3911::REGISTER, 2);
				config.status.read_reg_incr  = MCP3911::ALL;
				config.status.write_reg_incr = MCP3911::ALL;
				adc.reg_write(MCP3911::REG_STATUSCOM, MCP3911::REGISTER, 2);

				// read default values
				adc.reg_read(MCP3911::REG_CHANNEL_0, MCP3911::ALL);

				config.gain.ch0              = MCP3911::GAINX1;
				config.gain.boost            = MCP3911::BOOSTX1;

				config.status.ch0_modout     = 0;
				config.status.dr_pull_up     = 1;

				config.config.az_on_hi_speed = 1;
				config.config.clkext         = 1;
				config.config.vrefext        = 0;
				config.config.osr            = MCP3911::O4096;
				config.config.prescale       = MCP3911::MCLK4;
				config.config.dither         = MCP3911::MAX;

				config.status.read_reg_incr = MCP3911::ALL;
				config.status.write_reg_incr = MCP3911::TYPE;

				// vref adjustment
				config.vrefcal += 0x11;

				// write out all regs
				adc.reg_write(MCP3911::REG_MOD, MCP3911::TYPE);

				// set grouping for TYPE
				config.status.read_reg_incr  = MCP3911::TYPE;
				adc.reg_write(MCP3911::REG_STATUSCOM, MCP3911::REGISTER, 2);
		}

		double v0,v1;

		//led = 1;
		//io::pin_t<1, io::type_::DIGITAL> led{1};
		//led = 1;
		while(true) {
			//adc.reg_read(dev::MCP3911::REG_CHANNEL_0, dev::MCP3911::TYPE);
			uint8_t stacom[2];

			//led = !led();
			adc.spi_transfer(0, dev::MCP3911::REG_STATUSCOM, stacom, 2);

			//adc.get_value(&v0, 0);
			//adc.get_value(&v1, 1);

			//char buf[9] {0};
			//dtostrf(v0, 9, 8, buf);
			usart.write(stacom[0]);
			usart.write(stacom[1]);
			usart.write("\n\r");

			_delay_ms(250);
		}
}