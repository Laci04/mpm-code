//
// Created by dominik on 4/6/23.
//

#include "MCP3911.h"

void dev::MCP3911::begin( void ) {
		v_ref = 1.2023f;

		cs_ = 1;

		//spi.set_clock_divider(io::spi_clock_divider_t::DIV16);
		spi.set_transfer_mode(io::spi_transfer_mode_t::MODE0);

		reset();
}

void dev::MCP3911::spi_transfer(uint8_t mode, uint8_t addr, uint8_t *buff, size_t count ) {
		cs_ = 0;
		spi.transfer((addr << 1) | !mode);
		while (count--) {
				if ( mode == 1 )
						spi.transfer( *buff++ );
				else
						*buff++ = spi.transfer( 0xff );
		}
		cs_ = 1;
}

void dev::MCP3911::reset() {
		const uint8_t r = 0b0000'0000;
		spi_transfer(1, REG_CONFIG2, (uint8_t *)&r, 1); // 6.7.2 p. 47
}
bool dev::MCP3911::reg_read( uint8_t addr, dev::MCP3911::looping_t g, uint8_t count ) {
		uint8_t *data = ((uint8_t *)config)+addr;

		if (count == 0) {
				switch (g) {
						case REGISTER: count = 1;
								break;
						case GROUP: switch (addr) {
										case REG_CHANNEL_0:
										case REG_CHANNEL_1:  count = 3; break;
										case REG_MOD:
										case REG_STATUSCOM:  count = 4; break;
										case REG_OFFCAL_CH0:
										case REG_OFFCAL_CH1: count = 6; break;
										case REG_VREFCAL:    count = 1; break;
								}
								break;
						case TYPE: switch (addr) {
										case REG_CHANNEL_0: count = 6; break;
										case REG_MOD: count = 21; break;
								}
								break;
						case ALL: switch (addr) {
										case REG_CHANNEL_0: count = 0x1b; break;
								}
								break;
						default: return false;
				}
		}

		spi_transfer(0, addr, data, count);

		return true;
}
bool dev::MCP3911::reg_write( uint8_t addr, dev::MCP3911::looping_t g, uint8_t count ) {

		uint8_t *data = ((uint8_t *)config)+addr;

		if (count == 0) {
				switch (g) {
						case REGISTER: count = 1;
								break;
						case TYPE: switch (addr) {
										case REG_MOD: count = 21; break;
								}
								break;
						case GROUP: break;
						case ALL: break;
						default: return false;
				}}

		spi_transfer(1, addr, data, count);

		return true;
}
void dev::MCP3911::get_value( double *result, uint8_t channel ) {

		ch_data_t& val = (*config).ch[channel];

		int32_t data_ch = msb2l((uint8_t *)&val, 3);

		if (data_ch & 0x00800000L)
				data_ch |= 0xff000000L;

		uint8_t gain = (channel == 0)?(*config).gain.ch0:(*config).gain.ch1;

		*result = v_ref * data_ch / ( (3*32768*256/2) * (1 << gain) );
}
uint32_t dev::MCP3911::msb2l( void *src, uint8_t count ) {
		uint32_t value = 0L;
		uint8_t *b = (uint8_t *)src;

		while (count--) {
				value <<= 8;
				value |= *b++;
		}
		return value;
}
void dev::MCP3911::l2msb( uint32_t value, void *tgt, uint8_t count ) {
		uint32_t v = value;
		uint8_t *b = (uint8_t *) tgt;

		while ( 1 ) {
				count--;
				*( b + count ) = (uint8_t) ( v & 0xff );
				if ( count == 0 )
						break;
				v >>= 8;
		}
}
