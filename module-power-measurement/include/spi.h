//
// Created by dominik on 3/31/23.
//

#ifndef MODULE_POWER_MEASUREMENT_SPI_H
#define MODULE_POWER_MEASUREMENT_SPI_H

#include <avr/io.h>
#include <io.h>

namespace io {
    enum class spi_type_t {
        SLAVE,
        MASTER
    };

    enum class spi_host_mode_t {
        NORMAL,
        BUFFERED
    };

    enum class spi_bit_order_t {
        LSBFIRST,
        MSBFIRST
    };

    enum spi_transfer_mode_t {
        MODE0 = SPI_MODE_enum::SPI_MODE_0_gc,
        MODE1 = SPI_MODE_enum::SPI_MODE_1_gc,
        MODE2 = SPI_MODE_enum::SPI_MODE_2_gc,
        MODE3 = SPI_MODE_enum::SPI_MODE_3_gc
    };

    enum spi_clock_divider_t {
        DIV4 = SPI_PRESC_enum::SPI_PRESC_DIV4_gc,
        DIV16 = SPI_PRESC_enum::SPI_PRESC_DIV16_gc,
        DIV64 = SPI_PRESC_enum::SPI_PRESC_DIV64_gc,
        DIV128 = SPI_PRESC_enum::SPI_PRESC_DIV128_gc,
    };

    template <spi_type_t Type, spi_host_mode_t HMode = spi_host_mode_t::BUFFERED>
    class spi_t {
    public:
        constexpr explicit spi_t(int32_t clock = 4'000'000, spi_bit_order_t bit_order = spi_bit_order_t::MSBFIRST, spi_transfer_mode_t transfer_mode = spi_transfer_mode_t::MODE0) {
            PORTMUX.SPIROUTEA = 0x0;
						SPI0.CTRLA = 0;
						SPI0.CTRLB = 0;

						set_clock_speed(clock);
						set_bit_order(bit_order);
						set_transfer_mode(transfer_mode);

            if constexpr(Type == io::spi_type_t::MASTER) {
                //MASTER bit in SPIn.CTRLA
                SPI0.CTRLA |= SPI_MASTER_bm;        //Set HMode to MASTER
                SPI0.CTRLB |= SPI_SSD_bm;           //SPI select disable

                if constexpr(HMode == io::spi_host_mode_t::NORMAL) {
                    //writes & Reads to SPIn.DATA
                    //Receive data buffer
                } else {
                    //Bufen SPIn.CTRLB
                    SPI0.CTRLB |= SPI_BUFEN_bm;
                }
            } else {

            }
						PORTA.DIRCLR |= ( 1 << PIN1_bm );				//Wenn richtungs bit vorhanden, löschen
						PORTA.DIRSET |= ( 1 << PIN1_bm );			//Setzen
						PORTA.DIRCLR |= ( 1 << PIN3_bm );				//Wenn richtungs bit vorhanden, löschen
						PORTA.DIRSET |= ( 1 << PIN3_bm );			//Setzen

            SPI0.CTRLA |= SPI_ENABLE_bm;        //Enable SPI
        }

        void set_bit_order(spi_bit_order_t order) {
            if(order == spi_bit_order_t::LSBFIRST) {
                SPI0.CTRLA |= (SPI_DORD_bm);
            } else {
                SPI0.CTRLA &= ~(SPI_DORD_bm);
            }
        }

        void set_transfer_mode(spi_transfer_mode_t mode) {
            SPI0.CTRLB = ((SPI0.CTRLB & (~SPI_MODE_gm)) | mode);
        }

        void set_clock_divider(spi_clock_divider_t divider) {
            SPI0.CTRLA = ((SPI0.CTRLA &
                           (~(SPI_PRESC_gm | SPI_CLK2X_bm)))   // mask out values
                          | divider);                           // write value
        }

        void set_clock_divider(uint8_t divider) {
            SPI0.CTRLA = ((SPI0.CTRLA &
                           (~(SPI_PRESC_gm | SPI_CLK2X_bm)))   // mask out values
                          | divider);                           // write value
        }

        void set_clock_speed(uint32_t clock) {
            set_clock_divider(find_clock_config(clock));
        }

        int8_t transfer(uint8_t data) {
            asm volatile("nop");
            SPI0.DATA = data;
            while ((SPI0.INTFLAGS & SPI_RXCIF_bm) == 0);  // wait for complete send
            return SPI0.DATA;                             // read data back
        }

        uint16_t transfer16(uint16_t data) {
            union {
                uint16_t val;
                struct {
                    uint8_t lsb;
                    uint8_t msb;
                };
            } t;

            t.val = data;

            if ((SPI0.CTRLA & SPI_DORD_bm) == 0) {
                t.msb = transfer(t.msb);
                t.lsb = transfer(t.lsb);
            } else {
                t.lsb = transfer(t.lsb);
                t.msb = transfer(t.msb);
            }

            return t.val;
        }

        void transfer(void* buf, size_t count) {
            uint8_t *buffer = reinterpret_cast<uint8_t *>(buf);
            for (size_t i = 0; i < count; i++) {
                *buffer = transfer(*buffer);
                buffer++;
            }                        // read data back
        }

			private:

				constexpr uint8_t find_clock_config(const uint32_t clock) {
						const int div[] = {128, 64, 16, 4};
						uint32_t presc_clk = 0;

						uint8_t i = 0, i_old = i, j_old = 1;

						for(i; i < 4; ++i) {
								for(int j = 1; j <= 2; ++j) {
										if( j * F_CPU / div[i] <= clock) {
												i_old = i;
												j_old = j;
										} else {
												return ((SPI_CLK2X_bm * (j_old - 1)) | ((3 - i_old) << 1));
										}
								}
						}
				}
    };
}

#endif //MODULE_POWER_MEASUREMENT_SPI_H
