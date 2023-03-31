//
// Created by dominik on 3/31/23.
//

#ifndef MODULE_POWER_MEASUREMENT_SPI_H
#define MODULE_POWER_MEASUREMENT_SPI_H

#include <avr/io.h>

namespace io {
    enum class dir_t {
        INPUT,
        OUTPUT
    };

    enum class spi_host_mode_t {
        NORMAL,
        BUFFERED
    };

    enum spi_transfer_mode_t {
        MODE0 = 0b0000'0000,
        MODE1 = 0b0000'0001,
        MODE2 = 0b0000'0010,
        MODE3 = 0b0000'0011
    };

    template <dir_t Dir, spi_host_mode_t HMode, spi_transfer_mode_t TMode = MODE0>
    class spi {
    public:
        spi(size_t clock) {
            if constexpr(Dir == io::dir_t::OUTPUT) {
                //MASTER bit in SPIn.CTRLA
                SPI0.CTRLA |= SPI_MASTER_bm;
                if constexpr(HMode == io::spi_host_mode_t::NORMAL) {
                    //writes & Reads to SPIn.DATA
                    //Receive data buffer
                } else {
                    //Bufen SPIn.CTRLB
                    SPI0.CTRLB |= SPI_BUFEN_bm;


                }
            } else {

            }
            SPI0.CTRLB |= TMode;
            SPI0.CTRLB |= SPI_SSD_bm;
            SPI0.CTRLA |= SPI_ENABLE_bm;
            SPI0.INTCTRL |= (SPI_RXCIF_bm | );
            //SPI0.CTRLA |= SPI_PRESC_enum
            //CLKCTRL.MCLKCTRLA |= PRESC
        }

        void write(char ) {

        }

        char read() {
            return
        }

        if()

    private:
    };
}

#endif //MODULE_POWER_MEASUREMENT_SPI_H
