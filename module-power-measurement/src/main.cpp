// Created by Dominik Rzecki

/*
 * This work is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
 * This work is distributed in the hope that it will be useful, but without any warranty; without even the implied
 * warranty of merchantability or fitness for a particular purpose. See version 3 of the GNU General Public License for more
 * details. You should have received a copy of the GNU General Public License along with this program; if not, see www.gnu.org/licenses/
 *
 * Special thanks to Bernhard Schneider for making his MCP3911 library available under the MIT license.
 * */

// IDE pragmas
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

// Define Master CPU Clock
#define F_CPU 20'000'000ull

// avr includes
#include <avr/delay.h>
#include <avr/interrupt.h>

// include required coms
#include <io.h>
#include <spi.h>
#include <usart.h>

// for doing maths
#include <math.h>
#include <complex.h>

// include MCP3911 library
#include <MCP3911.h>

// adc configuration
dev::MCP3911::C adc;
dev::MCP3911::_ConfRegMap config;

// SPI object
io::spi_t<io::spi_type_t::MASTER, io::spi_host_mode_t::NORMAL> spi{4'000'000, io::spi_bit_order_t::MSBFIRST,
                                                                   io::spi_transfer_mode_t::MODE0};
// USART object (8N1 configuration by default)
io::usart<io::usart_mode_t::FULL_DUPLEX, 0> usart{921600};

// Sample request variable
volatile bool update_pending = false;

// Missing sample counter
volatile int32_t miss_cnt = 0;

// Reset function
void (*reset_func)(void) = 0;

// timestep type
struct __attribute__((packed, aligned(1))) timestep_t {
    int32_t ch[2]{0l};
};

/*
 * Compile time generation of arrays possible since C++17
 * The following constexpr constructor generates array of Normalized frequencies
 * for the requested harmonics at compile time
 */
template<class T, size_t N, int FS>
struct Omega {
    constexpr Omega() : arr() {
        for (size_t i = 0; i < N; ++i) {
            float &&k = (48.979f * (i + 1)) / (float) FS;
            arr[i] = 2 * M_PI * k;
        }
    }

    T arr[N];
};

int main() {
    //Enable 20MHz MASTER Clock and disable all prescalers
    _PROTECTED_WRITE(CLKCTRL_MCLKCTRLA, 0);
    _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0);

    // Setup MCP3911
    adc.begin();

    {
        using namespace dev;

        // set register map ptr
        adc._c = &config;

        adc.reg_read(MCP3911::REG_STATUSCOM, MCP3911::REGISTER, 2);
        config.status.read_reg_incr = MCP3911::ALL;
        config.status.write_reg_incr = MCP3911::ALL;
        adc.reg_write(MCP3911::REG_STATUSCOM, MCP3911::REGISTER, 2);

        // read default values
        adc.reg_read(MCP3911::REG_CHANNEL_0, MCP3911::ALL);

        config.gain.ch0 = MCP3911::GAINX1;
        config.gain.boost = MCP3911::BOOSTX1;

        config.status.ch0_modout = 0;
        config.status.dr_pull_up = 1;

        config.config.az_on_hi_speed = 1;
        config.config.clkext = 1;
        config.config.vrefext = 0;
        config.config.osr = MCP3911::O4096;
        config.config.prescale = MCP3911::MCLK4;
        config.config.dither = MCP3911::MAX;

        config.status.read_reg_incr = MCP3911::ALL;
        config.status.write_reg_incr = MCP3911::TYPE;

        // vref adjustment
        config.vrefcal += 0x11;

        // write out all regs
        adc.reg_write(MCP3911::REG_MOD, MCP3911::TYPE);

        // set grouping for TYPE
        config.status.read_reg_incr = MCP3911::TYPE;
        adc.reg_write(MCP3911::REG_STATUSCOM, MCP3911::REGISTER, 2);

    }

    cli(); // Stop emitting interrupts

    // RTC configuration
    // Enable 32kHz Oscillator
    CLKCTRL_OSC32KCTRLA |= CLKCTRL_RUNSTDBY_bm;
    // Choose 32kHz oscillator as RTC source
    RTC.CLKSEL = RTC_CLKSEL_enum::RTC_CLKSEL_INT32K_gc;
    // Choose RTC prescaler
    RTC.CTRLA |= RTC_PRESCALER_enum::RTC_PRESCALER_DIV8_gc;
    // Set periodic timer interrupt
    RTC.PITINTCTRL |= RTC_PI_bm;
    // Set periodic timer interrupt period (total downscale = PRESCALER * PERIOD) & enable PIT
    RTC.PITCTRLA = RTC_PERIOD_CYC32_gc | RTC_PITEN_bm;



    // Wait 1 second
    _delay_ms(1000);

    // Setup MCP3911
    adc.begin();

    // Define register configurations
    uint8_t conf_b[] = {0b1101'0110, 0b0000'0000};
    uint8_t statuscom_b[] = {0b0001'000, 0b1011'1000};
    uint8_t gain_b = {0b1100'0000};

    // Configure registers
    adc.SPI_write(dev::MCP3911::REG_CONFIG, conf_b, 2);
    adc.SPI_write(dev::MCP3911::REG_STATUSCOM, statuscom_b, 2);
    adc.SPI_write(dev::MCP3911::REG_GAIN, &gain_b, 1);

    // Definitions of constants
    constexpr int FS = 1024;                            // Sampling frequency
    constexpr size_t N_f = 1;                           // Number of Frequencies to transform (base + harmonics)
    constexpr auto omega = Omega<float, N_f, FS>{};     // Generate required normalized frequencies at compile time

    // Update request variable for new sample
    update_pending = false;

    // Temporary storage for sample data
    timestep_t data{0};

    // Complex voltage and current accumulation variable
    math::complex<float> current{0, 0}, voltage{0, 0};

    // Sample counter
    uint16_t n = 0;

    sei(); // Start emitting interrupts

    // Infinite program loop
    while (true) {
        // sample FS samples and then output
        if (n < FS) {
            if (update_pending) { // Update pending? (See ISR)
                adc.SPI_read(dev::MCP3911::REG_STATUSCOM, ((uint8_t *) (adc._c)) + dev::MCP3911::REG_STATUSCOM,
                             2); // Read ADC status
                if (adc._c->status.ch0_24bit_mode) {    // Check if adc aviable
                    if (!adc._c->status.ch0_not_ready && !adc._c->status.ch1_not_ready) { // If ADC ready ...
                        update_pending = false;

                        // Read channels
                        adc.SPI_read(dev::MCP3911::REG_CHANNEL_0, ((uint8_t *) (adc._c)) + dev::MCP3911::REG_CHANNEL_0,
                                     6);

                        // Convert bytes to integer
                        for (size_t i = 0; i < 2; ++i) {
                            auto &val = (*adc._c).ch[i];
                            data.ch[i] = adc.msb2l((uint8_t *) &(val), 3);
                        }

                        // Calibrate
                        float v_v = (data.ch[1] / (8388608l * 1.5f)) * 1.2023f * 660;      //656; Spannungsberechnung
                        float v_c = ((data.ch[0] / (8388608l * 1.5f)) * 1.2023f) / 0.05f;    //Stromberechnung

                        // Do fft for 50, 100 and 150 .. Hz
                        for (float i: omega.arr) {
                            voltage = voltage + math::complex<float>{cos(-i * n) * v_v, sin(-i * n) * v_v};
                            current = current + math::complex<float>{cos(-i * n) * v_c, sin(-i * n) * v_c};
                        }
                        // Increment sample counter
                        n = n + 1;
                    }
                } else {
                    // If MCP3911 not found, perform reset
                    usart.write("@\n\rdevice: MCP3911 not found!\n\rreset...$");
                    reset_func();
                }
            }
        } else {
            // Stop emitting interrupts
            cli();

            // Reset sample counter
            n = 0;

            // Compute phase difference and voltage amplitudes
            float phase = current.arg() - voltage.arg();
            float voltage_ = (voltage.abso() / FS) * 2; // Times two in order to account for both sides of the spectrum
            float current_ = (current.abso() / FS) * 2;

            // Output data on usart
            usart.write('#');
            usart.write((uint8_t *) &miss_cnt, 4);
            usart.write((uint8_t *) &voltage_, 4);
            usart.write((uint8_t *) &current_, 4);
            usart.write((uint8_t *) &phase, 4);

            // Reset voltage & current values
            voltage.re = 0;
            voltage.im = 0;
            current.re = 0;
            current.im = 0;

            // Reset counter of missed samples
            miss_cnt = 0;

            // Reset update pending variable
            update_pending = false;

            // Start emitting interrupts
            sei();
        }
    }
}

// Interrupt Service Routine for sampling request generation
ISR(RTC_PIT_vect) {
    // In order to check if samples were missed:
    if (update_pending == 1)
        miss_cnt = miss_cnt + 1;
    // Request sample
    update_pending = true;
    // Set periodic timer interrupt again (It's automatically disabled after triggered)
    RTC.PITINTFLAGS = RTC_PI_bm;
}

#pragma clang diagnostic pop