#include "io.h"
#include <spi.h>
#include <avr/delay.h>



constexpr int OUTPUT = 1, INPUT = 0;

io::pin_t<10, io::type_::DIGITAL> led (OUTPUT);     //Anstatt pinmode


int main() {
    led = 1;
    while(true) {
        led = !led();
        _delay_ms(250);
    }

    io::spi<io::dir_t::OUTPUT>::mode_t::BUFFERED;
}