#include "io.h"
#include <avr/delay.h>

constexpr int OUTPUT = 1, INPUT = 0;


io::pin_t<10, io::type_::DIGITAL> led (OUTPUT);
int main() {
// write your code here
  PORTMUX.EVSYSROUTEA |= 0b0000'0010;
  PORTMUX.USARTROUTEA |= PORTMUX_USART0_enum::PORTMUX_USART0_ALT1_gc;
  auto led_ = 1;
  while(true) {
    _delay_ms(1000);
    led_ = !led_;
    led = led_;
  }
  return 1;
}