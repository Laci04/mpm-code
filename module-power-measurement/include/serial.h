//
// Created by dominik on 1/23/23.
//

#ifndef WS_PARKING_CODE_SERIAL_H
#define WS_PARKING_CODE_SERIAL_H

#include "usart.h"
#include <stdio.h>

namespace io {
    io::usart<io::usart_mode_t::RS485, 0> usart_(9600);

    int usart0_print(char character, FILE *stream) {
        while(!usart_.tx_ready()) {;}
        usart_.write(character);
        return 0;
    }

    FILE USART_stream = FDEV_SETUP_STREAM(usart0_print, NULL, _FDEV_SETUP_WRITE);
};

#endif //WS_PARKING_CODE_SERIAL_H
