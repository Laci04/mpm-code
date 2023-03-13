//
// Created by dominik on 1/12/23.
//

#ifndef WS_PARKING_CODE_USART_H
#define WS_PARKING_CODE_USART_H

#include "device.h"
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 20000000ull

#define USART0_BAUD_RATE(BAUD_RATE) ((20000000ull * 64.0f / (16.0f * (float)BAUD_RATE))+0.5)

namespace io {
    enum class usart_mode_t {
        FULL_DUPLEX,
        HALF_DUPLEX,
        RS485
    };

    namespace {
        //struct shared_data {
            //volatile size_t n_tx_;
            //volatile size_t i_tx_;
            //volatile uint8_t *buff_tx_;
        //} usart_data[2];
    }

    template<usart_mode_t Mode, size_t NUsart>
    class usart {
    public:
        usart(uint32_t baud);

        void init(uint32_t baud);

        void detect_baud();

        void write(uint8_t *buff, size_t n);

        void write(uint8_t *buff);

        void write(uint8_t c);

        uint8_t read(char *buff, size_t n, char term = '\0');

        uint8_t read();

        const register8_t &receive_errors = receive_errors_;

        inline bool tx_ready() {
            return (usart_.STATUS & USART_DREIF_bm);
        }

        inline bool rx_ready() {
            return (USART0.STATUS & USART_RXCIF_bm);
        }

        inline bool tx_busy() {
            return (!(USART0.STATUS & USART_TXCIF_bm));
        }

        inline bool rx_done() {
            return (USART0.STATUS & USART_TXCIF_bm);
        }

    private:
        PORT_t& port_ = (NUsart == 0)? PORTB : PORTB;
        USART_t& usart_ = (NUsart == 0)? USART0 : USART1;
        uint32_t baud_;
        //uint8_t pin_arr[3];

        //shared_data &data_ = usart_data[NUsart];
        register8_t &receive_errors_ = usart_.RXDATAH;
    };

    template<usart_mode_t Mode, size_t NUsart>
    usart<Mode, NUsart>::usart(uint32_t baud) {
        if constexpr (NUsart == 0) {
            //usart_ = &USART0;
            //port_ = PORTB;
        } else {
            //usart_ = &USART1;
            //port_ = PORTA;
            //PORTMUX.USARTROUTEA |= PORTMUX_USART00_bm;
        }
        this->init(baud);
    }

    template<usart_mode_t Mode, size_t NUsart>
    void usart<Mode, NUsart>::init(uint32_t baud) {
        cli();
        if constexpr (Mode == usart_mode_t::RS485) {//USART im ISO 9600 8N1 Standard (Serial kompatibel)
            //RS485 config
            usart_.BAUD = (uint16_t) USART0_BAUD_RATE(baud);
            usart_.CTRLA |= (USART_RS485_bm); // | USART_RXCIE_bm | USART_TXCIE_bm | USART_DREIE_bm); //Enable rs485 and interrupts
            usart_.CTRLB |= (USART_RXMODE_NORMAL_gc); //NORMAL mode 16 samples per bit
            //9600 8N1 mode
            usart_.CTRLC |= (USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc |
                                    USART_SBMODE_1BIT_gc);

            port_.DIRSET |= (1 << 0); //DIR output
            port_.DIRSET |= (1 << 2); //TxD output
            port_.DIRCLR = (1 << 3); //RxD input

            //DBGCTRL_DBGRUN
            usart_.DBGCTRL = 0x0;
            //EVCTRL_IREI
            usart_.EVCTRL = 0x0;
            //RXPLCTRL_RXPL
            usart_.RXPLCTRL = 0x0;
            //TXPLCTRL_TXPL
            usart_.TXPLCTRL = 0x0;
        } else if (Mode == usart_mode_t::HALF_DUPLEX) {
						//Modus nnicht implementiert
        }
        sei();
    }

    template<usart_mode_t Mode, size_t NUsart>
    void usart<Mode, NUsart>::detect_baud() {
        usart_.STATUS |= USART_WFB_bm;
    }

    void process_tx(size_t usart_n);

    template<usart_mode_t Mode, size_t NUsart>
    void usart<Mode, NUsart>::write(uint8_t c) {
        usart_.CTRLB |= USART_TXEN_bm;

        if constexpr(Mode == usart_mode_t::RS485 || Mode == usart_mode_t::HALF_DUPLEX)
            usart_.CTRLB &= ~USART_RXEN_bm;

        while (!tx_ready()) { ; }

        usart_.TXDATAL = c;
    }

    template<usart_mode_t Mode, size_t NUsart>
    void usart<Mode, NUsart>::write(uint8_t *buff, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            write(buff[i]);
        }
    }

    template<usart_mode_t Mode, size_t NUsart>
    void usart<Mode, NUsart>::write(uint8_t *buff) {
        size_t i = 0;
        while (buff[i] != '\0' && i < 200) {
            write(buff[i]);
            i++;
        }
    }

    template<usart_mode_t Mode, size_t NUsart>
    uint8_t usart<Mode, NUsart>::read() {
        size_t n_cycles = 0;

				usart_.CTRLB |= USART_RXEN_bm;

        if constexpr(Mode == usart_mode_t::RS485 || Mode == usart_mode_t::HALF_DUPLEX)
            usart_.CTRLB &= ~USART_TXEN_bm;

        while(!rx_ready() && n_cycles < 1000) { n_cycles++; }

        return usart_.RXDATAL;
    }

    template<usart_mode_t Mode, size_t NUsart>
    uint8_t usart<Mode, NUsart>::read(char *buff, size_t n, char term) {
        uint8_t i = 0;
        char c;

        while(i < n && buff[i] != term) {
            buff[i] = read();
            i++;
        }
        if(i < n - 1)
            buff[i] = '\0';

        return i + 1;
    }
}

#endif  // WS_PARKING_CODE_USART_H
