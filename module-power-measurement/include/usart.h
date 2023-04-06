//
// Created by dominik on 1/12/23.
//

#ifndef WS_PARKING_CODE_USART_H
#define WS_PARKING_CODE_USART_H

#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 20000000ull

#define USART0_BAUD_RATE(BAUD_RATE) ((20000000ull * 64.0f / (16.0f * (float)BAUD_RATE))+0.5)


namespace io {
		constexpr uint8_t SERIAL8N1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_SBMODE_1BIT_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc);

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

        void write(const char *buff);

        void write(char c);

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
        PORT_t& port_ = (NUsart == 0)? PORTB : PORTA;
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
				uint8_t oldSREG = SREG;
        cli();
				usart_.CTRLB = 0;
        if constexpr (Mode == usart_mode_t::RS485) {//USART im ISO 9600 8N1 Standard (Serial kompatibel)
            //RS485 config
            usart_.BAUD = (uint16_t) USART0_BAUD_RATE(baud);
            usart_.CTRLA |= (USART_RS485_bm); // | USART_RXCIE_bm | USART_TXCIE_bm | USART_DREIE_bm); //Enable rs485 and interrupts
            usart_.CTRLB |= (USART_RXMODE_NORMAL_gc); //NORMAL mode 16 samples per bit
            //9600 8N1 mode
            usart_.CTRLC = SERIAL8N1;

            port_.DIRSET |= (1 << 0); //DIR output
            port_.DIRSET |= (1 << 2); //TxD output
            port_.DIRCLR = (1 << 3); //RxD input
        } else if (Mode == usart_mode_t::HALF_DUPLEX) {
						//Modus nnicht implementiert
            usart_.CTRLA |= USART_LBME_bm;
            port_.PIN2CTRL |= PORT_PULLUPEN_bm;
            usart_.CTRLC = SERIAL8N1;
            usart_.CTRLB = ((USART_RXMODE_NORMAL_gc) | USART_ODME_bm | USART_TXEN_bm | USART_RXEN_bm);

        } else {
						// Make sure no transmissions are ongoing and USART is disabled in case begin() is called by accident
						// without first calling end()

						uint8_t ctrlc = SERIAL8N1;            // low byte of 0 could mean they want SERIAL_5N1. Or that they thought they'd

						ctrlc &= ~0x04; // Now unset that 0x04 bit if it's set, because none of the values with it set are supported. We use that to smuggle in a "this constant was specified" for 5N1
						uint8_t ctrla = 0;// CTRLA will get the remains of the options high byte.
						uint16_t baud_setting = 0;                // at this point it should be able to reuse those 2 registers that it received options in!
						uint8_t   ctrlb = (~ctrla & 0xC0);        // Top two bits (TXEN RXEN), inverted so they match he sense in the registers.
						if (baud   > F_CPU / 16) {            // if this baud is too fast for non-U2X
								ctrlb   |= USART_RXMODE0_bm;        // set the U2X bit in what will become CTRLB
								baud   >>= 1;                       // And lower the baud rate by haldf
						}
						baud_setting = (((4 * F_CPU) / baud));  // And now the registers that baud was passed in are done.
						if (baud_setting < 64)                      // so set to the maximum baud rate setting.
								baud_setting= 64;       // set the U2X bit in what will become CTRLB
						//} else if (baud < (F_CPU / 16800)) {      // Baud rate is too low
						//  baud_setting = 65535;                   // minimum baud rate.'
						// Baud setting done now we do the other options not in CTRLC;
						if (ctrla & 0x04) {                       // is ODME option set?
								ctrlb |= USART_ODME_bm;                 // set the bit in what will become CTRLB
						}
						// Baud setting done now we do the other options.
						// that aren't in CTRLC;
						ctrla &= 0x2B;                            // Only LBME and RS485 (both of them); will get written to CTRLA, but we leave the event bit.
						if (ctrlb & USART_RXEN_bm) {              // if RX is to be enabled
								ctrla  |= USART_RXCIE_bm;               // we will want to enable the ISR.
						}
						uint8_t setpinmask = ctrlb & 0xC8;        // ODME in bit 3, TX and RX enabled in bit 6, 7
						if ((ctrla & USART_LBME_bm) && (setpinmask == 0xC8)) { // if it's open-drain and loopback, need to set state bit 2.
								//_state                 |= 2;            // since that changes some behavior (RXC disabled while sending) // Now we should be able to ST _state.
								setpinmask             |= 0x10;         // this tells _set_pins not to disturb the configuration on the RX pin.
						}
						if (ctrla & USART_RS485_bm) {             // RS485 mode recorded here too... because we need to set
								setpinmask             |= 0x01;         // set pin output if we need to do that. Datasheet isn't clear
						}
						uint8_t oldSREG = SREG;
						cli();
						usart_.CTRLB          = 0;            // gotta disable first - some things are enable-locked.
						usart_.CTRLC          = ctrlc;        // No reason not to set first.
						usart_.BAUD           = baud_setting; // Wish I could have set it long ago
						if (ctrla & 0x20) {                       // Now we have to do a bit of work
								setpinmask             &= 0x7F;         // Remove the RX pin in this case because we get the input from elsewhere.
								usart_.EVCTRL       = 1;            // enable event input - not clear from datasheet what's needed to
								usart_.TXPLCTRL     = 0xFF;         // Disable pulse length encoding.
						} else {
								usart_.EVCTRL       = 0;            // This needs to be turned off when not in use.
						}                                         // finally strip out the SERIAL_EVENT_RX bit which is in the DREIE
						usart_.CTRLA          = ctrla & 0xDF; // position, which we never set in begin.
						usart_.CTRLB          = ctrlb;        // Set the all important CTRLB...
						//_set_pins(_module_number, _pin_set, setpinmask); // set up the pin(s)
						port_.DIRSET |= (1 << 2); //TxD output
						port_.DIRCLR = (1 << 3); //RxD input
						SREG = oldSREG;                             // re-enable interrupts, and we're done.
        }
    }

    template<usart_mode_t Mode, size_t NUsart>
    void usart<Mode, NUsart>::detect_baud() {
        usart_.STATUS |= USART_WFB_bm;
    }

    void process_tx(size_t usart_n);

    template<usart_mode_t Mode, size_t NUsart>
    void usart<Mode, NUsart>::write(char c) {
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
    void usart<Mode, NUsart>::write(const char *buff) {
				if (buff == nullptr) {
						return;
				}
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
