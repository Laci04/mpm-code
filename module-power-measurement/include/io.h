//
// Created by dominik on 1/21/23.
//

#ifndef WS_PARKING_CODE_IO_H
#define WS_PARKING_CODE_IO_H

#include <avr/io.h>
#include <inttypes.h>
#include <stddef.h>
#include <math.h>

//Es wurden sehr C++ referenzen und möglichst wenig neue variablen verwendet um Flash und Ram nicht zu belegen
//pins gehen von 0 bis n, wobei sie nach den PORTS gereiht sind dh. pins 0 - 7 auf portA und 8 - 15 auf portB usw.
namespace io {
		enum class type_ {
				ANALOG,
				DIGITAL
		};
#ifndef F_CPU
		#define F_CPU 20000000UL
#endif
#define TIMEBASE_VALUE (uint8_t)ceil(F_CPU*0.000001)

		template<size_t N, type_ Type>
		class pin_t {
			public:
				explicit pin_t (int dir, int ctrl = PORT_PULLUPEN_bm) :dir_{dir} {
						if(&port_ == &PORTA && pin_ == 0)
								*const_cast<uint8_t*>(&pin_) = 1;
						        if constexpr (Type == type_::DIGITAL) {		//DIGITAL pin Einstellungen
								*( &( port_.PIN0CTRL ) + pin_ ) = ctrl;
								port_.DIRCLR |= ( 1 << pin_ );				//Wenn richtungs bit vorhanden, löschen
								port_.DIRSET |= ( dir << pin_ );			//Setzen
						} else {
							if(dir_ == 0) {		//ADC einstellungen
									*( &( port_.PIN0CTRL ) + pin_ ) |= PORT_ISC_INPUT_DISABLE_gc;
									ADC0.CTRLA |= ADC_ENABLE_bm;
									ADC0.CTRLB = ADC_PRESC_DIV4_gc;
									ADC0.CTRLC = ADC_REFSEL_VDD_gc | TIMEBASE_VALUE << ADC_TIMEBASE0_bp;/*ADC reference VDD*/
									ADC0.CTRLE = 3;	/*SAMPDUR*/
									ADC0.CTRLF = ADC_SAMPNUM_ACC4_gc | ADC_FREERUN_bm; //nbo need to start			/*Single*/
									ADC0.MUXPOS = pin_;////ADC_MUXPOS_AIN1_gc +  ;		/*ADC channel AIN5->PA5*/
									ADC0.COMMAND = ADC_MODE_BURST_gc; /*Burst*/
							} else {

							}
						}
				}
				template<class T>
				inline pin_t& operator=(T state) {
						if constexpr (Type == type_::DIGITAL) {
								port_.OUTCLR |= ( 1 << pin_ );
								port_.OUTSET |= ( state << pin_ );
						} else {

						}
						return (*this);
				}

				inline uint16_t operator()(){
						if constexpr (Type == type_::DIGITAL) {
								return (( port_.IN & ( 1 << pin_ ) ) >> pin_);
						} else {
								return ADC0.RESULT/4;
						}
				}

				bool operator==(pin_t& other) {
						if constexpr (Type == type_::DIGITAL) {
								return ( ( ( port_.IN & ( 1 << pin_ ) ) >> pin_ ) == other());
						} else {
								return (ADC0.RESULT/4) == other();
						}
				}

				template<class T>
				inline bool operator==(T state) {
						if constexpr (Type == type_::DIGITAL) {
								//auto reg1 = (dir_ == 1)? port_.OUT : port_.IN;
								return  ( ( port_.IN & ( 1 << pin_ ) ) >> pin_ ) == state;
						} else {
								return (ADC0.RESULT/4) == state;
						}
				}

				inline bool operator!() {
						if constexpr (Type == type_::DIGITAL) {
								return !( ( port_.IN & ( 1 << pin_ ) ) >> pin_ );
						} else {
								return !(ADC0.RESULT/4);
						}
				}

				inline bool adc_ready() {
						return (ADC0.INTFLAGS & ADC_RESRDY_bm);
				}

				inline bool adc_start() {
						ADC0.COMMAND |= ADC_START_enum::ADC_START_IMMEDIATE_gc; //start free running
				}

				inline bool adc_stop() {
						ADC0.COMMAND |= ADC_START_enum::ADC_START_STOP_gc; //start free running
				}

			private:
				PORT_t& port_ = ( N < 8 ) ? PORTA : PORTB;
				const uint8_t pin_ = (N % 8);
				uint8_t dir_;
		};
};

#endif  // WS_PARKING_CODE_IO_H