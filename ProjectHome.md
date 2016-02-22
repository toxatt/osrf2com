This project is simple receiver of radio sensors by Oregon Scientific (OS).
I.e. it is re-translator of RF packets to UART port.
Currently it works with OS v1.0, v2.1, v3.0 protocols. I tested it with some sensors but not with all.
The project is build for ATMEL AVR ATmega8 MCU at 16MHZ in IAR5.30 compiler.
I used RLP434A 433.92MHz receiver and connected its digital out pin to Input Capture Unit of Atmega8 MCU. UART TX pin is translated to RS-232 levels by MAX-232 IC. So schematics is relatively simple. See my website http://alyer.frihost.net or contact me for additional information.