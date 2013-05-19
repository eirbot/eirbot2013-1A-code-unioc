 sudo avrdude -e -p m128 -P /dev/ttyUSB0  -c jtag1 -U flash:w:main.hex -U eeprom:w:main.eep
