NRF54L15-QFAA-R7 with the internal SMPS and a 2.4 GHz antenna, 20ppm 32.768 crystal
P0.00 RST_N
P0.01 SRButton (active low)
P0.04 HDCINT
P1.03 BUSY_N
P1.04 LoadSwitch (of display)
P1.06 OPTINT
P1.07 BLEButton (active low)
P1.10 I2C_SDA
P1.12 I2C_SCL
P2.00 SPI_DC
P2.01 SPI_SCK
P2.02 SPI_MOSI
P2.05 SPI_CS
P2.07 SWO

HDC3022 (does not need pullup on interrupt pin), 0x45

OPT3005 (needs pullup on the interrupt pin), 0x44

Use internal I2C pullup resistors.

E2206KS0E1 SPI e-ink display (drivers in \external but in C++)