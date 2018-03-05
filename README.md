# edcstaff
A staff that finds its friend

Description : Walking staff that reacts to music and can locate a like staff

Sources
  - uses Radiohead https://github.com/PaulStoffregen/RadioHead
  - uses Adafruit_LiquidCrystal https://github.com/adafruit/Adafruit_LiquidCrystal

    
  Target
    - Works with all Arduino boards, uses hardware SPI port and I2C pins which are board dependent
    - Overview of Adafruit RFM69HCW (SPI) at: https://www.adafruit.com/product/3070
    - Overview of Adafruit i2c/SPI LCD Backpack at: https://learn.adafruit.com/i2c-spi-lcd-backpack/
    - Overview of 16x2 LCD panels available at: http://oomlout.com/parts/LCDD-01-guide.pdf
    - Overview of Adafruit Utimate GPS at: https://www.adafruit.com/product/746

    
  Revisions
    030418
    	- first version
    		adapted from Tiny GPS sample code and receiver project
    		currently relies on transmitter project
    		many problems with this code; it runs only briefly


  Feature Requests
    030318
      - comprehensive error handling
      - unique identier for each gps device?
      - reliable datagram
      - memory use close to limit for ATmega328P, will need to migrate platform soon
