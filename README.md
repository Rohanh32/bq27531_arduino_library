# bq27531 Arduino Library
Texas instrument BQ27531 fual gauge arduino library.

Texas Instrument's [BQ27531-G1](https://www.ti.com/product/BQ27531-G1) is a self-calibrating, I2C-based fuel gauge -- it measures your battery’s voltage to estimate its charge percentage and remaining capacity. The chip is also hooked up to a current-sensing resistor, which allows it to measure current and power.

This Arduino library abstracts away all of the low-level I2C communication, so you can easily initialize the fuel gauge then read voltage, state-of-charge, current, power, and capacity. It also implements all of the chip's low-battery, and SoC-change alerts on the GPOUT pin.

Repository Contents
-------------------

* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE. 
* **library.properties** - General library properties for the Arduino package manager. 

License Information
-------------------

This product is _**open source**_! 

If you have any questions or concerns, please contact rohanh32@gmail.com.

Distributed as-is; no warranty is given.
