# FLC-01
## Fitolamp controller ver.3

![board img](https://habrastorage.org/webt/fs/mx/rk/fsmxrkeqmmq0b7pfv6oscsz74zm.jpeg "Board view")

### Characteristics
FLC is the board that supports 3 independent output channels for 1W\3W\5W-LED driving.  
Board operates __Vin__ = 12..50 V.  
Power consumption depends on number and type of connected LEDs.

Each channel is based on special __LDD-xxH__ driver and can:
* provide __Vout__ = 10..48 V (depending on Vin value);  
* generate stabialized __Iout__ = 350..1000 mA (depending on choosen [LDD-xxH series](https://static.chipdip.ru/lib/428/DOC014428669.pdf);
* regulate output power (LED brightness) from 0 to 100% with step of 0.1% by software-control

There is an DIP-panel option on the board, so __every driver is changeable__ and can be selected for specific project.

_BOM_ and _Gerber_ files can be found at `./pcb/ver03/Project Outputs for FitoLamp_Project`

### Features
FLC board is based on STM32F103 and has embedded RTC timer. It has battery-powered information about current time and can proceed schedules. For output power managements it uses PWM signals. So, software-controllable functions of the board are:

* __power-on\off schedules__: you can set _Start time_ and _Stop time_ when light from LEDs is needed to be ON and OFF.
* __channels power control__: as mentioned above, it is possible to regulate independently each channel.  
* __smooth power-on\off__: last but not least, smooth mode that enables light spreading step-by-step.

For example, if you use _red_, _blue_ and _white_ LEDs lines as FitoLight setup, __it is possible to configure red\blue balance__.   
In other words, __you can adjust lamp stectrum depending on plants type you grow and it's vegetative stage__.

![red blue balance](https://habrastorage.org/webt/oo/7-/by/oo7-by4jglop3etsxfewpemuhxo.jpeg "red blue balance")

### Interfaces
The board has 3 interfaces for external control :
* Bluetooth 4 (BLE) 
* UART (3.3V)
* RS232 (12V)

Repository contains a simple Python3 GUI for serial connection in `./gui/flc_gui.pyw`.   
It is based on _tkinter_ module and can run both on Windows and Linux.

![flc gui](https://hsto.org/r/w1560/webt/ny/rl/u8/nyrlu8balijdhxun5cinq09puge.jpeg "FLC GUI")

For bluetooth connection [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&hl=ru&gl=US) application can be used.

### Communication Protocol
On all supported interfaces FLC uses same _JSON_ format as main communication protocol. So it should be easy to intergate it in other systems if needed. 

![flc protocol](https://hsto.org/webt/h-/iw/i1/h-iwi1sm7udujb5ivhnds8pivdi.jpeg "FLC Protocol")


### Connection
__No external resistors for LEDs current needed__. Connect LED curcuit to output channels directly.  

There are some options for input and output connectors on the board footprint.

DC-Input connectors could be one of:  

* DS-201 2,1х5,5
* 300-021-12

Output channels connectors could be one of: 

* 1x CWF-6R (DS1069-6 MR) - cable part then is CHU-6 (DS1069-6 F)
* 3x 300-021-12

RS232 interface has standart DB9 connector.  

![board connection](https://habrastorage.org/webt/64/dp/u9/64dpu97lcywteeefegrs-xds7zo.jpeg "FLC Connection")
