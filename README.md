# open-race
gnss module collecting track data

goal is to make a garmin catalyst/racebox better diy. Custom pcb better performance.

v2 pcb
- move to stm32h7: STM32H743IIK6
- Needs to support
  - much faster to drive slint
  - on board nand flash
    - emmc ram: Samsung KLMAG1JETD-B041 (16GB)
  - extra ram for stm32h7
    - APS6404L-3SQR-SN
  - much faster batteyr charging 2000ma
    - must have usb pd 
    - get 21700 form factor maybe do 1c charging
  - maybe micro sd card slot?? maybe not
  - bluetooth to phone
  - 5 inch 1000nit display maybe touchscreen?
  - extra buttons
  - extra leds
  - unicore um980 module
    - 20hz multi constellation spp fixes
    - 50hz rtk based fixes using 1hz spp. Rtk hard to work with? on standalone pcb not worth, probably need phone connection
    - maybe best in price range m10s and others dont have 20hz multi constellation, only single constellation, 

v1 pcb

![image](./data/IMG_4424.jpg)
![image](./data/Screenshot%202026-04-05%20222620.png)

# todo
- when writing software
    - make battery charging 0ma
    - so no dual charger conflict

# oopsies
- On v1 safeboot is pull uped with reset on same resistor, not necessary this causes mdoule to boot in safeboot mode and breaks everything.
