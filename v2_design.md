
goal is to make a garmin catalyst2 but cheaper + better. Custom pcb better performance custom softare, still exporting to proper desktop tooling.

build pcb with this layout:
ESP+Power | Digital+STM+RAM | GNSS+IMU
left to right, have a cm or 2 between digital sensitive to reduce signal interference

v2 pcb
- Needs to support
    - move to stm32h7: STM32H743ZIT6
        - much faster to drive slint
        - on board nand flash
    - emmc flash: Samsung KLMAG1JETD-B041 (16GB)
    - extra ram for stm32h7
        - APS6408L-3OBM-BA
    - much faster batteyr charging 1C atleast
        - must have usb pd 
        - get 21700 form factor maybe do 1c charging
        - BQ25895 for charging
        - MAX17260 to guage battery %
    - bluetooth to phone
    - 5 inch 1000nit display maybe touchscreen?
    - extra buttons
        - something super high quality and tactile?
    - extra leds
        - rgb smd
    - unicore um980 module
        - 20hz multi constellation spp fixes
        - 50hz rtk based fixes using 1hz spp. Rtk hard to work with? on standalone pcb not worth, probably need phone connection
        - maybe best in price range m10s and others dont have 20hz multi constellation, only single constellation, 
    - Antenna
        - Beitian BT-T076
    - buzzer, speaker probably overkill
    - esp32-c6 to handle bluetooth/wifi and smaller less important things like leds to save stm pins for important stuff
        - stm32-c6-mini-1
        - ability ot have random bluetooth sensors around the car? maybe not car very noisy
        - use this to run the leds
        - flashing over uart
