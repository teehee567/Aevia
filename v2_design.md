
goal is to make a garmin catalyst2 but cheaper + better. Custom pcb better performance custom softare, still exporting to proper desktop tooling.

build pcb with this layout:
ESP+Power | Digital+STM+RAM | GNSS+IMU
left to right, have a cm or 2 between digital sensitive to reduce signal interference


check this pcb to see if bga is possible
https://www.st.com/en/evaluation-tools/stm32h7s78-dk.html#cad-resources

considering STM32H7S7I8T6 for much better performance

v2 pcb
- Needs to support
    - move to stm32h7: STM32H7S7L8H6H (TFBGA225)
        - much faster to drive slint
        - on board nand flash
        - clocks:
            - HSE: NDK NX2016SA-24MHZ-EXS00A-CS10820
            - LSE: NDK NX2012SA-32.768KHZ-EXS00A-MU00527
    - flash:
        - MX66UW1G45GXDI00 128MB
    - extra ram for stm32h7
        - AP Memory APS256XXN-OBR-BG still need to pick speed grade to match XSPI clock
    - much faster batteyr charging 1C atleast
        - must have usb pd 
        - AP33772S pd controller, i2c
        - get 21700 form factor maybe do 1c charging
        - BQ25895 for charging
        - MAX17260 to guage battery %
        - bucks/boosts off SYS rail
            - TPS63802: 3.3V @ 2A buck-boost, clean rail (STM32H7, eMMC VCC, UM980 LDO, IMU, touch, peripherals)
            - TPS62A02: 3.3V @ 2A buck, RGB LEDs
            - TPS63802: 3.3V @ 1A buck-boost, dedicated for ESP32-C6
            - TPS62A02: 1.8V @ 2A buck (PSRAM, eMMC VCCQ, VDDIO2/OCTOSPI)
            - 5V boost for buzzer (optional, TBD)
            - LED boost driver for display backlight (TBD, pick after display)
        - ldos:
            - gnss LT3045EDD#TRPBF
            - imu TI TPS7A02
    - TDK IIM-42652 imu
    - USB C protection - TI TPD8S300A
    - leds:
        - for on board small dev indicators, Lite-On LTST-C190
        - APA102-2020-256-8 for brigther led
    - 5 inch 1000nit display maybe touchscreen?
        - use panelook.com
        - [search](https://www.panelook.com/modelsearch.php?op=advancedsearch&order=panel_id&inch_low=490&inch_high=680&signal_type_category=140&brightness_low=11501&sunlight_readable=1)
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


### Power Tree
```mermaid
flowchart TD;
    USB[USBC]
    PD[PD sink<br/>I2C to STM32]
    BQ[buck charger<br/>+ power path]
    BAT[21700 Li-ion]
    FG[fuel gauge]
    SYS((SYS rail<br/>3.5–4.4V))

    BL[LED boost driver<br/>backlight]
    B33[Buck-boost -> 3.3V @ 2A<br/>clean rail]
    B33L[Buck -> 3.3V @ 2A<br/>RGB LEDs<br/>may sag at low batt]
    BESP[Buck-boost -> 3.3V @ 1A<br/>ESP32-C6 dedicated]
    B18[Buck -> 1.8V @ 2A]
    B5[Boost -> 5V<br/>optional, for buzzer]

    L33[STM32H743 VDD/VDDA<br/>eMMC VCC<br/>random peripherals]
    LLED[RGB LEDs]
    LESP[ESP32-C6]
    L18[RAM<br/>eMMC VCCQ<br/>STM32H7 VDDIO2 / OCTOSPI]
    CORE[STM32H7 1.2V core<br/>via internal LDO]

    USB -->|5–20V pd| PD
    PD -->|VBUS_PD<br/>request 9V default| BQ
    BQ <-->|charge / discharge| BAT
    BAT --- FG
    BQ -->|power-path output| SYS

    SYS --> BL
    SYS --> B33
    SYS --> B33L
    SYS --> BESP
    SYS --> B18
    SYS --> B5

    B33 --> L33
    B33L --> LLED
    BESP --> LESP
    B18 --> L18
    L33 --> CORE

    L33 -->|ldo| imu
    L33 -->|ldo| um980
```