# open-race
gnss module collecting track data

goal is to make a garmin catalyst/racebox better diy. Custom pcb better performance.

### Software Features
#### High priority
- [ ] Delta T lap timing, with colour coding
- [ ] some way to prove lap times to 0.001, a way to analyse raw data to prove that it is as good as the hardware can possibly output

#### Mid priority
- [ ] G-G Diagram more research, assume imu output is very noisy
- [ ] auto track detection
  - [ ] custom track learning mode
  - [ ] manual start finish setting
- [ ] smart battery management
- [ ] Theoretical best time
- [ ] Auto start stop recording based on accel
- [ ] Sector splits for tracts that support it
  
#### low priority
- [ ] configurable led bar
- [ ] customizable gauges
- [ ] colour themes

#### stretch
- [ ] can bus / obd data logging for brakes and thottle and stuff
- [ ] Bluetooth over a esp32s3
  - [ ] allows for rtk mode, 50hz cm level accuracy
  - [ ] updates over bluetooth
- [ ] action cam sync

v2 pcb
- move to stm32h7: STM32H743ZIT6
- Needs to support
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
  - buzzer, speaker probably overkill
  - esp32s3 to handle bluetooth and smaller less important things like leds to save stm pins for important stuff
    - ability ot have random bluetooth sensors around the car? maybe not car very noisy
    - use this to run the leds

v1 pcb

![image](./data/IMG_4424.jpg)
![image](./data/Screenshot%202026-04-05%20222620.png)

# todo
- when writing software
    - make battery charging 0ma
    - so no dual charger conflict

# oopsies
- On v1 safeboot is pull uped with reset on same resistor, not necessary this causes mdoule to boot in safeboot mode and breaks everything.
