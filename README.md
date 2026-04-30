# open-race
gnss module collecting track data

Dont really actually recommend someone make this, parts need to be soldered using a reflow oven/pad/station, good amount of money in tooling, 

Why is this necessary? because there is performance left on the table with better components.

goal is to make a garmin catalyst2 but cheaper + better. Custom pcb better performance custom softare, still exporting to proper desktop tooling.



### Software Features
#### High priority
- [ ] Delta T lap timing, with colour coding
- [ ] some way to prove lap times to 0.001, a way to analyse raw data to prove that it is as good as the hardware can possibly output
- [ ] export into harrys laptimer or something that takes in data.

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
  - Antenna
    - Beitian BT-T076
  - buzzer, speaker probably overkill
  - esp32s3 to handle bluetooth and smaller less important things like leds to save stm pins for important stuff
    - ability ot have random bluetooth sensors around the car? maybe not car very noisy
    - use this to run the leds


#### GPS Specs
UM980 is the best module for this purpose.

The UM980 does this at 20hz in full multi gnss mode, and there is currently nothing online that can beat it for raw gps quality at this price point.

GPS antenna sits inside the windshield, and the metal roof of the car blocks basically everything coming from behind it. That kills half of the view of the sky, leads to poor quality and lower actual refresh time. A fix is using multi gnss where the module combines data from individual satellites across GPS + GLONASS + Galileo + BeiDou into a single location fix. More satellites in view = more accuracy, and with four constellations at the same time you have way more sats in view. note that even if a module says multi gnss doesnt mean that it can do multi at max hz

UM980 also supports multi band: a single band module sees one frequency per satellite, so a signal bouncing off the car body or nearby barriers looks identical to a direct one. With L1 + L2/L5 the receiver can compare data from a single satellite to throw away bad data. because different bands bounce differently.

Antennas: most off the shelf trackers use a ceramic patch. patch gain points straight up out of the flat face and drops off fast towards the horizon, so it only works well laying flat. The BT-T076 is a helical lower peak but way better coverage when tilted and at low elevation. picks up the horizon sats a tilted patch would miss + most patch antennas are 1 band only (l1).

*Disclaimer* honestly data being better isnt an insane performance increase and off the shelf products do their job well. Nothing is gonna beat some sort of transponder in the track for lap timing. + software matters a lot

How that compares to off the shelf products:
- Dragy Pro (ublox neo m10s): 10hz multi gnss, 20hz single constellation, L1 only

Above are the only products i could find the actual module of, however i *highly* doubt that any of the products that claim 25hz + multi gnss + 10cm accuracy are actually 25hz + multi gnss + 10cm at the *same* time, simply because i cant find a single reference to a module that can actually output that. [All my gps research](./data/gps_module_research.md)

GNSS modules considered:
- **Unicore UM980**
  - L1 + L2/L5 + L6 multi band, multi constellation, full RTK
  - 20hz output in full multi gnss mode, ~50hz on rtk mode (not sure this a good idea to use)
  - ~$130 aud
- **ublox NEO-M9N**
  - L1 only, multi constellation
  - 25hz single constellation, 10hz multi
  - ~$20 aud
- **ublox NEO-M10S**
  - L1 only, multi constellation 
  - 25hz single constellation, 10hz multi
  - ~$20 aud
- **ublox ZED-F9P**
  - L1 + L2 dual band, multi constellation, full RTK
  - capped at 8hz with everything on, 20hz only with reduced config
  - ~$130 aud
- **Quectel LC29H (DA/EA)**
  - dual band L1 + L5, multi constellation, RTK on the EA variant
  - 10hz max
- **Septentrio mosaic-X5**
  - all bands, all constellations, VERY GOOD RTK, 100hz capable
  - the actual best raw quality
  - *$1k+ aud*
- **Skytraq PX1122R**
  - L1/L2 multi-band RTK, multi-constellation
  - 20hz max?
  - docs/pricing/everything very hard to find

v1 pcb

![image](./data/IMG_4424.jpg)
![image](./data/Screenshot%202026-04-05%20222620.png)

# todo
- when writing software
    - make battery charging 0ma
    - so no dual charger conflict

# oopsies
- On v1 safeboot is pull uped with reset on same resistor, not necessary this causes mdoule to boot in safeboot mode and breaks everything.
