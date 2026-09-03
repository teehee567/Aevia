# v2_mini reflow and double-sided assembly audit

Date checked: 2026-08-29

Scope: the fitted items in [`production/digikey_bom.csv`](production/digikey_bom.csv) and their placement in [`v2_mini.kicad_pcb`](v2_mini.kicad_pcb). DNP capacitors C12/C24/C25/C26, mounting holes, bare test pads, and the no-fit Tag-Connect footprint are excluded.

## Short answer

Every component actually fitted by this BOM is an SMT/reflow-capable component. There is no fitted battery, battery holder, buzzer, through-hole-only switch, or other BOM item that is categorically forbidden from reflow.

The board is nevertheless **not compatible with an ordinary two-pass, same-alloy double-sided reflow process as presently placed**:

- U1 UM980 is on the front at `(105.12, 95.32)` and U2 ESP32-S31-WROOM-3 is on the back at `(97.67, 95.50)`. Their component outlines substantially overlap.
- Unicore says not to put the UM980 on the underside during reflow and does not recommend putting it through a soldering cycle twice.
- Espressif says to solder the WROOM module in a **single reflow**.
- Consequently, front-first makes the UM980 face downward and encounter a second hot cycle during the back pass; back-first gives the WROOM a second hot cycle during the front pass. Surface tension may hold a particular unit, but neither ordering follows both manufacturers' instructions. The UM980 manual's explicit warning is stronger evidence than a weight/contact-count estimate.

The robust production fix is to put both modules on one side or obtain a reviewed selective-reflow process from the module manufacturers/PCBA vendor. A handheld hot-air gun is particularly risky here because its airflow can move an underside component and because the two modules overlap thermally. If a prototype is attempted anyway, use a board preheater, thermocouples on both module bodies/pads, controlled low airflow, and X-ray inspection; treat it as an unqualified rework process, not as a repeatable assembly plan.

## Critical modules

| Ref / manufacturer part | PCB side | Reflow and moisture limits | Assembly decision |
|---|---:|---|---|
| U1 — Unicore **UM980**, 54-LGA, 17 × 22 × 2.6 mm, 1.88 g | Front | Bundled UM980 manual R1.9: ramp ≤3 °C/s; 150–180 °C for 60–120 s; >217 °C for 40–60 s; peak ≤245 °C; cool ≤4 °C/s. It explicitly says not to design/solder the module on the back of the PCB to prevent falling and does not recommend two soldering cycles. It recommends a 0.15 mm stencil. Vacuum packed with desiccant; the public manual does not give a numeric MSL. If baking is required, remove it from carrier packaging first because the packaging is limited to 55 °C, then follow the bag label/IPC handling. | Reflow only in a qualified process. Do not rely on it remaining attached upside-down in the WROOM-side pass. This is the highest mechanical/process risk on the board. Source: [Unicore product page and current manual download](https://en.unicore.com/products/um980/) and [the manufacturer manual bundled with this repo](../../data/datasheets/UM980_User%20Manual_EN_R1.9.pdf). |
| U2 — Espressif **ESP32-S31-WROOM-3-N16R16V**, 99-pad module, 22 × 30 × 3.5 mm | Back | MSL3; mount within 168 h after opening at ≤30 °C/60% RH. Bake if the 10% HIC spot indicates excess humidity or floor life is exceeded; remove from tape and follow J-STD-033. Espressif explicitly says **single reflow**, SAC305, 150–200 °C for 60–120 s, >217 °C for 60–90 s, peak 235–250 °C. Avoid ultrasonic cleaning/welding. | It is reflowable, but not qualified for a second reflow. It is an LGA-style module with many underside/perimeter pads, not something to finish with an iron. Sources: [ESP32-S31-WROOM-3 datasheet, Product Handling](https://documentation.espressif.com/esp32-s31-wroom-3_datasheet_en.html#product-handling) and [Espressif dry-pack rules](https://documentation.espressif.com/esp-packaging/en/latest/esp32s31/index.html). |
| U3 — Murata **SCH16T-K01-1**, 24-pin MEMS | Back | Reflowable SMD; MSL3. Murata says electrical performance is specified after 24 h from reflow and that assembly can affect accuracy. The exact assembly document APP10871 is controlled/restricted; the public datasheet does not publish a complete numeric oven profile. Ultrasonic agitation/cleaning is prohibited. | Reflow in the documented JEDEC/bag-label envelope, keep it dry, and do not use a concentrated hot-air blast or ultrasonic cleaner. Allow 24 h before precision calibration/acceptance testing. Sources: [Murata SCH16T series page](https://www.murata.com/en-us/products/sensor/gyro/overview/lineup/sch16t), [public SCH16T-K01 datasheet](https://www.murata.com/-/media/webrenewal/products/sensor/pdf/datasheet/datasheet-sch16t-k01-short.ashx?cvid=20251211010000000000&la=en-sg), and [bundled full datasheet](../../data/datasheets/sch16t-k01-datasheet-full.pdf). |

## Connectors and switches

| Ref / manufacturer part | Reflow status and limit | What to do |
|---|---|---|
| J1 — Hirose **U.FL-R-SMT-1(01)** | Reflowable. Hirose's U.FL profile gives a 250 °C maximum lead temperature, ≤10 s at peak, 130–180 °C preheat for ≤120 s, and at most two reflows. Manual touch-up is 350 °C for ≤5 s. | Reflow the receptacle. Attach the coax plug/cable only after the board has cooled. Do not use the cable to retain the connector. [Hirose U.FL catalog](https://www.hirose.com/en/product/document?clcode=CL0321-1039-0-15&documentid=ed_U.FL_CAT&documenttype=Catalog&lang=en&productname=UFL-2LPHF6-04N2TV-AC-170&series=U.FL) |
| J2 — Amphenol **GTFP08441BEU** microSD socket | Reflowable SMT, high-temperature thermoplastic. Official specification permits two reflows with peak 260 °C (+0/−10 °C) and 150–200 °C preheat for 60–120 s. | Reflow the empty socket. Insert the microSD card only after cooling/cleaning. [Product page](https://www.amphenol-cs.com/product/gtfp08441beu.html), [product specification](https://cdn.amphenol-cs.com/media/wysiwyg/files/documentation/ps-7500.pdf) |
| J3 — Amphenol **SFV24R-2STE1HLF** 24-way FPC connector | Manufacturer lists solder process **reflow**; the application specification recommends SAC305 and permits up to two reflows. | Reflow the connector with its latch closed and no FPC installed. Connect the display/ribbon only after cooling. It has no pick cap, which matters to machine assembly but does not make it hand-solder-only. [Product page](https://www.amphenol-cs.com/product/sfv24r2ste1hlf.html), [application specification](https://cdn.amphenol-cs.com/media/wysiwyg/files/drawing/bjm-sfv101.pdf) |
| J4 — Hirose **DF58-2P-1.2V(21)** battery connector | Reflowable SMT. 150–180 °C for 90–120 s; >220 °C for ≤60 s; peak 250 °C for ≤10 s. Manual touch-up 350 ±10 °C for ≤3 s. | Reflow the empty board connector. The BOM contains **no battery**; plug the Li-ion cell in only after assembly, inspection, and electrical checks. [Hirose DF58 catalog](https://www.hirose.com/en/product/document?clcode=CL0666-1003-0-21&documentid=en_DF58_CAT&documenttype=Catalog&lang=en&productname=DF58-4P-1.2V%2821%29&series=DF58) |
| J6 — GCT **USB4105-GF-A** USB-C | Designed for SMT contacts with through-hole shell stakes. GCT rates peak solder temperature 255 °C (−0/+5 °C) for ≤5 s. | Reflow the fine contacts. Because this footprint's shell stakes are through-hole, inspect them and hand-solder the stakes after reflow if the prototype stencil/process does not deposit solder there. Do not attempt to replace the hidden SMT-contact reflow with shell-stake solder alone. [GCT USB4105 page/specification](https://gct.co/connector/usb4105) |
| SW1–SW7 — Omron **B3U-1000P** | Reflowable. For B3U: preheat 150–180 °C, >220 °C for ≤60 s, peak 260 °C; at most two solderings, with ≥5 min between cycles. Published curve assumes a 1.6 mm PCB. Manual solder ≤350 °C for ≤3 s. | Reflow is normal. Do not use an automatic solder bath. Do not wash/immerse this non-sealed switch. [Omron switch soldering guidance](https://omronfs.omron.com/en_US/ecb/products/pdf/A306-E1.pdf) |

None of these connectors is a reason to leave the whole connector off until after reflow. The **mating items**—battery, USB cable, microSD card, FPC/display, and U.FL coax cable—must be installed after reflow.

## LED, temperature sensor, ICs, and other semiconductors

| BOM parts | Status |
|---|---|
| D32–D35 — ROHM **MSL0402RGBU1** RGB LEDs | Reflowable but not flow-solder guaranteed. MSL3, 168 h floor life. If expired/HIC indicates moisture, bake 60 ±3 °C for 40–48 h at <10% RH (ROHM recommends only one bake). Maximum two reflows; peak ≤250 °C, peak interval ≤10 s, 230–250 °C interval ≤40 s. The silicone lens is soft: do not press it or aim strong hot-air flow at it. [ROHM product page](https://www.rohm.com/products/led/chip-leds-multi-color-type/msl0402rgbu-product), [bundled manufacturer datasheet](../../data/datasheets/msl0402rgbu1-e.pdf) |
| TH1 — Murata **NCU18XH103F60RB** 0603 NTC | Standard reflowable chip thermistor. Use the NCU mounting profile/current manufacturer data and avoid excess dwell or repeated local heating. No reason to hand-install after reflow. [Murata NCU series](https://www.murata.com/en-us/products/thermistor/ntc/overview/lineup/ncu) |
| D1 **TPD1E0B04DPYR**, U5 **TCA9536ADTMR**, U6 **TPD4EUSB30DQAR**, U10 **TPS63802DLAR** | TI lists these packages as MSL1, 260 °C peak, unlimited floor life. Reflow normally; no bake when stored normally. [TI quality/package lookup](https://www.ti.com/quality-reliability-packaging-download/report?opn=TPD1E0B04DPYR) |
| D2 **ESDS311DYFR** | TI lists MSL3, 260 °C peak, 168 h floor life. Dry-pack handling/bake per bag label and J-STD-033 if expired. [TI quality/package lookup](https://www.ti.com/quality-reliability-packaging-download/report?opn=ESDS311DYFR) |
| U4 **LP5813ADRRR** | TI WSON-12, reflowable; TI lists the family/package as MSL1, 260 °C peak. [TI product/quality page](https://www.ti.com/product/LP5813/part-details/LP5813ADRRR) |
| U8 **BQ25622ERYKR** | TI lists MSL2, 260 °C peak, one-year floor life. Follow dry-pack label if the bag has been open longer. [TI quality/package lookup](https://www.ti.com/quality-reliability-packaging-download/report?opn=BQ25622ERYKR) |
| U7 **MAX16169AALTA+T**, U9 **MAX17048G+T10** | Pb-free leadless surface-mount packages intended for reflow. Follow the exact lot's ADI material declaration/reel MSL label and J-STD-020 profile; neither should be hand-installed after reflow. The BOM datasheets identify 6-LFCSP/µDFN and 8-TDFN-EP packages. [MAX16169 product page](https://www.analog.com/en/products/max16169.html), [MAX17048 product page](https://www.analog.com/en/products/max17048.html), [ADI package/MSL resources](https://www.analog.com/en/resources/packaging-quality-symbols-footprints/package-resources.html) |
| Q1 **NTNS3164NZT5G** | Pb-free SOT-883 intended for reflow; datasheet gives lead solder temperature 260 °C for 10 s. Use the reel label/J-STD-020 for MSL/profile. [onsemi datasheet](https://www.onsemi.com/download/data-sheet/pdf/ntns3164nz-d.pdf), [onsemi soldering and mounting manual](https://www.onsemi.com/pub/collateral/solderrm-d.pdf) |

## Passives

All remaining fitted BOM items are ordinary SMT chip passives and are intended for reflow. They are not candidates for “install after reflow.” This group covers every remaining manufacturer part number:

- MLCCs: Murata **GRM1555C1H101JA01D**, **GRM155R71E473KA88D**, **GRM188R61E475KE11D**, **GRT188R61A226ME13D**; Samsung **CL05B104KO5NNNC**, **CL10A106KP8NNND**, **CL05B104KB54PNC**, **CL05A105KA5NQNC**; TDK **C2012X5R1V106K125AC**; Yageo **CC0805MKX5R8BB226**; Taiyo Yuden **MCASG168AB7105KTNA01**. Murata identifies ordinary chip MLCCs as MSL1, representative of the normal no-bake handling for this class. [Murata MLCC MSL guidance](https://www.murata.com/support/faqs/capacitor/ceramiccapacitor/mnt/0009)
- Ferrites/inductors: TDK **MPZ2012S601ATD25**; Murata **LQW18AN68NJ00D**, **DFE252012F-1R0M=P2**; Samsung **CIGT201610EHR47MNE**. Reflow normally to the manufacturer's product-family profile; observe any dry-pack label rather than inventing a generic bake.
- Resistors: Yageo **RC0402FR-0710KL**, **RC0402FR-0715RL**, **RC0402FR-07100KL**, **RC0402FR-0722RL**, **RC0402FR-075K1L**, **RC0402FR-075K62L**, **RC0402FR-075K23L**, **RC0402FR-07499RL**, **RC0603FR-07511KL**, **RC0603FR-0791KL**, **RC0402FR-0722KL**; Vishay **CRCW040230K1FKED**; Stackpole **RMCF0402FT75K0**. These 0402/0603 thick-film chip resistors are standard reflow parts and do not need post-reflow installation.

The BOM's `FB1,FB2` value says `BLM21SP601SN1D`, but its specified manufacturer part is **MPZ2012S601ATD25**. Assembly/purchasing should follow the MPN, not the stale value text.

## Practical paste and sequence guidance

- Once a side has reflowed correctly, **do not apply new paste to its already-soldered joints** before heating the other side. New paste is for bare pads/components being placed; use flux only for actual rework.
- Do not treat pre-tinned bare pads as a substitute for a controlled paste deposit under the UM980 or WROOM. Their numerous hidden lands need controlled solder volume and inspection. Hand-dispensing a few blobs or heating until the edge “looks melted” cannot verify the centre joints.
- The USB-C receptacle, microSD socket, FPC connector, U.FL receptacle, DF58 board connector, and B3U switches should be populated during their side's controlled reflow, not saved for arbitrary hot-air attachment later.
- The only clear “install later” objects are mating/non-BOM items: Li-ion cell, microSD card, FPC/display, U.FL antenna cable, and USB cable.
- A second-side component falling is not determined by weight alone. Pad geometry, molten-solder surface tension, paste volume, component centre of gravity, time above liquidus, board vibration, and hot-air velocity all matter. Here the manufacturer-specific UM980 and WROOM cycle restrictions decide the process before a probability estimate does.

## Recommended disposition

1. For a repeatable build, revise placement so U1 and U2 are on the same side, then give both a single controlled SAC305 reflow inside the common window (peak not above the UM980's 245 °C limit, while satisfying the WROOM's 235–250 °C requirement).
2. If the PCB cannot change, send the exact stack-up, paste/alloy, and placement files to a PCBA vendor and require a written selective-reflow/rework plan with thermocouple evidence. Because U1/U2 overlap, “shield the other side” is not enough by itself.
3. Inspect hidden module joints by X-ray. Electrical boot/GNSS tests alone do not rule out voids, head-in-pillow/non-wet joints, or marginal ground-pad soldering.

## Source limitations

Connector and module manufacturers often do not assign a numeric MSL to non-plastic-IC products in public catalogues. Where no numeric MSL is published above, follow the delivered moisture-barrier bag/HIC label and the latest manufacturer revision rather than assuming MSL1. The current ESP32-S31-WROOM-3 datasheet is marked preliminary, so recheck it before production release.
