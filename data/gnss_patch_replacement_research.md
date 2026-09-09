# V2 Mini GNSS patch replacement: exact prototype specification

- **Date checked:** 2026-08-18
- **Target:** Aevia V2 Mini, Unicore UM980, `+3V3_GNSS` antenna bias and board-mounted Hirose U.FL receptacle
- **Comparison meant by “both”:** the existing/planned Beitian BT-T076 active quadrifilar helix and a complete active patch assembly, using the H0/HW versus P0/PW matrix in [`gnss_antenna_design.md`](../gnss_antenna_design.md)
- **Budget decision:** under the revised hard **A$50 component cap**, buy the **Pulse Electronics / YAGEO `GNSSL1L2L5182530`**. It is an active stacked L1/L2/L5 patch, operates correctly from the V2 Mini's 3.3 V bias and has a direct I-PEX MHF I/U.FL plug. It was A$38.819 including GST at DigiKey Australia when checked. It is the strongest well-documented budget prototype found, but it is **not** a strict all-band substitute for the BT-T076. The cheapest manufacturer-documented strict all-band patch found is the Beitian `BT-T413` at about A$53.46 before shipping and connector customisation; the Taoglas `ADFGP.60A.01.0150D` remains the much better documented strict all-band reference.

## Under-A$50 recommendation

Order one **Pulse Electronics / YAGEO [`GNSSL1L2L5182530`](https://yageogroup.com/content/datasheet/asset/file/DATASHEET_GNSSL1L2L5182530)**, DigiKey order code `553-GNSSL1L2L5182530-ND`.

| Item | Exact specification |
|---|---|
| Australian price checked | A$35.29 ex GST / **A$38.819 inc GST** at [DigiKey Australia](https://www.digikey.com.au/en/products/detail/pulse-electronics/GNSSL1L2L5182530/23600037), with 2,843 units shown in stock; [element14 Australia](https://au.element14.com/pulse-electronics/gnssl1l2l5182530/antenna-1-164ghz-to-1-189ghz-rhcp/dp/4453993) showed A$42.603 inc GST and 19 units of global stock |
| Construction | Active stacked ceramic L1/L2/L5 patch with filters, LNA and ESD protection |
| V2 Mini power compatibility | 2.5-18 V input, 16 mA maximum: **passes at 3.3 V** |
| RF interface | 50 ohm, RHCP, directional; 100 mm of 1.13 mm coax with I-PEX MHF I plug, directly compatible with V2 Mini J1 U.FL |
| Frequency windows | L5: 1164-1189 MHz; L2: 1215-1237 MHz; L1: 1561-1602 MHz |
| LNA gain, typical | 30 dB at L5, 33 dB at L2, 28 dB at L1 |
| Noise figure | 2.2 dB at L5, 2.2 dB at L2 and 1.7 dB at L1 |
| Passive element | Return loss better than 15/13/10 dB and peak gain above 2/3.3/5 dBi at L5/L2/L1 respectively |
| Mechanical | 30 x 30 x 12.69 mm excluding cable, 12 g, -40 to +85 deg C |

This part needs no voltage converter, SMA adapter or replacement V2 Mini pigtail. Its 28-33 dB LNA gain also sits within the UM980 manual's 18-36 dB optimum-input-gain window. Strictly, the UM980 number and Pulse LNA gain are not guaranteed to use the same RF reference plane, but this is still a much better gain match than many cheap 40 dB survey pucks.

For a controlled first test, centre the module over the ground plane used by the Pulse characterization. The current Issue 24/08 manufacturer datasheet says that its published measurements were made on a **120 mm diameter ground plane**. Mouser's catalogue metadata instead says 80 x 80 mm; the primary Pulse/YAGEO test statement takes precedence. Pulse does not specify the plate alloy or thickness. A locally cut 1 mm aluminium disc is a reasonable rigid prototype choice, but that material/thickness is an engineering choice rather than a Pulse specification. The exposed module is ESD-sensitive and not IP-rated, so use a non-conductive protective fixture and provide strain relief for its small coax.

### Exactly what the budget part retains and loses

Based on the three published passbands, rather than the distributor's broad constellation labels:

| Constellation | Signals inside the published windows | Signals outside the published windows |
|---|---|---|
| GPS | L1, L2, L5 | None of the main UM980 GPS bands |
| Galileo | E1, E5a | E5b, E6 |
| BeiDou | B1I/B1C and B2a | B2I/B2b, B3 |
| GLONASS | G1 | G2, G3 |
| QZSS | L1, L2, L5 | L6 |
| NavIC / IRNSS | L5 | — |
| SBAS | L1, L5 | — |

The mapping above is an inference from the vendor's RF passbands and published GNSS centre frequencies. Pulse names GPS, GLONASS, Galileo, BeiDou and IRNSS applications, but does not publish a signal-by-signal checked matrix.

Therefore this antenna is suitable for a useful **common-band patch-versus-helix test**, but it is not a complete all-signal 1:1 replacement for the BT-T076. Compare only matched signals inside the Pulse windows. In particular, do not attribute a loss of Galileo E5b, BeiDou B2/B3, GLONASS G2/G3 or L6 observations to patch geometry; those signals are outside this antenna's specified passbands.

No reputable, in-stock active patch with documented L1/L2/L5 **plus** the 1207-1279 MHz E5b/B2/G3/B3/E6/L6 region was found below A$50 at an authorised Australian distributor. At this budget the honest compromise is reduced band coverage, not uncertain marketplace specifications.

## Direct-U.FL strict-all-band option

The best-value direct-fit step up from the Beitian price-floor part is the **Quectel [`YFGD000AA`](https://www.quectel.com/product/yfgd000aa-active-gnss-full-band-screw-mount-ppo-patch-cable-embedded-antenna/)**. This is a factory **I-PEX MHF1/U.FL** part with an 89 +/- 3 mm lead, not a custom cable request, so it plugs directly into the V2 Mini's Hirose U.FL J1. [DigiKey](https://www.digikey.com/en/products/detail/quectel/YFGD000AA/28144360) showed 12 units in stock at **US$46.79** when checked. At the contemporaneous US$/A$ mid-market rate that is about **A$66.60 ex GST / A$73.30 inc GST**, before any card-rate difference; the Australian order is above DigiKey's A$60 free-shipping threshold.

| Item | Quectel-published specification |
|---|---|
| Construction | Active full-band ceramic PPO patch, RHCP, directional, 50 ohm, screw mount |
| Strict frequency support | Continuous 1164-1300 and 1525-1606 MHz windows; the 27-page datasheet explicitly checks GPS L1/L2/L5, GLONASS G1/G2/G3, Galileo E1/E5a/E5b/E6, BeiDou B1I/B1C/B2a/B2b/B2I/B3, QZSS L1/L2/L5/L6 and NavIC L5 |
| V2 Mini power compatibility | 3-5 V and 19.6 +/- 4 mA: passes at 3.3 V, with much lower expected current than the BT-T413's 45 mA maximum |
| Active stage | 38 +/- 4 dB LNA gain, at most 2.5 dB noise figure, output VSWR below 2 and at least 60 dB filter rejection 100 MHz outside the two passband edges |
| Published matching | VSWR 1.17-1.9 at ten tested frequencies from 1176 through 1602 MHz |
| Passive RF data | Efficiency 31-77.8%, passive peak gain 0.6-3.97 dBi and axial ratio 0.85-2.56 dB at the ten test points |
| Mechanical | 78.6 x 75.6 x 16.2 mm, 60 g, 89 +/- 3 mm of 1.37 mm coax, -40 to +85 deg C; exposed embedded assembly rather than an IP-rated antenna |

This is the **recommended buy for a cost-conscious, direct-U.FL, strict-all-band patch test**. It costs only about A$20 more than the BT-T413 element before the Beitian's shipping and connector-customisation charges, while adding an ordinary stocked U.FL SKU, an authorised channel, a released 27-page manufacturer datasheet, complete band windows, filter rejection and per-band matching/efficiency/gain/axial-ratio measurements.

The one electrical caution is gain. Quectel specifies 38 +/- 4 dB and its plotted curves are approximately 38-39 dB through most GNSS bands, about 2-3 dB above the UM980 manual's 36 dB “optimum input gain” maximum before cable loss. That UM980 figure is not published as a damage limit, and the short lead adds some loss, but this is not the ideal nominal match. Test the unmodified unit first and inspect per-signal C/N0, AGC/overload symptoms and urban interference behaviour. If overload appears, attenuation would change the A/B chain and should be treated as a separately documented variant.

The stronger lower-gain alternative is the **Siretta [`ECHO50/0.1M/I-PEX/MHF1/S/17`](https://www.siretta.com/products/antennas/echo-50/)**. It is also a stocked standard 100 mm MHF1/U.FL active all-band stacked patch, but at roughly A$214-257 depending distributor/tax display. It uses dual orthogonal feeds and a hybrid coupler, is ground-plane independent, and publishes detailed per-frequency matching, active gain and axial-ratio data. Its 23 +/- 2 dB LNA gain and at most 1.5 dB noise figure are a cleaner match to the UM980. Choose it over Quectel only if the gain margin and dual-feed/ground-independent design justify approximately another A$150; Siretta still does not publish production phase-centre calibration. Its revision-1.0 text also has editorial inconsistencies: the compact band list omits QZSS L6 although the spectrum chart and 1278.75 MHz RF data include it, and its summary range ends at the nominal 1602 MHz GLONASS G1 test point.

Other direct-U.FL routes checked:

| Candidate | U.FL status | Result |
|---|---|---|
| Quectel `YFGD000AA` | Exact stocked 89 mm MHF1/U.FL SKU | **Recommended value choice:** strict all-band, detailed RF/filter data, 3.3 V compatible and about A$73 inc GST equivalent. Main caveat is approximately 38-39 dB measured LNA gain versus the UM980's 36 dB optimum maximum. |
| Siretta `ECHO50/0.1M/I-PEX/MHF1/S/17` | Exact stocked 100 mm MHF1/U.FL SKU | Strict all-band, dual-feed stacked patch, ground-plane independent and 23 dB LNA gain; better nominal UM980 gain match, but roughly A$214-257. |
| Calian/Tallysman `33-3997EXF-09-0150` | Exact stocked 150 mm U.FL SKU | Strict all-band, 28 dB gain and outstanding filtering/phase-centre documentation, but **A$567.31 at Mouser Australia**. This is the metrology-grade option, not a sensible first topology test. |
| Calian/Tallysman `33-3990EXF-09-0150` | Exact stocked 150 mm U.FL SKU | Strict all-band, 37 dB gain and extended filtering; US$364.57 at DigiKey/Mouser global listings. Its gain is also just above the UM980's published 36 dB optimum ceiling before cable loss. |
| Taoglas `ADFGP.60A.01.0150D` | Stocked only as 150 mm SMA male | Strict all-band and A$212.63 listed at Mouser Australia, but no stocked ADFGP.60 U.FL ordering code was found. A custom lead must be quoted, or the normal U.FL-to-SMA-female V2 pigtail retained. |
| Abracon `APXG6016GH` / `APXG6413GH-0600A` | Stocked standard is MMCX; manufacturer says cable/connector can be customised | Strict all-band, but the custom U.FL version has no ordinary one-piece distributor SKU. The stocked APXG6413 was US$160.11 at Mouser and needs an MMCX-to-U.FL lead. |
| Abracon `AANI-AH-0126-1` | Exact stocked 100 mm MHF1/U.FL SKU | Strict all-band and well documented at US$244.07 from DigiKey, but it is a **helix**, so it cannot answer the intended patch-versus-helix geometry question. |
| Taoglas `ADFGP.55A.07.0100C` / `ADFGP.50A.07.0100C` | Direct MHF-family/U.FL lead | Good lower-cost patches but not strict all-band: no complete E6/L6/B3/G2/G3 claim. |
| u-blox `ANN-MB2-00` | SMA only | Robust all-band external antenna with a 5 m lead; no U.FL production variant is listed. |
| 2J / Antenova active patches | U.FL exists on narrower-band products or as custom leads | No manufacturer-documented strict-all-band active U.FL patch comparable to Echo 50 was found in their current catalogues. |

For installation, direct U.FL is cleanest. For repeated A/B swapping, it is mechanically worse than keeping one U.FL-to-SMA-female pigtail installed and changing antennas at SMA; U.FL is a miniature internal interconnect with limited mating life. If the Echo 50 MHF1 version is bought, plan only a few carefully controlled swap blocks, power down before each swap, pull the plug vertically with the proper extraction tool, and strain-relieve the 1.13 mm cable.

## Cheapest strict all-band option found

The cheapest currently orderable **active ceramic patch** found with a manufacturer-published signal list covering the UM980's complete GNSS set is the Beitian [`BT-T413`](https://www.beitian.com/sys-pd/1589.html). Beitian's [official store listing](https://store.beitian.com/products/high-precision-rtk-five-star-multi-frequency-gps-l1b1-built-in-ceramic-antenna-gnss-satellite-positioning-can-be-customized) showed the `BT-T413` selectable, 100 units in stock and **US$37.28**. At the checked mid-market rate of [US$1 = A$1.434](https://wise.com/us/currency-converter/usd-to-aud-rate), that is approximately **A$53.46 before card conversion, shipping, GST/import treatment and any connector customisation**. Beitian calculates shipping only after destination details are entered at checkout, so the delivered Australian price cannot be established from its public page.

| Item | Beitian-published specification |
|---|---|
| Signals | GPS L1/L2/L5; BeiDou B1I/B2I/B3I/B1C/B2a/B2b; GLONASS L1/L2/L3; Galileo E1/E5a/E5b/E6; SBAS L1/L5; QZSS L1/L2/L5/L6; IRNSS L5 |
| Antenna | Ceramic patch, RHCP, 50 ohm, 3 dBi passive gain, less than 3 dB axial ratio and less than 2:1 output VSWR |
| Active stage | 33 +/- 2 dB LNA gain, less than 2 dB LNA noise figure, less than 2:1 output VSWR and +/- 2 dB passband ripple |
| V2 Mini power compatibility | **3.3-12.0 V**, 45 mA maximum: passes at the V2 Mini's 3.3 V bias, although current is substantially higher than the Pulse part's 16 mA maximum |
| Mechanical | 56 x 56 x 14.8 mm, 65 g, four 3.2 mm mounting holes, exposed OEM assembly with no published IP rating |
| Stock RF lead | 98 mm RG-178 terminating in **MCX-JW** (right-angle MCX plug), not U.FL and not the normal SMA male used by the BT-T076 chain |

The `BT-T413` is therefore the cheapest strict-band **antenna element/active assembly**, but the standard store configuration is not a literal connector-for-connector replacement. The store describes the product as customisable. Before paying, request either:

1. a normal-polarity **SMA male (`SMA-J`)** so it can reuse the V2 Mini's existing normal U.FL-to-SMA-female pigtail and be swapped against the BT-T076 at SMA; or
2. an **I-PEX MHF I/U.FL plug** if a direct connection to V2 Mini J1 is preferred.

Get the cable type, length, connector and final price confirmed in writing; Beitian does not publish the custom-connector surcharge. Do not order the stock MCX version unless an extra MCX-female-to-U.FL cable is acceptable. A documented Amphenol U.FL-to-MCX-socket cable alone was A$38.85 ex GST at DigiKey, which would erase most of this antenna's cost advantage.

This is the price floor, not the best-documented high-precision choice. Beitian publishes the required signal list, bias range, LNA figures and basic RF/mechanical data, but no frequency response plots, out-of-band filter-rejection values, radiation patterns, efficiency, phase-centre variation calibration, specified test ground plane or environmental qualification. In particular, the public specification documents LNA passband ripple but does **not** identify the filter topology or rejection. Treat it as an inexpensive topology prototype and validate current, C/N0 by signal, cycle slips and RTK behaviour against the BT-T076; do not treat it as a production-qualified equivalent to the Taoglas reference.

## Strict all-band procurement reference (over budget)

| Qty | Exact item | Purpose | Checked one-off price |
|---:|---|---|---:|
| 1 | Taoglas [`ADFGP.60A.01.0150D`](https://www.taoglas.com/datasheets/ADFGP.60A.01.0150D.pdf) | Active, dual-feed stacked RHCP all-band patch, LNA and filters, 150 mm RG-174, normal SMA male | A$212.63 listed by [Mouser Australia](https://au.mouser.com/c/passive-components/antennas/?application=GNSS), 50 listed in stock when checked |
| 0 or 1 | Amphenol RF [`336313-12-0100`](https://www.amphenolrf.com/en-us/assets/file/4065861603/) **or** Taoglas [`CAB.719`](https://www.taoglas.com/product/cab719-hirose-u-fl-to-100mm-1-13-to-smafbkst/) | 100 mm, 50 ohm, right-angle U.FL/AMC plug to normal SMA female bulkhead; required only if the current helix chain has no correct pigtail, or as a spare | Amphenol A$13.51 ex GST / A$14.861 inc GST at [DigiKey Australia](https://www.digikey.com.au/en/products/detail/amphenol-rf/336313-12-0100/5417986); CAB.719 was listed near A$7.19 |
| 1 | 150 mm diameter metallic ground plane, drilled for four M3 positions on a 70 mm pitch-circle diameter | Reproduces the plane used for the published passive antenna values | Fabricate locally; the public Taoglas sheet specifies “metallic” but not alloy or thickness |
| 4 | M3 fasteners plus spacers | Rigidly fixes antenna to plane | Use the spacing rule below |

Expected cost is **A$212.63 before the plane, shipping and tax** if the known-good installed pigtail can be reused, or about **A$226** with the Amphenol spare. The Taoglas spare lowers that slightly. This is materially below a Calian full-band survey/rail patch while retaining far better component disclosure than the BT-T076.

The simplest housed alternative is one **Taoglas [`AA.250.101111`](https://www.taoglas.com/datasheets/AA.250.101111.pdf)** at **A$247.71 listed by [Mouser Australia](https://au.mouser.com/ProductDetail/Taoglas/AA.250.101111?qs=Imq1NPwxi74KYLEO7xY0JQ%3D%3D)**, using that same installed U.FL-to-SMA-female pigtail. It costs only about A$35 more than the exposed ADFGP.60, before the ADFGP.60's locally fabricated plane and fixture. Price and stock are volatile; Mouser listed 17 units when checked.

Do **not** order RP-SMA. The patch has a normal-polarity SMA male. The pigtail must have a normal SMA female/jack at the antenna end and a U.FL/AMC plug at the V2 Mini end. If that is already the installed BT-T076 interface, leave the U.FL connection untouched and swap at SMA. This keeps the adapter and its loss common to both test cases and avoids unnecessary mating cycles on the small U.FL connector.

## What “1:1 replacement” requires here

The V2 Mini production BOM identifies J1 as `U.FL-R-SMT-1(01)`, L1 as 68 nH and C1 as 100 pF; the schematic connects J1's centre conductor to `+3V3_GNSS` through L1, while C1 DC-blocks the UM980 RF input. The antenna is not included in the production BOM. See the [V2 Mini GNSS schematic](../hardware/v2_mini_pcb/GNSS_Mini.kicad_sch) and [production BOM](../hardware/v2_mini_pcb/production/digikey_bom.csv).

A no-PCB-change active patch must therefore meet all of these requirements:

| Requirement | V2 Mini / UM980 need | ADFGP.60 result |
|---|---|---|
| Construction | Complete active RHCP patch assembly, not a bare ceramic radiator | Pass: dual stacked, dual-feed patch with SAW filter and LNA |
| RF coverage | UM980 signals across the low GNSS region and L1/G1 high region, including L6/E6 and G3/B3 if the receiver is to remain all-band | Pass by the vendor's frequency/band table: GPS/QZSS L1/L2/L5/L6, GLONASS G1/G2/G3, Galileo E1/E5a/E5b/E6, BeiDou B1/B2a/B2b/B3, NavIC L5, SBAS and L-band. BeiDou B2I is not named, but occupies the same 1207.14 MHz RF region as named B2b; treating that as B2I coverage is an RF-frequency inference, not an explicit Taoglas signal claim. |
| Polarisation / impedance | RHCP, 50 ohm | Pass |
| Bias | Must include 3.3 V | Pass: 2.0–5.5 V |
| Bias current | Must not exceed the prior 55 mA BT-T076 design expectation; lower is preferable because Mini shares the GNSS rail | Pass on typical figure: 19 mA; the datasheet does not publish a maximum, so measure the actual unit before release |
| Gain into receiver | UM980 manual calls 18–36 dB the optimum input-gain range | Pass on the published numbers: LNA/filter 24.0–27.0 dB typical; complete active gain 26.0–29.9 dBi |
| Input/output match | 50 ohm, controlled RF path | Pass; VSWR maximum 2:1 at the six published test frequencies |
| Connector | Must reach J1 U.FL without changing the PCB | Pass with the specified normal U.FL-to-SMA-female pigtail |
| Environment | Sun-loaded dashboard is a harsher case than room temperature | Antenna is specified for -40 to +85 °C, 95% RH non-condensing at 65 °C |

The UM980 bands and 18/30/36 dB optimum-input-gain figures come from the bundled [UM980 User Manual R1.9](./datasheets/UM980_User%20Manual_EN_R1.9.pdf). “Optimum input gain” is not labelled as an absolute operating or damage range, so do not add passive peak gain and LNA gain and treat 36 dB as a hard limit without Unicore confirming its reference plane.

## Exact ADFGP.60 published data

All numbers below are from Taoglas specification `SPE-22-8-143-D`, revision D dated 2024-05-03.

| Test frequency (MHz) | 1176.45 | 1227.6 | 1278 | 1561 | 1575.42 | 1602 |
|---:|---:|---:|---:|---:|---:|---:|
| Passive peak gain (dBi) | 1.6 | 3.8 | -0.3 | 4.1 | 4.4 | 3.7 |
| Axial ratio (dB) | 0.25 | 1.03 | 1.25 | 1.24 | 1.00 | 0.47 |
| LNA/filter gain, typical (dB) | 26.3 | 27.0 | 24.0 | 25.3 | 25.1 | 24.6 |
| Noise figure, typical (dB) | 3.5 | 3.4 | 3.3 | 3.6 | 3.6 | 3.9 |
| Complete active gain, published (dBi) | 26.0 | 29.8 | 29.8 | 29.8 | 29.9 | 28.7 |

Other published details:

- 80 mm diameter × 19.1 mm maximum, 73 g;
- 80 mm diameter, 1 mm FR-4 carrier PCB and 56.1 mm tin-finished steel shield;
- 150 mm RG-174 cable and normal SMA male connector;
- four 3.5 mm mounting holes equally spaced on a 70 mm PCD, sized for M3 hardware;
- minimum 100 mm diameter metallic ground plane;
- antenna-to-plane gap `A` of 0.5–20 mm with conductive fasteners/spacers, or 0.5–3 mm with non-conductive fasteners/spacers; and
- the passive gain, efficiency, axial-ratio and VSWR table was measured on a **150 mm diameter** ground plane. Use 150 mm, not merely the 100 mm minimum, for the first comparison.

## One-piece housed alternative: AA.250.101111

The Taoglas Comet [`AA.250.101111`](https://www.taoglas.com/product/comet-all-band-gnss-active-magnetic-mount-antenna/) is the better purchase if “complete replacement” means a finished antenna that can be handled, moved between vehicles and exposed to weather without designing an enclosure. Its revision-D datasheet (`SPE-23-8-241-D`) publishes:

- the same strict band coverage required here: GPS L1/L2/L5, QZSS L1/L2C/L5/L6, Galileo E1/E5a/E5b/E6, GLONASS G1/G2/G3, BeiDou B1/B2a/B2b/B3, NavIC/IRNSS L5 and L-band;
- RHCP and 50 ohm impedance, with maximum 1.5:1 VSWR at its ten tabulated test frequencies;
- 1.8–5.5 V bias and 16–21 mA typical current, compatible with the V2 Mini's 3.3 V bias;
- dual-stage LNA/filter gain of 23.5–26.4 dB and noise figure of 3.2–3.9 dB across the tabulated bands;
- axial ratio of 0.35–1.7 dB and passive zenith gain of -1.5 to 4.2 dBic across the tabulated bands;
- normal-polarity SMA male on 1 m RG-174 cable;
- an ABS, IP67, magnetic-mount enclosure, 86.4 mm diameter × 26 mm, 138 g, rated -40 to +85 °C; and
- optional permanent screw mounting via Taoglas bracket `MB.A.MA32X`.

It still needs a normal-polarity SMA-female-to-U.FL pigtail to mate to V2 Mini J1. Reuse the installed one if it is correct; order the specified cable only if none exists or as a spare. It is a complete housed antenna, not a direct U.FL product.

For the **controlled inside-windscreen topology experiment**, the ADFGP.60 remains the preferred first part. Taoglas specifies its plane diameter, hole pattern and antenna-to-plane spacing, so the patch and its 150 mm plane can be tilted together in a repeatable bracket. Its integral cable is only 150 mm, versus 1 m on the AA.250, which reduces the cable/placement difference from the short BT-T076 chain. The AA.250's housing, magnetic base and long cable are inseparable parts of the measurement. Its datasheet's six-hour field test was on a static open-sky rooftop, in free space and on a 30 × 30 cm plane; it does not establish the best mounting treatment behind a windscreen. That makes the AA.250 the more convenient external/roof comparator, but the ADFGP.60-plus-defined-plane assembly the cleaner way to answer whether the patch topology wins in the intended dashboard geometry.

## Important limits in the public data

This is the best reasonably priced strict-band prototype found, not a release-qualified survey antenna.

1. Taoglas's total-gain row does not numerically reconcile with its passive-peak plus LNA rows at 1278 MHz (`-0.3 + 24.0` is not `29.8 dBi`). PCO and PCV are also blank at 1278 MHz. Ask Taoglas to confirm the reference planes and correct table before a production decision.
2. Taoglas's [high-precision field-test table](https://www.taoglas.com/high-precision-gnss-antenna-performance/) lists the ADFGP.60 as tested in “Free Space” and reports 122.84 cm 2DRMS without RTK and 34 cm with RTK in its six-hour ZED-F9P setup. That conflicts with the product page's broad cm-level positioning claim and is much worse than several other Taoglas patches in the same table. It may reflect the free-space setup rather than the datasheet's specified plane, but the public page does not establish the cause. This is another reason to reproduce the 150 mm plane and judge the installed A/B data rather than the marketing claim.
3. The antenna is an exposed embedded assembly, not an IP-rated enclosed dash puck. It needs a protective RF-transparent fixture whose material and spacing are frozen for the whole test.
4. It is **not** physically 1:1 with the 43.5 mm × 40.8 mm BT-T076. The required plane is larger still. “1:1” here means one complete active antenna chain can replace the other at J1 without a PCB modification.
5. The V2 Mini has no antenna-current telemetry in the current design. Measure 3.3 V and current at the antenna end during bring-up and inspect for GNSS-rail droop or abnormal heating.

## Alternatives checked

| Candidate | Price when checked | Why it is not the strict recommendation |
|---|---:|---|
| Taoglas [`AA.250.101111`](https://www.taoglas.com/datasheets/AA.250.101111.pdf) | A$247.71 at [Mouser Australia](https://au.mouser.com/ProductDetail/Taoglas/AA.250.101111?qs=Imq1NPwxi74KYLEO7xY0JQ%3D%3D) | **Strict all-band and the best one-piece housed alternative**, not a rejected option. Prefer it for convenience, IP67 protection or an external magnetic-mount comparison. Prefer ADFGP.60 for the controlled inside-windscreen plane/orientation experiment described above. |
| Taoglas [`ADFGP.55A.07.0100C`](https://www.taoglas.com/datasheets/ADFGP.55A.07.0100C.pdf) | A$112.52 ex GST / A$123.772 inc GST at [DigiKey Australia](https://www.digikey.com.au/en/products/detail/taoglas-limited/ADFGP-55A-07-0100C/26724142) | Attractive budget A/B part: active, 1.8–5.5 V, about 18 mA, 100 mm cable and MHF I/U.FL-compatible plug. However, Taoglas does not claim QZSS L6, Galileo E6 or BeiDou B3 for it, so it is not a full UM980 1:1 replacement. Use only if the comparison is deliberately restricted to common L1/L2/L5-family signals. |
| Taoglas `ADFGP.50A.07.0100C` | Similar lower-cost class | Also omits L6/E6/B3 claims, and its public axial-ratio table is not consistently as strong. Not strict all-band. |
| Calian [`TW3990XF`](https://sites.calian.com/app/uploads/sites/8/2024/06/Tallysman%C2%AE-TW3990XF-Datasheet.pdf) | A$653.71 at [Mouser Australia](https://au.mouser.com/ProductDetail/Tallysman/33-3990XF-01-01?qs=HFfMDpzxxd3PGgLL092w5w%3D%3D) | Better environmental/filter disclosure and a supplied 100 mm plane, but the orderable no-cable TNC-female version is roughly three times the ADFGP.60 price and its 37 dB typical LNA gain is just above the UM980's published 36 dB optimum maximum before cable loss. Excessive for the first topology test. |

## Bring-up and A/B procedure

1. Confirm the existing helix label and connector. Call it BT-T076 only if the actual unit matches the Beitian model and revision.
2. Power down, disconnect the helix pigtail at J1 and connect the Pulse antenna's I-PEX MHF I plug directly to J1. The budget antenna cannot share the helix's SMA pigtail, so record this unavoidable feed-line difference and minimize U.FL mating cycles. Measure the 3.3 V antenna rail and antenna current during bring-up; the Pulse maximum is 16 mA.
3. Centre the Pulse module on the 120 mm diameter metallic test plane specified by its Issue 24/08 datasheet. Tilt the patch **and its plane together**. If the final product can only accommodate a smaller plane, test that as a separate integration variable after the reference-plane run.
4. Run H0 (helix axis vertical), HW (helix axis glass-normal), P0 (patch face/plane horizontal), and PW (patch face/plane glass-normal). Add 15° and 30° if possible.
5. Keep antenna phase-centre location, cable route, receiver settings and fixture repeatable. The Pulse antenna's integral 100 mm cable remains part of its assembly.
6. Compare matched per-signal C/N0 by band/elevation/azimuth, observation availability, residuals, cycle slips, RTK fixed time and lap-line repeatability. Restrict the topology conclusion to signals inside both antennas' specified passbands; do not decide from satellite count or one peak-gain number.
7. Keep the BT-T076 as the deployment baseline until the patch repeats a win across multiple satellite geometries and complete laps.

## Sources

- Aevia [`gnss_antenna_design.md`](../gnss_antenna_design.md), [V2 Mini GNSS schematic](../hardware/v2_mini_pcb/GNSS_Mini.kicad_sch), [V2 Mini production BOM](../hardware/v2_mini_pcb/production/digikey_bom.csv), and [UM980 User Manual R1.9](./datasheets/UM980_User%20Manual_EN_R1.9.pdf).
- Pulse Electronics / YAGEO, [`GNSSL1L2L5182530` Issue 24/08 manufacturer datasheet](https://yageogroup.com/content/datasheet/asset/file/DATASHEET_GNSSL1L2L5182530), [DigiKey Australia listing](https://www.digikey.com.au/en/products/detail/pulse-electronics/GNSSL1L2L5182530/23600037), and [element14 Australia listing](https://au.element14.com/pulse-electronics/gnssl1l2l5182530/antenna-1-164ghz-to-1-189ghz-rhcp/dp/4453993).
- Beitian, [`BT-T413` official manufacturer page](https://www.beitian.com/sys-pd/1589.html) and [official store listing with specification images, live price and availability](https://store.beitian.com/products/high-precision-rtk-five-star-multi-frequency-gps-l1b1-built-in-ceramic-antenna-gnss-satellite-positioning-can-be-customized); [Wise USD/AUD mid-market rate](https://wise.com/us/currency-converter/usd-to-aud-rate) used only for the approximate Australian-dollar conversion.
- Taoglas, [`ADFGP.60A.01.0150D` product page](https://www.taoglas.com/product/allband-gnss-high-precision-patch-antenna/), [revision-D datasheet](https://www.taoglas.com/datasheets/ADFGP.60A.01.0150D.pdf), [`AA.250.101111` product page](https://www.taoglas.com/product/comet-all-band-gnss-active-magnetic-mount-antenna/), [revision-D AA.250 datasheet](https://www.taoglas.com/datasheets/AA.250.101111.pdf), [high-precision GNSS field-test table](https://www.taoglas.com/high-precision-gnss-antenna-performance/), and [`CAB.719` product page](https://www.taoglas.com/product/cab719-hirose-u-fl-to-100mm-1-13-to-smafbkst/).
- Amphenol RF, [`336313-12-xxxx` official cable-family sheet](https://www.amphenolrf.com/en-us/assets/file/4065861603/) and [`336313-12-0100` drawing](https://www.amphenolrf.com/library/download/link/link_id/591503/parent/336313-12-0050/).
- Beitian, [`BT-T076` official product page and datasheet images](https://www.beitian.com/en/sys-pd/620.html).
- Calian, [`TW3990XF` official datasheet](https://sites.calian.com/app/uploads/sites/8/2024/06/Tallysman%C2%AE-TW3990XF-Datasheet.pdf).
- Siretta, [`Echo 50` official product page](https://www.siretta.com/products/antennas/echo-50/) and [revision-1.0 Echo 50 datasheet](https://docs.rs-online.com/0bd2/A700000015436602.pdf); [Mouser Australia Echo 50 category listing](https://au.mouser.com/c/rf-wireless/antennas-accessories/?product+type=GNSS+Antennas+-+GPS%2C+GLONASS%2C+Galileo%2C+Beidou) and [element14 Australia exact MHF1 listing](https://au.element14.com/siretta/echo50-0-1m-i-pex-mhf1-s-17/internal-antennas/dp/4855660).
- Quectel, [`YFGD000AA` official product page](https://www.quectel.com/product/yfgd000aa-active-gnss-full-band-screw-mount-ppo-patch-cable-embedded-antenna/), [released version-1.0 YFGD000AA datasheet](https://cdn.soselectronic.com/productdata/7e/49/829c37ad/yfgd000aa.pdf), and [DigiKey exact-part listing](https://www.digikey.com/en/products/detail/quectel/YFGD000AA/28144360).
- Calian, [`TW3990EXF` official datasheet](https://www.tallysman.com/app/uploads/2024/04/Tallysman%C2%AE-TW3990EXF-Datasheet.pdf) and [official connector ordering guide](https://www.tallysman.com/app/uploads/2022/09/Ordering-Guide-EN-OCT5.pdf); [Mouser Australia `33-3997EXF-09-0150` listing](https://au.mouser.com/ProductDetail/Tallysman/33-3997EXF-09-0150?qs=HFfMDpzxxd1X8BCpJ3Zhsw%3D%3D).
- Abracon, [`APXG6016GH` official datasheet](https://abracon.com/datasheets/APXG6016GH.pdf), [`APXG6413GH-0600A` official datasheet](https://abracon.com/datasheets/APXG6413GH-0600A.pdf), and [`AANI-AH-0126` official datasheet](https://abracon.com/datasheets/AANI-AH-0126.pdf).
- u-blox, [`ANN-MB2` official product page](https://www.u-blox.com/en/product/ann-mb2-antenna) and [datasheet](https://content.u-blox.com/sites/default/files/documents/ANN-MB2_DataSheet_UBXDOC-963802114-12775.pdf).
- Distributor price/stock pages linked directly in the procurement and comparison tables; pricing and stock are volatile and were checked on the date above.
