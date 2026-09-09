# Aevia GNSS antenna decision: quadrifilar helix versus patch

- **Status:** design and validation note
- **Date:** 2026-08-17
- **Applies to:** Aevia V2 Mini, UM980, an antenna mounted inside the front windscreen
- **Decision confidence:** high on the installation principles; deliberately conditional on the exact replacement patch, windscreen construction, and measured vehicle geometry

**Hardware assumption:** the user reports that the installed antenna is helical, while the repository only identifies the BT-T076 as the *planned* helix. BT-specific values and connector conclusions in this note apply only if the label and datasheet revision confirm that the installed assembly is an authentic BT-T076. The current V2 Mini BOM does not identify an antenna or pigtail.

## Decision in one page

There is no topology-only answer in which every patch is better than every helix, or vice versa. For Aevia, orientation and installation can be worth substantially more than the small difference between two headline peak-gain numbers.

There are two different decisions: what is safe to deploy now, and which configuration has the highest ceiling after qualification. The recommended order is:

1. **Best absolute result:** a documented, full-band active GNSS antenna mounted externally near the centre of the metal roof, using the ground plane and mounting method required by its manufacturer.
2. **Safe interior baseline now:** retain the installed helix and test it first with its axis vertical. It needs no newly engineered patch plane and is the lower-risk choice while the exact patch, vehicle mask, and installed helix identity remain unknown.
3. **Highest-potential interior prototype:** a high-quality active patch that covers every signal Aevia intends to use, on the exact ground plane or integrated base for which it was characterised. Start with its face horizontal, then test 15°, 30°, and the measured glass-normal angle. Horizontal is the conventional patch baseline, but a forward tilt can win if it aligns the lobe with the car's actual clear windscreen aperture. It must accept the V2 Mini's 3.3 V bias and be compatible with the UM980 manual's stated 18–36 dB *optimum input-gain* window after the gain reference is clarified with Unicore.
4. **Do not choose from topology or tilt alone:** a large glass-normal tilt is risky for a directional patch because it sacrifices other sky sectors, but it is not automatically wrong. The installed antenna-plus-vehicle pattern, not the free-space antenna pattern alone, decides the result.

For the two requested comparisons:

| Installation | Helix | Patch | Decision today |
|---|---|---|---|
| Front of dash; boresight normal to the windscreen | Its broader angular response makes it the lower-risk unmeasured choice at a large tilt. It still points away from zenith and its installed pattern is unknown. | It may lose zenith/side/rear gain, or it may benefit by concentrating gain into the only clean forward-glass aperture. Glass-normal mounting does **not** reduce glass loss for a fixed satellite ray. | **Keep the helix as the safe baseline; no guaranteed RF winner.** Test this patch orientation rather than rejecting it if the forward aperture dominates. |
| Same dash location; boresight straight up | Good, robust baseline when a defined patch plane cannot be supplied. It may admit more reflected energy and may have less direct-signal gain than a properly installed quality patch. | Conventional baseline geometry. A high-quality patch on its specified plane is the preferred upgrade candidate for cleaner code/carrier and more stable phase-centre behaviour. | **Keep the helix now; select the patch only if installed tests show a downstream data-quality win.** |

The conditional language is intentional. The exact improvement cannot be stated from the present component data. The BT-T076 sheet gives one generic `3 dBi` gain number and `≤3 dB` axial ratio, but no gain or axial-ratio plots by frequency and direction. No replacement patch has been selected, and the windscreen rake and coating are unknown. A switch based only on advertised peak gain would therefore be guesswork.

## 1. What is actually in Aevia

The [V2 Mini design note](./v2_mini_design.md), schematic/PCB and README identify the current receiver as the **Unicore UM980** and the planned antenna as the **Beitian BT-T076**. The current V2 Mini has a U.FL antenna connector, an RF path designed and routed as a short 50 Ω connection, and a 3.3 V bias tee that is energised whenever the GNSS rail is powered. There is no production RF-impedance measurement in the repository. The current firmware is only a scaffold (`Hello, world!`), so there is not yet any Aevia observation log with which to compare antennas.

The [UM980 product page](https://en.unicorecomm.com/products/um980/) and the [R1.9 user manual stored in this repository](./data/datasheets/UM980_User%20Manual_EN_R1.9.pdf) specify reception of:

- GPS L1 C/A, L1C, L2P(Y), L2C and L5;
- BeiDou B1I, B2I, B3I, B1C, B2a and B2b;
- GLONASS G1, G2 and G3;
- Galileo E1, E5a, E5b and E6;
- QZSS L1 C/A, L1C, L2C, L5 and L6; and
- NavIC L5.

The manual gives 1.5 m horizontal and 2.5 m vertical RMS standalone positioning, 0.8 cm + 1 ppm horizontal and 1.5 cm + 1 ppm vertical RTK, 0.03 m/s velocity RMS, and up to 50 Hz RTK in a specific mode. It does not establish that all constellations, raw observations, and diagnostics can be output together at 50 Hz. Those are receiver test figures, not promises for a windscreen installation. Unicore explicitly makes performance dependent on satellite visibility and geometry, multipath, and antenna quality. Its table calls **18 dB minimum, 30 dB typical, and 36 dB maximum** the *optimum input gain*; it does not define those figures as absolute operating or damage limits.

The [BT-T076 manufacturer's page and datasheet images](https://www.beitian.com/en/sys-pd/620.html), revision 5.53 dated 2024-12, claim:

| Item | BT-T076 published value |
|---|---|
| Bands | GPS L1/L2/L5; GLONASS L1/L2; Galileo E1/E5a/E5b; BeiDou B1/B2/B3; QZSS L1/L2/L5/L6; NavIC L5; SBAS L1/L5. Galileo E6 and GLONASS G3 are not explicitly listed. E6 is close in frequency to QZSS L6, but coverage, group delay and performance must not be inferred from frequency overlap alone. |
| Polarisation | RHCP |
| Passive antenna gain | `3 dBi` (one value; no band, angle, reference plane, or uncertainty stated) |
| Axial ratio | `≤3 dB` (no angle or band stated) |
| VSWR | `≤2` |
| LNA gain / noise figure | `32 ±2 dB` / `≤2 dB` |
| Supply / current | 3.3–5.0 V / ≤55 mA |
| Phase-centre error | ±2 mm claimed |
| Connector and size | SMA-J; 43.5 mm diameter × 40.8 mm; ≤35 g |
| Missing information | 3-D RHCP and LHCP patterns; gain and axial ratio versus elevation/azimuth and band; mounting/ground-plane test condition; filter rejection and linearity; unit-to-unit tolerance by band |

This is enough to regard the BT-T076 as a plausible low-cost active wide-band helix. It is not enough to claim support or characterised performance for every UM980 signal, model installed signal quality, or prove the README's previous claim that it has lower peak gain but better low-elevation coverage than a patch. That pattern claim is a reasonable generic tendency, not a verified BT-T076 comparison.

## 2. Definitions that prevent an orientation mistake

- A **patch face** is its broad radiating ceramic/metal face. Its intended boresight is the line normal to that face. A horizontal face points vertically upward.
- A **helix axis** is the long symmetry axis through the quadrifilar helix. For this note, that axis is treated as its nominal boresight.
- Let **α** be the boresight tilt away from local vertical, toward the front of the vehicle. `α = 0°` means straight up. A windscreen-normal installation has `α` equal to the angle between the inward/outward windscreen normal and vertical; this must be measured on the actual car.
- “The helix lies flat on the dash and points up” means its base/support is on the dash and its **axis is vertical**. If the cylindrical helix is literally placed on its side, its axis is horizontal; that is a different and generally poor configuration.

For a satellite at elevation `e` and azimuth offset `φ` from the vehicle's forward direction, the off-boresight angle `θ` is

```text
cos(θ) = sin(e) cos(α) + cos(e) sin(α) cos(φ)
```

This simple geometry explains the difference between the two proposed installations. With `α = 0°`, `θ = 90° − e` for every azimuth: a nominally symmetric antenna is insensitive to yaw. With an illustrative, not assumed, windscreen-normal tilt of `α = 60°`:

| Satellite direction | Off-boresight angle `θ` |
|---|---:|
| 30° elevation, directly ahead | 0° |
| 30° elevation, directly to the side | 75.5° |
| 30° elevation, directly behind | 120° (behind the antenna) |
| Zenith | 60° |

At this illustrative 60° tilt, the antenna boresight favours one forward sector and places zenith, side, and rear directions far off-axis. That geometric fact does not supply the missing dB pattern, nor does it prove that the directions are usable or blocked in the car. As the car turns, the antenna lobe and the vehicle's forward-glass aperture rotate together while the satellites remain in Earth-referenced directions. Alignment between those two patterns can improve or worsen the combined result.

It is useful to separate two effects. A tilted directional antenna introduces **free-space antenna yaw sensitivity**; a horizontal, azimuth-symmetric antenna does not. But every **installed antenna-plus-vehicle pattern** is heading-dependent because the roof, pillars, glass aperture/coating, occupants and reflections also rotate relative to the satellites. A horizontal patch is only nominally yaw-symmetric before those vehicle effects. Heading-dependent C/N0 must therefore be measured, not assigned from orientation alone.

Measure `α` directly rather than relying on an ambiguous “windscreen rake” number. For example, glass described as 30° back from vertical has a normal about 60° from vertical, matching the table above. If the proposed bracket is actually only 30° from vertical, zenith is only 30° off-axis and the penalty is much milder; the same forward/rear and heading trade remains.

At `α = 30°`, a satellite at 45° elevation directly ahead is only 15° off-boresight rather than 45° for a vertical boresight, and one at 20° elevation ahead is 40° rather than 70°. The corresponding rear directions become worse. That may be a good exchange if the roof has already blocked the rear sky and the forward glass is the clean aperture; only the candidate's pattern and the installed sky mask can convert these angles into a dB or data-quality answer.

One further distinction is essential: **tilting an antenna at the same location changes the antenna pattern seen by the satellite; it does not change the physical satellite ray or that ray's angle through the glass.** Glass incidence is set by the satellite direction and windscreen plane. Pointing a patch or helix normal to the glass can align its boresight with a forward ray, but it does not make every ray traverse less glass or evade a metallic coating.

## 3. Why patch and helix behave differently

### 3.1 Radiation pattern and useful gain

GNSS satellites transmit right-hand circularly polarised (RHCP) signals. An antenna's useful figure is therefore not total LNA gain or a single peak dBi number. It is installed **RHCP realised gain as a function of frequency, elevation, and azimuth**, including mismatch and material losses.

A conventional patch concentrates response in the hemisphere above its face. It normally has maximum RHCP gain near its normal and rolls off toward the plane/horizon, with a ground-plane-dependent back lobe. ESA's [Navipedia antenna reference](https://gssc.esa.int/navipedia/index.php?title=Antennas) describes typical patch coverage of about 160° and notes that real GNSS antennas can lose roughly 10–20 dB from zenith to the horizon while axial ratio also deteriorates off-axis. These are generic orders of magnitude, not a prediction for an unspecified Aevia patch.

A quadrifilar helix is commonly designed for a smoother hemispherical response and is more tolerant of varying orientation. It is not isotropic, and “360° horizontal coverage” does not define its elevation pattern. The [u-blox GNSS antenna integration note](https://content.u-blox.com/sites/default/files/products/documents/GNSS-Antennas_AppNote_%28UBX-15030289%29.pdf) says that a reasonably sized helix will typically have less sensitivity than a comparably practical patch because comparable direct gain needs comparable aperture. The same note cautions that a helix can report more satellites in a difficult environment because its broad pattern also accepts reflections; those extra observations can have distorted ranges and worsen navigation.

Consequences for Aevia:

- A flat, properly integrated patch tends to maximise direct high-elevation signals and suppress energy from below or behind. This is attractive for low-multipath code, carrier phase, and RTK.
- A helix tends to give up some concentration for angular tolerance. This is attractive when the mount is tilted, the device rolls/pitches substantially, or no stable patch ground plane exists.
- More tracked satellites is not by itself a win. Fourteen clean, well-distributed direct signals can outperform twenty signals that include strong reflected paths.
- Small vehicle pitch and roll are usually far less severe than a 50–65° windscreen-normal tilt. They remain worth logging, but neither those motions nor free-space geometry alone determines the best installed angle.

### 3.2 Polarisation and axial ratio

An ideal GNSS antenna receives RHCP direct signals and rejects opposite-hand energy. A spec called **axial ratio (AR)** measures how close the polarisation is to circular: 0 dB is ideal; 3 dB is only the usual edge of a broad “circular” classification. Reflections can partially or fully reverse handedness, so low AR and low LHCP gain help reject multipath.

For an idealised elliptically polarised antenna at one direction, let `r = 10^(AR/20)`. Its ideal co-polar to cross-polar discrimination is

```text
XPD = 20 log10((r + 1) / (r − 1))
```

Thus 3 dB AR corresponds to about 15.3 dB ideal RHCP/LHCP discrimination, 1 dB to about 24.8 dB, and 0.5 dB to about 30.8 dB. The direct-signal mismatch at 3 dB AR is only about 0.13 dB; the larger concern is poorer rejection of the wrong handedness. These calculations are ideal values at a specified angle. A datasheet's one zenith AR number says little about the horizon, other bands, or the installed vehicle.

The BT-T076's `≤3 dB` claim is therefore adequate but not survey-grade evidence. A good replacement patch or helix should provide AR and RHCP/LHCP patterns over angle and frequency, not merely a checkbox saying RHCP.

### 3.3 Ground plane, phase centre, and nearby material

Patch performance depends on its RF ground plane. The u-blox integration note shows that ground-plane size, shape, symmetry, and patch placement can change gain, axial ratio, impedance match, resonance, directivity, and the back lobe. It gives 50 × 50 to 70 × 70 mm as a useful generic allowance for common L1 patches, but that is not a universal full-band prescription. Full-band stacked patches can require a larger, candidate-specific plane; for example, the representative Taoglas part discussed below is characterised on a 150 mm diameter plane.

A plastic dashboard above the vehicle's metalwork is **not automatically the documented RF ground plane**. Distance, electrical bonding, shape, and nearby displays/PCB/cables all matter. A random steel plate can also make a patch worse if its dimensions differ from the design condition. Follow the selected antenna's drawing and verify the installed match/pattern.

Helices are often designed without a ground plane, but this too is part-specific. u-blox reports that an intentional plane can improve some helix implementations by 2–3 dB, while metal within roughly 5 mm of a free-space helix's radiating section can distort its near field and pattern. The BT-T076 sheet does not state its test condition, so neither adding nor removing metal should be assumed beneficial.

Nearby glass, a radome, adhesive, the dash, display, PCB, cable, a person's hand, and vehicle metal change the near field. This can shift resonance and alter both the far-field pattern and phase centre. There is no reliable correction from dimensions alone; the choices are electromagnetic simulation and installed measurement.

For standalone metre-level use, a few millimetres of phase-centre variation may be unimportant. For RTK and IMU fusion, it matters. The antenna must also be rigidly mounted and its lever arm to the vehicle/IMU reference point measured. Body roll rotates that lever arm, and an unmodelled position offset can appear as a trajectory error even when the receiver solution itself is good.

## 4. The two requested mounting scenarios

### Scenario A: both boresights normal to the windscreen

Assumption: the helix axis and patch normal point outward perpendicular to the glass at the same front-dash location. The patch face is therefore approximately parallel to the glass. A fair orientation comparison must keep the antenna phase-centre xyz position, cable route, ground plane/base, enclosure and nearby hardware fixed as far as mechanically possible; moving them changes masking and multipath independently of angle.

**Helix:** This is a use case in which the topology's usually broader pattern is valuable. Compared with a conventional directional patch at the same large `α`, a well-designed helix may retain more usable gain toward zenith, side sectors, and some rearward directions. The price can be lower direct-signal gain and greater acceptance of dashboard, bonnet, glass, barrier, and ground reflections. There is no BT-T076 angular data or selected-patch pattern to say which is ahead, or whether the difference is 1 dB or 10 dB, in any particular direction.

**Patch:** This is the riskier unmeasured orientation when `α` is large. The forward low/mid-elevation sector may be strong, but zenith is `α` degrees off-axis, satellites beside the car are far off-axis, and much of the rear sky can enter through the patch's weak back hemisphere. On the other hand, the roof may already obscure the rear sky, so concentrating gain into the forward glass can be beneficial. The direction and magnitude of the trade depend on the installed sky mask, beamwidth, off-axis axial ratio, and the satellite geometry around the complete route.

There is one important exception: if an installed sky-mask survey shows that nearly all clean, usable sky is through a narrow forward windscreen sector while zenith and the sides are already blocked, aiming a directional patch into that aperture can outperform a vertical patch for those rays. It still must beat the vertical and intermediate-angle configurations over complete laps, because the favoured satellite azimuth changes with vehicle heading.

An automotive L1 experiment by Aloi et al., [“A detailed experimental study on the benefits of electrically grounding glass mounted GPS antennas to the vehicle roof”](https://doi.org/10.1049/iet-map.2013.0616), measured a 25 × 25 × 4 mm patch on a vehicle-roof mock-up at 45°, 60°, and 75° windscreen angles. It found that the roof distance, grounding arrangement, glass loading, and tilt all changed patterns; the roof-centre reference had about 5 dBc peak gain. The reported differences included roughly 1.5 dB trends from tilt and about 2.5 dB at some median pattern points from roof proximity/grounding. This is direct evidence that vehicle installation can move results by several dB, but it is **not transferable as a correction**: it was a tuned L1-only patch near the roofline, not an unspecified full-band patch on Aevia's lower dash.

**Verdict:** if constrained to a large glass-normal tilt before any measurement, the helix is the lower-risk baseline because it is less dependent on a newly designed plane and is likely broader. That is an integration-risk recommendation, not proof of better RF data. Sweep `α = 0°, 15°, 30°`, and the measured glass-normal angle for both complete assemblies. A modest or full forward cant may win if it matches the real aperture; it must demonstrate that win over complete laps and multiple satellite geometries.

### Scenario B: both boresights straight up from the dash

Assumption: patch face horizontal; helix axis vertical; the same phase-centre position, cable route and surrounding hardware as far as the different assemblies permit.

**Helix:** This puts its intended main region on the sky and removes deliberate free-space forward/rear asymmetry. It is the safest configuration for the existing antenna when no validated patch assembly is available. Its broad lower-elevation response may retain signals through the front and side glass, but can also accept more reflected energy. The installed vehicle pattern remains heading-dependent.

**Patch:** This is the correct baseline orientation for a conventional GNSS patch. If it covers all required bands, uses a good dual-feed RHCP design, and sits on its specified plane, it may provide stronger/cleaner high-elevation observations and better rejection of below-plane multipath than the current undocumented helix. That could help raw code and phase quality, ambiguity continuity, and repeatability. A high-quality helix can still outperform a mediocre patch, and a bare patch sitting on plastic with the wrong plane can be detuned enough to erase or reverse the expected advantage.

**Verdict:** a documented quality patch is the preferred upgrade candidate for raw-observation purity and RTK in this scenario, **not a guaranteed winner**. Until a specific patch passes the electrical and installed tests below, the current vertical helix is the lower-risk deployment choice.

### Scenario comparison

| Consideration | Tilted helix | Tilted patch | Vertical helix | Horizontal-face patch |
|---|---|---|---|---|
| High-elevation/zenith use | Conditional; zenith off-axis | Usually poor at large `α` | Intended geometry | Intended geometry; often strongest |
| Forward low-elevation sky | Favoured | Strongly favoured | Off-axis but usually usable | Off-axis; often downweighted anyway |
| Rear/side sky | Often smoother than a directional patch; unknown here | Can be weak/backside | Free-space azimuth balance is plausible | Free-space azimuth balance if the plane is symmetric |
| Added antenna-pattern yaw sensitivity | Moderate at a large tilt | Potentially high | Low if its free-space pattern is symmetric | Low if antenna/plane are symmetric; vehicle yaw effects remain |
| Multipath rejection | Candidate-specific; broad response can admit more | Directional, but useful alignment is installation-specific | Candidate-specific; broad response can admit more | Potentially best with a proper plane and low off-axis AR |
| Need for defined ground plane | Often lower, part-specific | High | Often lower, part-specific | High |
| RTK/raw phase potential | Must be measured | Must be measured against the aperture | Plausible baseline | Highest conventional-patch potential, still candidate-specific |
| Recommendation | Lower-risk of the unmeasured large-tilt choices | Test if it aligns with the real aperture | Keep as deployment baseline | Preferred upgrade candidate to qualify |

## 5. “How much better?” — what can and cannot be quantified

A 3 dB change is a factor of two in received power. In a stable, unsaturated front end, a 3 dB change in installed passive antenna gain or pre-LNA loss will appear approximately as a 3 dB-Hz C/N0 change for that ray. It does **not** mean twice the position accuracy. Position error depends on geometry, correlation/noise, multipath bias, receiver weighting, frequency availability, and whether tracking or RTK ambiguity thresholds are crossed.

For scale, the following is the *best-case thermal-noise-only* interpretation. The last column is the ideal measurement standard-deviation multiplier if every other error source is unchanged; real GNSS positioning often improves less because multipath, satellite geometry, atmosphere, phase-centre variation, and receiver weighting dominate.

| C/N0 improvement | Carrier/noise power ratio | Ideal noise-limited standard deviation |
|---:|---:|---:|
| 1 dB | 1.26× | 0.89×, about 11% lower |
| 2 dB | 1.58× | 0.79×, about 21% lower |
| 3 dB | 2.00× | 0.71×, about 29% lower |
| 6 dB | 3.98× | 0.50×, about 50% lower |

The following published magnitudes set the scale; they are examples from different antennas and test setups, not transferable Aevia prediction bounds:

| Mechanism/evidence | Magnitude | Meaning and limit |
|---|---:|---|
| Generic real GNSS antenna roll-off, zenith to horizon | about 10–20 dB | [ESA's reference](https://gssc.esa.int/navipedia/index.php?title=Antennas) gives this as a common range. A large tilt can move a satellite across much of that pattern, but no BT/candidate plot means no installed prediction. |
| Ground plane added to some helix designs | about +2–3 dB | Generic u-blox integration example, not a BT-T076 guarantee; a too-close/wrong plane can degrade it. |
| Experimental L1 windscreen patch installation changes | order 1.5–2.5 dB in reported comparisons | Aloi et al.; specific antenna, roof distance, tilt, and grounding. Shows “several dB” installation sensitivity, not a reusable Aevia number. |
| Ordinary 6–19 mm architectural glass examples at GPS L1 | about 1–4 dB | Broad literature examples summarised by [Liu et al.](https://link.springer.com/article/10.1186/s43020-020-00013-z), not direct measurements in Aevia or an automotive-windscreen specification; laminate, angle, frequency and setup differ. |
| Coated/energy-efficient glass examples | about 10–30 dB | Broad literature examples summarised by Liu et al., with construction and setup varying. They show that conductive coating can dominate; measure the actual vehicle. |
| 34 mm high-speed-train glazing experiment | up to 20 dB-Hz loss in surviving signals; 65% fewer tracked satellites | [Liu et al.](https://link.springer.com/article/10.1186/s43020-020-00013-z) also observed many more cycle slips and metre-level PPP degradation. This is a severe-glass case, **not a car prediction**. |
| Survey-grade patch versus two quality multi-frequency QHAs, 10–30° elevation | QHA code noise/multipath RMS about 30% and 70% higher; phase/high-frequency multipath about 40% and 70% higher | [Wanninger et al.](https://doi.org/10.1515/jag-2021-0042), static survey setup. It demonstrates that antenna quality changes observables, not that BT will match either QHA. |
| Same survey study, 5-minute fast-static coordinates | horizontal accuracy broadly similar/sub-centimetre; vertical RMS 0.80/0.93 cm patch, 1.02/1.51 cm one QHA, 2.08/2.50 cm the other | Raw observables can be 30–70% noisier while final horizontal results remain similar; vertical worsened about 1.3–2.7× in those tests. Dynamic windscreen results may differ completely. |

The most important conclusion from these examples is scale: headline peak differences between plausible candidate datasheets may be only a few dB, while orientation, an incorrect patch ground plane, a deep pattern null, or metallised glass can cost ten or more dB in affected rays. Those large effects can decide lock and ambiguity continuity; a one-number “3 dBi versus 4 dBi” comparison cannot.

One representative full-band active patch makes the comparison problem visible. The [Taoglas ADFGP.60A datasheet](https://www.taoglas.com/datasheets/ADFGP.60A.01.0150D.pdf) reports, on a 150 mm diameter ground plane, passive peak gain from approximately −0.3 dBi at L6/E6 to 4.4 dBi at L1, axial ratio roughly 0.25–1.25 dB by band, and total gain about 26–30 dB. This wide per-band variation shows why a patch cannot be represented by one peak-gain number. The BT sheet's single undifferentiated `3 dBi` claim has no band-specific value, angular definition, or stated test plane, so no numerical BT-versus-Taoglas subtraction is valid.

Likewise, a well-documented helix can be much better characterised than the current one. The [Calian HC990XF datasheet](https://sites.calian.com/app/uploads/sites/13/2024/06/Calian%C2%AE-HC990XF-Datasheet-Rev.-202304.pdf) specifies band-by-band gain around 1.8–2.3 dBic, ≤0.5 dB zenith AR, ±3 mm phase-centre variation, filtering, and no ground-plane requirement. Its lower headline gain than BT's generic 3 dBi does not make it lower quality. Pattern, polarisation purity, filtering, phase stability, and test disclosure matter.

## 6. How the antenna changes Aevia's result data

The antenna can improve the measurements available to the UM980; it cannot change the receiver's nominal update rate, correct bad timestamps, or make three decimal places in a lap time accurate by itself.

The causal chain is:

```text
installed antenna gain / polarisation / multipath / interference
        ↓
per-signal C/N0, tracking continuity, code and carrier residuals
        ↓
usable bands and satellites, geometry, cycle slips, RTK ambiguity state
        ↓
position and velocity error, solution availability and latency
        ↓
virtual-line crossing time and lap/sector repeatability
```

### Observable-level effects

- **C/N0:** better installed passive gain and lower pre-LNA loss raise C/N0 approximately dB-for-dB until receiver compression/interference effects intervene. More LNA gain alone amplifies signal and antenna noise together; it cannot recover SNR lost before the LNA.
- **Pseudorange/code:** stronger direct signals reduce thermal/code tracking jitter, while reflected signals introduce biased ranges that do not average like white noise. A broad helix can therefore show more satellites yet worse code residuals.
- **Carrier phase and lock:** good C/N0 and favourable tracking-loop/interference conditions reduce thermal phase noise and loss of lock. A low-quality signal can cycle-slip; RTK then has to repair or re-resolve ambiguities, creating fixed-to-float transitions and discontinuities.
- **Phase centre:** offset and variation create direction-, attitude-, frequency-, and mounting-dependent systematic phase/position bias. Stable, characterised phase-centre behaviour helps RTK consistency and lever-arm calibration, but it does not directly prevent cycle slips.
- **Multi-frequency availability:** an L1-only or weak-L5/L6 patch throws away part of the UM980's central advantage. The replacement must be evaluated per signal and per band.
- **Geometry:** a tilted directional pattern can eliminate useful side/rear/high-elevation signals, worsen DOP, and make quality vary with heading. Satellite count alone does not capture this.
- **Interference:** a high-gain but poorly filtered active antenna can overload its own LNA or the receiver near cellular, Wi-Fi, display, USB, SD, and switch-mode noise. The u-blox note explains that GNSS power is below the thermal-noise floor and that both in-band noise and strong out-of-band transmitters can drive receiver AGC/desensitisation.

### Position error to lap-time error

For a virtual timing line with unit normal `n`, position error `δr`, and vehicle velocity `v`, the first-order crossing-time error is

```text
δt_position ≈ (n · δr) / (n · v)
```

For a timing line approximately perpendicular to the racing line, this reduces to along-track position error divided by speed. Illustrative values are:

| Speed | 1.0 m along-track error | 0.10 m | 0.01 m | Distance travelled in 1 ms |
|---:|---:|---:|---:|---:|
| 60 km/h | 60 ms | 6.0 ms | 0.60 ms | 1.67 cm |
| 100 km/h | 36 ms | 3.6 ms | 0.36 ms | 2.78 cm |
| 200 km/h | 18 ms | 1.8 ms | 0.18 ms | 5.56 cm |

This is only the spatial term. Total lap-crossing error also includes GNSS time/message latency, PPS alignment, receiver dynamics, sample interpolation, start-line definition, antenna/IMU lever arm, body attitude, filtering, and software. At 20 Hz, epochs are 50 ms apart; interpolation can remove raw sample quantisation, but only if timestamps, velocity/trajectory modelling, and adjacent fixes are reliable. Reporting `0.001 s` is not evidence of 1 ms accuracy. At 100 km/h, 1 ms alone corresponds to just 2.8 cm along the track, considerably below the UM980's nominal standalone RMS and plausible only with a stable high-quality solution plus careful timing/processing.

A better antenna will most visibly improve **availability and tails**, not necessarily the average lap: fewer C/N0 collapses, fewer discarded signals and cycle slips, longer RTK-fixed intervals, smaller 95th/99th-percentile position excursions, and less heading-correlated lap-to-lap error. A patch could yield cleaner raw data yet similar final horizontal positions, as the Wanninger study illustrates. Conversely, crossing a tracking or RTK threshold can produce a step change much larger than the dB improvement suggests.

## 7. Antenna assembly types are not interchangeable

“Patch” can mean several electrically different products:

1. **Bare passive ceramic patch.** This is only the radiating element, perhaps with matching. It normally needs a carefully designed host ground plane, a short 50 Ω feed, matching/ESD work, and often a nearby LNA. Cable loss before the first LNA directly degrades system noise figure. It is not a plug-in substitute for an active SMA antenna.
2. **Active patch module.** This adds an LNA and usually filtering. It may have a small metal backplate but can still require a larger specified external ground plane. “Active” does not mean “ground-plane independent.” Read the characterisation condition.
3. **Active patch with an integrated, sufficient ground plane/base.** A larger enclosed assembly can control its own RF geometry. This is the most plausible interior patch upgrade if it is explicitly characterised freestanding or with its supplied plane.
4. **Roof-intended magnetic puck.** Its magnetic metal base and the vehicle roof may be part of the RF design. A puck specified “on a 100 mm ground plane” or “vehicle roof mount” should not be assumed to work the same on a plastic dashboard. A product explicitly specified ground-plane independent is different.
5. **Survey/geodetic antenna.** These often use stacked dual-feed patches, choke structures, stable phase centres, strong filtering, a large enclosure, and calibration. They can give the cleanest phase data but may be too large, expensive, or roof-dependent for Aevia.

Whenever `+3V3_GNSS` is powered, that rail feeds the U.FL centre conductor through a 68 nH inductor. It is the same filtered rail that powers the UM980. The existing 100 pF capacitor blocks DC only between the biased RF node and the UM980 `ANT_IN` pin; it does **not** remove DC from U.FL J1. The UM980 `ANT_DETECT`, `ANT_OFF`, `ANT_SHORT_N`, and `VCC_RF` pins are unconnected, so the present PCB has no firmware-controlled antenna enable, open/short indication, or antenna-current measurement. The UM980 manual recommends a separately protected `ANT_BIAS` supply for robustness, whereas V2 Mini shares the receiver rail.

A passive patch is therefore not automatically drop-in. Verify its DC input before connection: a candidate that is a DC short could pull down the shared GNSS rail through the bias inductor. If it is incompatible, depopulate the bias-feed inductor or add a deliberate DC block/bias-isolation or switching design. An active candidate must operate at the voltage that actually reaches it under load. If the installed helix is confirmed as a BT-T076, its stated 3.3–5.0 V range puts V2 Mini at its lower limit; measure voltage and current externally at the antenna end across battery, temperature, cable, and tolerance rather than accepting the rail label.

Keep three gain questions separate for an active assembly:

- **Satellite link and C/N0:** installed passive RHCP realised gain versus band and direction, plus pre-LNA loss and polarisation/multipath effects.
- **Noise figure and receiver drive:** antenna-LNA/filter gain and noise figure, followed by cable/adapter loss. Evaluate this with the Friis noise relationship and the actual component reference planes.
- **Interference headroom:** filtering, LNA and receiver compression, IP3/intermodulation, AGC and nearby in-/out-of-band emitters. GNSS satellite power itself is not the likely overload source.

The UM980 manual states an 18–36 dB *optimum input-gain* range but does not clearly say whether `G_ant` means the LNA/filter/cable chain alone or total directional active realised gain including the passive element. Therefore the BT-T076's `3 dBi` element claim must **not** simply be added to its `32 ±2 dB` LNA claim and compared with 36 dB as though that were a hard limit. Confirm Unicore's definition and each vendor's reference plane, then verify the complete chain with installed measurements. Observe AGC/jamming diagnostics if the UM980 protocol and selected firmware expose them.

V2 Mini uses U.FL, while the standard BT-T076 datasheet specifies SMA-J. If that is the installed antenna, the assembly needs a controlled U.FL-to-SMA pigtail; no pigtail type, length, loss, or accessory line is defined in the current Mini BOM. A different helical termination changes or removes that requirement. Keep any pigtail short and mechanically strain-relieved. An active antenna's LNA before the cable makes post-LNA cable loss much less damaging to system noise figure than the same loss before an LNA, but loss still lowers receiver input level and margin; adapters also add variation and failure points.

## 8. Windscreen, vehicle, and placement effects

### Glass

Ordinary uncoated glass costs a few dB in some reported non-automotive tests, but heat-reflective, athermic, IR, or metallised layers can dominate the entire link budget. u-blox warns that reception through a metallic-coated windscreen may be impossible and notes that some cars provide a small RF-transparent region, often near the mirror. The high-speed-train study cited above is a useful severe-case demonstration, not an automotive estimate.

Before choosing an antenna:

- check the vehicle manual and glass markings for athermic/heat-reflective/metallised treatment;
- identify any OEM toll-tag/GNSS transparent zone;
- avoid embedded conductive heater/defroster wires, camera/radar modules, and metallic tint;
- measure attenuation by matched inside/outside C/N0 tests per band and direction.

Changing antenna tilt at one position does not bypass coating. Moving the antenna to an RF window or outside does.

### Body masking and multipath

The roof, pillars, firewall, bonnet, and occupants block or reflect different sky sectors. A dashboard antenna does not see a clean half-sphere. An illustrative Rohde & Schwarz [GNSS vehicle obscuration/multipath test note](https://cdn.rohde-schwarz.com/pws/dl_downloads/dl_application/application_notes/1gp101/1GP101_1E_Obscuration_and_Multipath_GNSS_Receiver_Testing.pdf) uses different attenuation masks for windows, roof, and body panels precisely because position within a vehicle changes the RF mask; its sample dB values are a simulation model, not Aevia measurements.

Prefer the highest, most central clear location that does not interfere with airbags or sight lines. Keep the antenna away from the display, ESP32/Wi-Fi antenna, USB, SD clocking, switch-mode inductors, and long digital cables. The vehicle roof is usually the best combination of sky view, symmetry, and ground plane; the front dash is a compromise.

The bonnet, dash, glass, barriers, grandstands, other vehicles, and track surface all create multipath. A quality horizontal patch and ground plane can suppress below-plane energy. A helix's broader pattern preserves angular coverage but can accept more of it. Low-elevation satellites already travel through more atmosphere and encounter more multipath; retaining every one is not automatically worth degrading the observation set. Receiver elevation/C/N0/residual weighting remains important for either antenna.

### Dynamic installation tolerance

- A symmetric vertical antenna adds little intrinsic free-space yaw dependence; a tilted lobe adds more. The installed vehicle pattern is heading-dependent in both cases, so explicitly test every vehicle heading.
- Pitch, roll, vibration, and hills perturb `α`. A helix normally degrades more gently; a patch must have sufficient beamwidth around every intended installed angle.
- A few centimetres of mount movement can change a reflection phase by a large fraction of a GNSS wavelength. Use a rigid, repeatable fixture.
- Adhesive thickness, radome, cable routing, and metal plates are RF design variables. Freeze them with the antenna choice.
- If the installed helix is confirmed as BT-T076, its stated upper operating temperature is +70 °C. Confirm the actual sun-loaded dash thermal requirement; do not assume cabin air temperature represents the antenna enclosure.

## 9. What “antenna quality” should mean in procurement

Quality matters least in open sky when all candidate assemblies give strong C/N0, uninterrupted multi-band tracking, and a standalone metre-level solution. In that regime the receiver and software may hide modest observable differences. It matters most when the link has little margin, the vehicle creates strong reflections, carrier phase/RTK is used, nearby transmitters or digital electronics threaten overload, temperatures vary, or results must repeat across units and remounts. These are exactly the conditions in which low axial ratio, stable phase centre, filtering/linearity, documented patterns, and manufacturing tolerance are more important than a peak-gain claim.

Do not rank candidates by total active gain or price alone. Require, in priority order:

1. **Exact frequency coverage:** every UM980 signal Aevia intends to log, especially L1/E1/B1, L2, L5/E5, B2/B3, and L6/E6. Many inexpensive “multi-band” patches are only L1+L5 or L1+L2.
2. **Installed RHCP realised-gain patterns:** by band, at least versus elevation in orthogonal azimuth cuts; 3-D data are better. Require the mounting plane and cable reference.
3. **Axial ratio and LHCP rejection versus angle and band:** zenith-only AR is insufficient for windscreen multipath.
4. **Ground-plane and installation definition:** dimensions, symmetry, required bonding, clearances, radome and cable position. Prefer a supplied/integrated plane if an interior custom plane is impractical.
5. **LNA chain:** gain tolerance, noise figure, input/output match, 3.3 V operation and current, gain flatness, group delay, compression/IP3, and stability.
6. **Filtering:** in-band insertion characteristics and out-of-band rejection around cellular, Wi-Fi, and Aevia's digital/switching emissions. Strong gain without linearity/filtering can be worse.
7. **Phase behaviour:** phase-centre offset/variation by band, group-delay variation, and calibration availability. This becomes decisive for RTK and GNSS/IMU fusion.
8. **Environmental and production quality:** operating/storage temperature, humidity, vibration, connector strain, ESD, ingress if external, lot tolerance, traceability, and repeatability after remounting.
9. **Complete interface fit:** connector, cable type/length/loss by frequency, bias voltage/current, dimensions, mass, and mounting repeatability.

Peak passive gain mostly tells what happens in one direction under one test condition. A lower-peak antenna with low AR, smooth phase centre, good horizon coverage, strong filtering, and honest plots can produce better data than a higher-number part.

## 10. Required A/B test before changing the design

Because installation uncertainty is larger than the likely component margin, the decision should be experimental. Test the **complete assemblies**, not loose radiator elements.

### 10.1 Test matrix

At minimum:

| ID | Antenna | Orientation |
|---|---|---|
| H0 | installed helix complete assembly (BT-T076 only if confirmed) | axis vertical (`α = 0°`) |
| HW | same installed helix assembly | axis normal to measured windscreen |
| P0 | selected required-band active patch plus specified plane/base | face horizontal (`α = 0°`) |
| PW | same patch assembly | face parallel to / normal pointing through windscreen |

Add `α = 15°` and `30°` forward for both if fixture time permits. Include a documented external roof antenna as the reference. Test any passive bare patch only after making the bias interface safe and defining its LNA/ground plane.

### 10.2 Control the experiment

- Same UM980 hardware, firmware, constellations, frequencies, tracking/elevation masks, update rate, dynamic model, correction source, and output messages.
- Same antenna position to within a documented tolerance, with the same cable path and known cable/adapter loss. Record antenna serial number and complete assembly revision.
- Same windscreen and vehicle state. Record rake angle, antenna `α`, xyz position, ground plane dimensions, coating/tint, nearby devices, and whether heater/camera electronics are active.
- For simultaneous comparison, use two identical receivers and swap antennas between receiver channels halfway through to remove receiver bias. Keep antennas separated enough to avoid coupling while exposing nearly the same sky. If only one receiver exists, repeat tests and compare matched satellite/signal/elevation/azimuth bins rather than raw averages from different sky geometry.
- Static open-sky baseline first, then stationary inside-car test, slow 360° heading rotation in an open area, and repeated dynamic laps. Repeat on multiple satellite geometries and in both open and barrier-heavy portions of a circuit.
- Do not judge from one screenshot, time-to-first-fix, or satellite count.

### 10.3 Firmware/logging required

The current Rust firmware does not yet read or log GNSS. First obtain the UM980 commands/logs reference and confirm which fields the installed receiver firmware can output; the bundled user manual confirms NMEA and Unicore formats but not every diagnostic below. Before antenna qualification, record all available required fields without UART/SD drops:

- raw code/pseudorange, carrier phase, Doppler, lock time and loss-of-lock/cycle-slip flags for every constellation, satellite, signal and frequency;
- per-signal C/N0, used/rejected state, satellite azimuth/elevation, and tracking status;
- solution time, position, velocity, DOP/covariance or estimated accuracy, satellite counts, fix type, and RTK state (none/single/float/fixed);
- RTK correction age, baseline/solution status, ambiguity resets, time to first fixed solution, and fraction of time fixed;
- receiver AGC and interference/jamming diagnostics if the protocol and firmware expose them;
- PPS edge captured against a hardware timer, the corresponding GNSS time message, UART receive time, message latency, buffer overruns, SD write stalls, and firmware configuration;
- synchronised SCH16T IMU samples and its time mapping to PPS/GNSS epochs;
- session metadata: antenna type/serial, cable, adapter, externally measured supply voltage/current at the antenna, location, `α`, ground plane, vehicle, windscreen/coating, weather, track direction and software revisions.

The current PCB cannot report antenna open/short/current state through the UM980 because `ANT_DETECT`, `ANT_OFF`, `ANT_SHORT_N`, and `VCC_RF` are unconnected; that requires external instrumentation or a hardware revision. Confirm that UART bitrate and storage throughput can carry the selected raw multi-frequency messages at 20 Hz. Treat 50 Hz as a separate feasibility test: the manual permits up to 50 Hz RTK only in a specific mode and does not promise every raw signal and diagnostic simultaneously. An antenna comparison is invalid if the logger silently discards epochs differently.

### 10.4 Metrics

Compare by constellation, signal/band, 10° elevation bin, and forward/side/rear azimuth sector:

- median, 10th percentile, and distribution of C/N0 for common signals;
- C/N0 versus vehicle heading for individual satellites, revealing tilted-pattern yaw dependence;
- fraction of possible observations logged and fraction used in the solution;
- code-minus-carrier/multipath combinations, pseudorange and carrier residual RMS/tails;
- loss-of-lock and cycle slips per 1,000 observations;
- DOP and geometry after receiver weighting/rejection;
- RTK time to fix, fixed percentage, fix outages/resets, and float/fixed transition count;
- stationary horizontal/vertical scatter and relative dynamic position/velocity differences against the roof comparator;
- 50th/95th/99th-percentile absolute errors only against independent truth such as a calibrated high-grade survey GNSS/INS with a surveyed base, an optical system, or an appropriate track reference;
- lap-line crossing residual and lap/sector repeatability against a track transponder or independently surveyed/time-synchronised reference.

As a useful health check rather than a pass/fail spec, the u-blox note says a well-designed system commonly shows about 44–50 dB-Hz on strong high-elevation signals and about 47 dB-Hz with a standard active antenna. Receiver reporting conventions differ, so Aevia should compare **delta C/N0 on matched signals** and failure-tail metrics rather than impose u-blox's absolute value on UM980.

### 10.5 Measure total in-cabin installation penalty

Use two matched antenna/receiver chains, one immediately inside and one on an unobstructed external/roof comparator, then swap chains. Map C/N0 difference by band, elevation, azimuth and heading. This measures the **combined** effect of glass, different antenna location, roof ground plane, body masking, pattern and multipath; swapping removes receiver-chain bias but does not isolate those mechanisms. A single average will hide a coating or pillar that affects only one sector/band. True glass-only attenuation needs controlled glass/no-glass measurements with otherwise matched geometry, or a representative windscreen coupon in a suitable RF setup.

### 10.6 Pre-register the decision rule

Choose the primary outcome before examining candidate results—for example, RTK-fixed availability plus 95th-percentile lap-line crossing error if RTK timing is the target, or 95th-percentile standalone along-track error and velocity continuity otherwise. First repeat the H0 baseline, including remounting, over multiple satellite geometries and both track directions to measure normal test variability. A candidate wins only if its repeated/crossover confidence interval clears both that baseline variability and a pre-agreed practically meaningful improvement in the primary outcome.

Use matched-signal C/N0, cycle slips, residuals and satellite counts to explain the result, not to replace it. A repeatable 2–3 dB improvement across the actually visible sky is meaningful link margin, but there is no universal dB threshold that guarantees a better position or lap time. Do not select a candidate whose median improves while its outage rate, heading dependence or 95th/99th-percentile errors worsen.

## 11. Final decision matrix

These labels are design priors, not measured rankings.

| Need / condition | Vertical installed helix | Quality required-band patch, horizontal on specified plane | Windscreen-normal helix | Windscreen-normal conventional patch |
|---|---|---|---|---|
| Integration risk today | **Favoured baseline**; assembly already exists | **Unqualified** until part, plane, bias and pattern are defined | Orientation unqualified | Assembly and orientation unqualified |
| Free-space high-elevation response | Candidate-specific; geometry is conventional | **Promising candidate**, pattern-specific | Zenith is off-axis | Zenith is off-axis, potentially strongly at large `α` |
| Match to a forward-only clear aperture | Broad response may cover it | Main lobe not centred on it | Broad response aimed toward it | **Potentially favoured** if its lobe actually matches that aperture |
| Ground-plane dependency | Usually lower, but part-specific | High; exact plane/base required | Usually lower, but part-specific | High; exact plane/base and installed tilt required |
| Multipath and phase quality | BT/current part undocumented | Potentially strongest with good AR, PCV and back-lobe control | Current part undocumented | Could screen cabin reflections, but off-axis AR and geometry may worsen |
| Heading behaviour in vehicle | Little intrinsic yaw asymmetry; vehicle effect remains | Little intrinsic yaw asymmetry; vehicle effect remains | Adds antenna tilt asymmetry to vehicle effect | Adds the strongest directional tilt, which may align with or oppose vehicle aperture |
| Role in the test | **Deployment baseline H0** | **Preferred upgrade candidate P0** | Required comparison HW | Required comparison PW when forward aperture may matter |

Recommended decision:

- **Immediately:** keep the installed helix as the baseline, confirm whether it is actually the BT-T076, mount its axis vertically for H0, verify the 3.3 V bias under load, and complete the appropriate controlled cable/pigtail and logging path.
- **Prototype in parallel:** obtain one active patch assembly that covers every signal Aevia will use, with a defined or integrated ground plane, 3.3 V operation, vendor-confirmed receiver gain compatibility, good AR/pattern/filter data, and stable phase centre. Start horizontal, then test intermediate and measured glass-normal angles.
- **Do not select the patch because of peak gain alone.** Select it only if the four-way installed test improves common-signal C/N0/residuals, cycle-slip tails, RTK fixed availability, and lap-line repeatability without heading-dependent outages.
- **If the actual windscreen is metallised or the interior loses roughly 10 dB or more in important sectors/bands:** treat this as a provisional engineering trigger to stop optimising topology inside. Use the OEM RF-transparent aperture or move the antenna outside to the roof.
- **If Aevia's primary goal remains standalone lap timing rather than RTK:** the improvement may be modest in average horizontal position and much larger in bad-case consistency. The current helix mounted vertically may already be sufficient; spend engineering effort on PPS-aligned timestamps, interpolation, lever-arm calibration, IMU fusion, and validation against a transponder as well as on the antenna.

## 12. Open facts required for a release decision

1. Exact installed helix make, model, revision, connector and cable; confirmation whether it is the planned BT-T076.
2. Exact vehicle model, windscreen rake, construction/coating, and available RF-transparent area.
3. Exact antenna xyz location, permitted size, and whether an external roof cable is acceptable.
4. Exact replacement patch part number and whether it is bare, active, integrated-plane, or roof-puck construction.
5. Candidate per-band patterns, AR, PCV, ground-plane requirement, filter/linearity, 3.3 V current, cable and connector data.
6. Measured V2 Mini bias voltage/current and complete-chain gain/loss; confirmation of UM980 gain interpretation.
7. Installed VNA/return-loss result where accessible and the four-way GNSS observation test.
8. Whether release mode is standalone, correction-aided/DGNSS, RTK float, or RTK fixed; the optimum acceptance metric changes with it.

Until those are known, the honest conclusion is not “helix is X% better” or “patch is X metres better.” It is: **the current helix mounted vertically is the safest first baseline, not a proven optimum; a correctly integrated quality patch has the highest conventional-patch ceiling for clean RTK-capable data, but its best angle may be horizontal, intermediate, or glass-normal depending on the installed aperture; and glass/body effects can overwhelm both.**

## References

- Aevia, [V2 Mini design note](./v2_mini_design.md), [PCB design](./hardware/v2_mini_pcb/v2_mini.kicad_pcb), and [production BOM](./hardware/v2_mini_pcb/production/digikey_bom.csv), used for the fitted RF/bias architecture and current hardware status.
- Unicore Communications, [UM980 product page](https://en.unicorecomm.com/products/um980/) and [UM980 User Manual R1.9](./data/datasheets/UM980_User%20Manual_EN_R1.9.pdf).
- Beitian, [BT-T076 official product page and datasheet images](https://www.beitian.com/en/sys-pd/620.html), rev. 5.53.
- u-blox, [*GNSS antennas: RF design considerations for u-blox GNSS receivers*](https://content.u-blox.com/sites/default/files/products/documents/GNSS-Antennas_AppNote_%28UBX-15030289%29.pdf), UBX-15030289 R03.
- European Space Agency Navipedia, [*Antennas*](https://gssc.esa.int/navipedia/index.php?title=Antennas).
- L. Wanninger, M. Thiemig and V. Frevert, [*Multi-frequency quadrifilar helix antennas for cm-accurate GNSS positioning*](https://doi.org/10.1515/jag-2021-0042), Journal of Applied Geodesy 16(1), 2022.
- D. N. Aloi et al., [*A detailed experimental study on the benefits of electrically grounding glass mounted global positioning system antennas to the vehicle roof*](https://doi.org/10.1049/iet-map.2013.0616), IET Microwaves, Antennas & Propagation 8(11), 2014.
- Z. Liu, Y. Gong and L. Zhou, [*Impact of China's high speed train window glass on GNSS signals and positioning performance*](https://link.springer.com/article/10.1186/s43020-020-00013-z), Satellite Navigation 1:14, 2020.
- Taoglas, [ADFGP.60A all-band active patch datasheet](https://www.taoglas.com/datasheets/ADFGP.60A.01.0150D.pdf), used only as a representative full-band patch, not a selected component.
- Calian, [HC990XF full-band quadrifilar helix datasheet](https://sites.calian.com/app/uploads/sites/13/2024/06/Calian%C2%AE-HC990XF-Datasheet-Rev.-202304.pdf), used only as a quality/characterisation benchmark.
- Rohde & Schwarz, [*Obscuration and multipath simulation for GNSS receiver testing*](https://cdn.rohde-schwarz.com/pws/dl_downloads/dl_application/application_notes/1gp101/1GP101_1E_Obscuration_and_Multipath_GNSS_Receiver_Testing.pdf), used only to illustrate vehicle-dependent RF masking.
