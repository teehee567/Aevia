# v2_mini manual double-sided reflow guide

> [!WARNING]
> There is no fully manufacturer-approved two-pass process for this PCB. The UM980 and ESP32-S31-WROOM-3 overlap on opposite sides. Unicore does not recommend two soldering cycles for the UM980, and Espressif specifies a single SAC305 reflow for the WROOM. The Sn63/Pb37 process below is therefore an unqualified prototype method, not a production process.

This procedure is tailored for fresh eutectic 63Sn/37Pb solder paste, front and back stencils, suitable flux, magnification, a temperature-controlled hot-air station with low airflow, no thermocouples, and no rigid PCB holder. It assumes there is a workable way to hold the PCB flat while printing paste.

Sn63/Pb37 paste becomes fully liquid at approximately 183 degrees C. Follow the temperature and timing limits printed on the solder-paste container or its technical data sheet; they take precedence over the generic guidance here. Do not use the SAC305 temperatures from the component datasheets as an Sn63/Pb37 target. The components may have lead-free terminal finishes, so the completed joints are mixed metallurgy and may not behave as perfectly eutectic Sn63/Pb37.

Without thermocouples, the actual joint temperature, peak temperature, and time above liquidus cannot be verified. The visible cues below can reduce guesswork, but they cannot prove that hidden centre or ground-pad joints reflowed. Do not treat the hot-air station's displayed air temperature as the PCB temperature.

For the detailed component-by-component assessment and manufacturer sources, see [reflow-assembly-audit.md](reflow-assembly-audit.md).

## Equipment and setup

Required:

- Temperature-controlled hot-air station with adjustable low airflow; do not use a paint-stripping heat gun
- Front and back stencils
- Fresh Sn63/Pb37 solder paste within its shelf life
- Suitable no-clean flux for touch-up and rework
- Magnification
- Fine tweezers
- Stopwatch or timer
- A stable, level, non-combustible support arrangement with an open cavity below underside components
- Fume extraction and normal lead-handling precautions

A PCB preheater is strongly recommended. It reduces the time spent blasting the modules with hot air and reduces temperature gradients. A preheater is not a substitute for a thermocouple, however.

Let refrigerated paste reach room temperature while still sealed, for the time specified by its maker. Mix it only as directed, use a clean paste tool, and do not return used paste to the fresh container. Keep food and drink away from the bench, collect lead-contaminated waste, clean the work surface, and wash hands after handling the paste or assembled PCB.

The paste Gerbers are:

- [Front paste](production/gerbers/v2_mini-F_Paste.gtp)
- [Back paste](production/gerbers/v2_mini-B_Paste.gbp)

Stencil one side at a time. The existing stencil workaround is acceptable only if it keeps the PCB flat, does not smear the first-side paste or components, and leaves the stencil parallel to the board. Inspect every aperture deposit under magnification before placing parts.

The stencil workaround is not automatically suitable as a reflow support. Before heating, make a separate support arrangement that:

- Contacts only unused PCB edges
- Leaves at least 5 mm clearance beneath all populated components
- Cannot soften, tip, spring, or slide when hot air is applied
- Holds the PCB horizontal without bending or twisting it
- Does not significantly block airflow or act as a large heat sink at one edge

Test the cold setup with gentle taps and the intended air setting before placing paste. If it moves, improve it before attempting reflow.

## 1. Check moisture exposure

If the components were freshly removed from sealed dry bags, proceed within their specified floor life.

Bake according to the packaging label and J-STD-033 if:

- The WROOM or SCH16T has been exposed for more than 168 hours
- An MSL3 LED has exceeded its floor life
- The humidity-indicator card shows excessive moisture

Remove components from tape, reel, or other temperature-limited packaging before baking. Do not improvise baking temperatures; follow the component bag label or manufacturer instructions.

## 2. Learn the paste's visual behaviour

If possible, make a small practice print on a scrap PCB with similar copper and thermal mass before assembling this board. Use the same stencil, paste thickness, support arrangement, hot-air nozzle, distance, and airflow planned for the real board.

Observe the sequence under magnification:

1. The paste warms and the flux activates.
2. The deposits slump slightly.
3. The solder changes quickly into a bright, mobile liquid and pulls toward the pads.
4. On cooling, the bulk paste deposit freezes quickly; mixed-metal joints may be less abrupt.

Use the practice piece to find a conservative station setting and hand motion. Record the air setting, nozzle distance, warm-up time, and time from the first joint becoming liquid until the last joint becomes liquid. These are process notes, not measured component temperatures.

Do not add extra flux to normal stencil deposits. Fresh paste already contains flux, and extra liquid can make small parts float or move. Reserve the separate flux for touch-up, rework, and the USB-C shell stakes.

## 3. Reflow the back first

The back contains U2 WROOM, U3 SCH16T, J3 FPC connector, and the switches.

1. Hold the bare PCB completely flat using the stencil workaround.
2. Stencil the back with fresh Sn63/Pb37 paste.
3. Lift the stencil cleanly and inspect every deposit under magnification. Clean and reprint the whole side if important deposits are smeared, bridged, incomplete, or misregistered.
4. Place the small passives and ICs first.
5. Place the switches and FPC connector.
6. Place the SCH16T and WROOM last.
7. Do not manually add paste blobs beneath the WROOM EPAD. Excess paste can lift the module and prevent its other pads from contacting the PCB.
8. Transfer the assembly to the stable, non-combustible reflow support without flexing the PCB.
9. If a preheater is available, warm the complete PCB gradually and evenly, staying within the solder-paste maker's preheat guidance.
10. Apply hot air at the lowest useful airflow. Keep the nozzle moving in broad, overlapping circles so that the region heats evenly.
11. Watch several visible joints in different parts of the heated region, especially beside the largest thermal masses. Start the timer when the first visible Sn63/Pb37 deposit becomes fully liquid.
12. Continue broad heating only until all inspected witness joints have flowed and nearby parts have pulled into alignment. Use the paste maker's time-above-liquidus guidance; do not extend the dwell merely to make exposed joints look smoother.
13. Withdraw heat gradually. Do not touch, bump, or move the board while any joint is shiny or mobile. After the last visible joint freezes, leave the board undisturbed for at least two minutes.
14. Inspect every visible joint under magnification. Do not underfill or glue the WROOM.

Visible edge wetting does not prove that the WROOM or SCH16T centre pads reflowed. If the paste never changes cleanly from a deposit into a mobile fillet, stop after cooling and diagnose the setup rather than repeatedly cooking the board.

## 4. Stencil and populate the front

1. Let the board cool fully before turning it over.
2. Use the stencil workaround to support only unused PCB edges. Leave at least 5 mm clearance beneath the WROOM and ensure nothing presses against the WROOM, IMU, switches, or FPC connector.
3. Confirm that the PCB cannot rock or bow during the print.
4. Stencil the front with fresh Sn63/Pb37 paste.
5. Inspect the UM980 and fine-pitch IC deposits especially carefully. Clean and reprint the whole side if critical deposits are defective.
6. Place the small components first.
7. Place U1 UM980, the microSD socket, U.FL receptacle, battery connector, and USB-C receptacle last.
8. Do not install the battery, microSD card, USB cable, antenna cable, display, or FPC cable.

The USB-C footprint does not provide paste for its four through-hole shell stakes. Reflow the fine SMT contacts during the front-side pass and solder the shell stakes afterward.

## 5. Reflow the front

This is the highest-risk step. Because Sn63/Pb37 melts at approximately 183 degrees C, the WROOM joints on the underside may remelt earlier and remain liquid longer than they would with SAC305. The board must remain horizontal, supported, and completely still.

1. Place the PCB front-side up on the tested reflow support. Ensure that there is open clearance beneath the WROOM and that no support touches any component.
2. Recheck stability using a cold, low-airflow pass before applying heat.
3. If a preheater is available, warm the whole PCB gradually and evenly according to the paste maker's guidance.
4. Apply the lowest useful airflow and use broad, overlapping circles over the front assembly area.
5. Heat the UM980 evenly. Never concentrate the air stream on one corner and never press the module while the solder is liquid.
6. Start the timer when the first visible Sn63/Pb37 deposit becomes fully liquid.
7. Stop active heating once visible joints across the slowest-heating areas have flowed and the paste maker's minimum time-above-liquidus guidance has been met. Keep the liquid interval as short as practical; extra dwell increases the chance that the underside WROOM will shift or fall.
8. Withdraw heat gradually without changing airflow in a way that can disturb components.
9. Do not touch, bump, rotate, or move the PCB while any joint is shiny or mobile. After the last visible joint freezes, leave the assembly undisturbed for at least two minutes.

No visual-only method can confirm the temperature or reflow state of the overlapping hidden joints. Shielding may reduce direct hot-air heating, but it cannot stop heat conducted through the PCB.

## 6. Finish the USB-C connector

After the PCB has completely cooled:

1. Apply a small amount of flux to the four USB-C shell stakes.
2. Use a sufficiently large soldering-iron tip to heat each stake quickly.
3. Use the solder-wire maker's recommended iron setting. Approximately 300-330 degrees C is a reasonable starting range for Sn63/Pb37 with an adequately sized tip; increase heat transfer by using a larger tip before increasing dwell time.
4. Complete each joint within a few seconds and allow the connector to cool between stakes if its body becomes warm.
5. Inspect the fine reflowed USB contacts for bridges.

Do not attempt to form the fine hidden USB contacts using only a soldering iron.

## 7. Inspection and initial power-up

Before connecting the battery:

1. Inspect both sides under magnification for shifted components, bridges, solder balls, opens, and WROOM movement during the second pass.
2. Check resistance from 3.3 V to ground and from VBUS to ground.
3. Check the USB pins for bridges.
4. X-ray U1, U2, and the leadless power ICs if that service is available. This is the only practical inspection of their hidden joints.
5. Power the board from a current-limited bench supply.
6. Confirm that the WROOM boots.
7. Confirm that the IMU responds over SPI.
8. Confirm communication with the UM980.
9. Confirm that USB enumerates.
10. Only then connect the microSD card, display/FPC, antenna, and battery.

Electrical function does not prove that every hidden joint is mechanically sound. Treat the result as a prototype unless the hidden joints have been inspected and the process has been validated.

## Stop conditions

Stop heating and allow the board to cool if:

- The support shifts, rocks, bows, softens, or starts to tip
- Components skate or blow out of position
- Flux smokes heavily or chars
- The PCB, connector bodies, or component packages discolor
- Visible paste remains grainy or balled after nearby joints have clearly flowed
- The WROOM moves during the front-side pass

Do not immediately reheat a failed area. Let the board cool, inspect the cause under magnification, clean the area if needed, and decide whether a controlled local rework is still credible.

## Production recommendation

For a repeatable or production-quality assembly, revise the placement so the UM980 and WROOM are on the same side, or use a PCBA vendor-developed selective process with thermocouples and X-ray inspection. A successful visual-only prototype run does not establish a safe production profile.
