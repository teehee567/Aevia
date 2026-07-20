things to watch out for

## Important rules

- make sure ot use stack upand controlled impedence rules from jlcpdb before routing
- See how much 8 layer pcb will cost
- 2 complete full ground planes
- make fast signal return path short, continuous
- order to route, xspi ram.flash, rf/antenna, mcu decoupling, power, usb, display, slow signals
- check datasheet land pattern, exposed pad rule, antenna keep out, decoupling placement, reference alyout requirement.
- run erc/drc

## making small

- use 0402, if using 0201 chekc if jlcpcb does it on basic mode, use larger when voltage, capcitance, power needs.
- put fine pitch on same side and try make other side fully solderable with iron,
- make jlcpcb give stencils
- have some fiducials
- have a couple test pads for important stuff

## STM32N657
- do bga escape with stack up
- place bypass capacitor  next to pin with smhort power connection
- keep vdda filtering and current away from other stuff
- dont make a split groun island
- place hse and lse near mcu pins

## flash hyperram
- flash hyperram ons ame side and close to stm32
- clock path takes priority
- make memory bus short and direct
- match signals within groups required by interface, account of package dealy and via length and trace length
- give clocks space, keep inside limits on datasheet

## usb c
- place tpd8s300a behind connector
- for usb d+/d- constant spacing, same reference plane, similar via transiotn, minimal stub, geometry is important
- keep d+/d- away from switch nodes, inductors, crystals, rf, board edgesstitch ground near connector transition
- keep high current vubs out of sensitive digital area
- dcap bypass is 0.1uF 50v x7r

## power
- current loop needs to be small
- put input cacptiro, switching, inducotr, output capacitor next to pins close
- make sw node only as large as required, no large switch node polygon, no signals near
- no arbitrary gorund splits
- size copper, vias, connectors, for worst case current and temperature rise
- make sure copper and stuff to dissipate heat fo rcomponents
- put feedback dividers at regulator feedpack pin, sense output after power path as recommened by datasheet
- keep check voltage rating for every capacitor/resistor that sees vbus, sys, batter, switch node
- keep battery guage short compact

## gnss
- um980 rf on 50ohm using stackup, keep short, direct, free of stubs/vias.
- place antenna connector at edge
- solid ground under rf unless keepout
- via fence beside feed
- keep away from everything

## esp32-c6
- put pcb antenan at board edge and keepout

## imu
- put sch16t near rgid part of board, near screw holde
- make sure to knwo sensor axis orientation
- make copper around imu thermally symmetric
- try make copper cold
