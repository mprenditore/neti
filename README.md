# NETI Gatekeeper

NETI Gatekeeper is an open source project created to control a garage door with 3 phases motor using a single phase input with all the sensors needed.
The PCB is designed with modularity in mind so that it will be easy to swap faulty hardware with new one.

## BOM

- Wemos D1 Mini
- 2 channels relay board
- ACDC 5V power supply module
- 12/24V power supply for additional hardware (Eg. photosensors, radio receiver)
- EDG connectors (3.81mm pitch)
  * 1x 2P (Vout)
  * 1x 3P (Motor)
  * 1x 3P (Lock relay for Mk3 and Mk6)
  * 1x 7P (I/O)
- IEC C8 connector
- 1 channel relay board (Mk3 and Mk6 versions)
- Fuse and fuse holder (Mk5 and Mk6 versions)
- 2x 2P screw terminal blocks (5mm pitch)
- Buzzer 12mm

## Motor connection diagram

The motor can be wired both in *Star* or *Delta* and will require a Capacitor to invert the motor direction.

```
           C *____________
            /  \         |
           {}  {}        =
        L /      \ N     |
         *___{}___*      |
         |        |      |
         |        |      |
         |        |      |
         |        |      |
         |        |      |
        _L________N______C__
        |                  |
        |     NETI PCB     |
        |__________________|

```