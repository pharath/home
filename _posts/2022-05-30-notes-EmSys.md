---
title: "Embedded Systems"
read_time: false
excerpt: "Notes on Embedded Systems"
header:
  teaser: /assets/images/linux_teaser.jpg
  overlay_image: /assets/images/linux_teaser.jpg
  overlay_filter: 0.5 
toc: true
toc_sticky: true
categories:
  - EmSys
  - Notes
tags:
  - emsys
  - notes
---

# Microcontroller

- µC $\approx$ low-end microprocessor + memory + I/O + additional peripherals
- more general than ASIPs and SoCs
- Pentium Processor [FDIV Bug](https://en.wikipedia.org/wiki/Pentium_FDIV_bug)
    - [FDIV Bug deutsch](https://de.wikipedia.org/wiki/Pentium-FDIV-Bug)
    - see [history of microprocessors](https://www.heise.de/hintergrund/Die-ersten-25-Jahre-in-der-Geschichte-der-Mikroprozessoren-4981272.html?seite=4)
- modern µCs nowadays may have more than one microprocessor core

## Structure of a microcontroller

- Clock
    - clocks that use an electronic oscillator regulated by a quartz crystal to keep time
    - [crystal oscillator Schaltzeichen](https://en.wikipedia.org/wiki/Crystal_oscillator)
- Volatile mem (SRAM) "Flüchtiger Speicher"
    - is deleted, when voltage is turned off
- Non-Volatile mem (EEPROM, Flash)
    - is stored permanently, even if voltage is turned off
    - different technology than RAM: you cannot write, erase and overwrite as often as you can with RAM
        - a car's control devices' µCs are overwritten or erased only very rarely
    - for PCs: BIOS is stored in a ROM chip
        - if you turn on your PC and the PC boots, then first the boot program is loaded from the BIOS into the RAM. After that the OS takes over. From then on all the interesting stuff happens in the RAM.
    - for EmSys/microcontrollers: non-volatile mem plays a much bigger role because you don't have a HDD/SSD or an internet connection over which you could load any programs. Usually e.g. a motor control µC ships with the software stored in the EEPROM/flash memory. When the motor control µC boots this software is loaded into the RAM and the software execution also happens in the RAM. The EEPROM/Flash mem is also where the control parameters are stored, i.e. the characteristic curve, lookup tables, etc.. This is why the size of EEPROM/Flash mem compared to RAM is much higher for µCs than for PCs.
        - e.g. if you bring a car to the inspection, the control algorithms are updated, i.e. the control devices are flashed, bugs are fixed, new functions are added or improved, etc. The car workshops only have to deal with the non-volatile mem. They have test equiqment with which they can access the car's network. They get a CD or DVD from the car's manufacturer and install the new software on the car's control devices.
- serial interface
    - RS-232
    - USB
- Bus controller
    - Bus interfaces: e.g. 
        - CAN Bus
        - Ethernet (increasingly important in industry automation)
- D/A converters
    - e.g. in order to calculate a digital value with microprocessor, convert this value and then give an external motor an analog control signal
    - quick n dirty conversion: use a PWM signal

## Digital I/O pins

- to monitor and control external hardware
- grouped into **ports** of 8 pins (if 8-bit architecture)
    - bidirectional (i.e. can be used as input or output pins)
- monitoring, access and control of digital I/O pins via **3 Registers for each port**:
    - Data Direction (DDR) [Register consists of 8 bit]
        - the corresponding bits in the DDR Register specify if the corresponding pin is an input pin (0) or an output pin (1)
    - Port (PORT) [8 bit, same name as corresponding pin group e.g. PORTA]
        - **for output pins** the corresponding bits in the Port Register specify the output pins' output values, i.e. high 5V (1) or low 0V (0)
        - **for input pins** the corresponding bits in the Port Register control **pull-up resistors**
    - Port Input (PIN) [8 bit]
        - read only
        - **for input pins** contains the current value (high or low)
        - **for output pins**: contains the same values as in PORT Registers (usually we do not need these values because we can read them from the PORT Registers)

### Pull-up Resistors

- example: reading a button
- geöffneter Schalter/"Stück Draht in der Luft" $\Rightarrow$ **Problem**: the voltage at the pin and therefore also the PIN Register bit corresponding to the (physical) pin will be undefined
- **pull-up resistor** connected to VCC "pulls up" the (undefined) pin to 5V
    - allows the voltage to "collapse", i.e. voltage behind the pull-up resistor is 0, **if the button is pressed**
        - Note: voltage behind the pull-up resistor is VCC, **if the button is not pressed**

# Bus Systems

- [List](https://automotive.softing.com/standards/bus-systems.html)
    - CAN
    - CAN FD
    - FlexRay
    - Ethernet
    - K-Line
    - LIN
    - MOST
