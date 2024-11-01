<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->


## How it works

This projects simulates [Conway's Game of Life](https://en.wikipedia.org/wiki/Conway%27s_Game_of_Life) in hardware on a small (32x16) grid with periodic boundary conditions. At each time step, the output of the simulation is printed to an ANSI serial terminal over a serial (UART) interface. The initial state of the board is pseudo-random, generated using a linear-feedback shift register. Single characters received over the serial interface are used to control the simulation, according to the following table:
* `<space>`: start/stop simulation
* `0`: randomize state
* `1`: single-step the simulation

The UART interface of the project is exposed according the the Tiny Tapeout recommended pinout, with `ui_in[3]` used for RX signal and `uo_out[4]` for TX. The UART is configured as 8N1 at 115200 baud, with no flow control.

VGA output of the simulation state is also exposed on the bidirectional pins, that are all configured as outputs and wired to work with a [TinyVGA PMOD](https://github.com/mole99/tiny-vga).

## How to test

Connected the UART interface of the project to any UART terminal, or to an UART-to-USB PMOD or adapter, e.g., the one provided by the onboard RP2040 of the PCB. Configure the serial interface for 8 bits, 1 start bit, no parity bit, 1 stop bit (8N1), with no hardware or software flow control. Open the terminal and type any character: this will bring up a welcome message explaining how to control the simulation.


## External hardware

UART terminal, or UART-to-USB adapter (PMOD or on-board via RP2040). And/or [TinyVGA PMOD](https://github.com/mole99/tiny-vga) for VGA output.


