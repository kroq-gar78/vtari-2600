
# Atari 2600 emulator

A semi-functional Atari 2600 emulator.

It is not entirely accurate (e.g. cycle counts are a little off), and there are no games that I know of are completely playable.


## What works

However, there are a collection of simple tests that _do_ work (tests/kernel*.bin).

The emulator does render graphics to the screen.

Games:

 * Rubik's Cube simulator: renders parts of the cube, and does the initial scramble

 * Space Invaders: the player and some of the aliens render and move

 * Pac-Man: the playfield and all the sprites render, albeit with the sprites in the wrong places


## Compile and run

On Ubuntu/Debian, the following packages are needed:

 * `libsdl2-dev`
 * `libsdl2-ttf-dev`

Running `make all` will produce the executable `cpu`.

    To run an Atari 2600 binary, simply run 

```
./cpu <filename> >/dev/null
```

(Warning: there is a lot of debug output, so make sure to include `>/dev/null`).

There are example tests in `tests/` from a website (cited in `links.txt`) that do show simple things (e.g. color test, sprites, moving sprites).

To exit, press `ESC`
To move in a direction, use arrow keys.
To fire, press Space (you can't see the effect).
To reset the game press `z`.
To select the game press `x`.

You will need to reset (and possibly select) to start Space Invaders.
Movement is (strangely) reversed in this game.

## Structure

The entrypoint of the program is `main.c`.
The CPU emulator (for the MOS 6502/7) is in `cpu.c`. (It has all the instructions, which is why it's so long.)
The handler for memory (the PIA and RIOT (RAM)) is in `mem.c`, and `mem_set` and `mem_get8` handle almost all accesses to memory.
TIA emulation is found in `tia.c`, but is accessed through `mem.c`, due to memory-mapped registers.
In the original console, this chip handled rendering graphics to the screen.
I essentially simulated the scanning electron beam of the TV in this module.
Keyboard input is also handled here.

One part I enjoyed was generating skeleton code for all of the ~150 opcodes.
I parsed the table using regex's and split the table into two: one with the base instruction (e.g. ADC), and one with the addressing mode (e.g. absolute).
This allowed me to cut the number of functions by at least half.
In the instruction skeletons (e.g. `inst_adc`), I determined the addressing mode and fetched 8 bits *and* 16 bits at the given address, even if I didn't need to.
This allowed me to quickly implement each instruction, letting me focus on the instruction logic instead of writing skeleton code.

All foreign sources of code are either documented in `links.txt`, or in the relevant section of code.

