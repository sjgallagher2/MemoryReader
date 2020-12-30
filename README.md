# Memory Reader

This program is a (sloppy) ROM reader that supports up to 21 address bits and 16-bit data. For the pinout, see the defines in Src/main.h. It's sloppy because it doesn't use a buffer to read memory in, instead it gathers a word and then sends over VCP, then gets the next word. Circular buffer is there for serial input only.