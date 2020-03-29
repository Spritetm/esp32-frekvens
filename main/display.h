
/*
Display driver for Frekvens LED display board.

Note that this driver uses two framebuffers: a front- and a backbuffer. Normally,
the frontbuffer is the buffer being displayed on the LEDs. The backbuffer can be modified
by calling display_setpixel() to change individual pixels. The backbuffer and the frontbuffer
can be swapped by calling display_flip(), at which point all the previous changed by calling
display_setpixel() will become visible.
*/

//Initialize the LED display.
void display_init();

//Set a pixel at (x,y) to a certain value v (0-255) in the backbuffer
void display_setpixel(int x, int y, int v);

//Flip front- and backbuffers
void display_flip();
