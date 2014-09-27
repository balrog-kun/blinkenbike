Description
===========

This Arduino sketch drives WS2811 (and similar) RGB LED strips on
bicycle wheel to produce a Persistence of Vision effect.  The
components and electrical wiring are explained in
http://www.instructables.com/id/Bike-wheel-WS2811-LED-effects-with-Arduino/

There are a few different LED programs in the code and you can add
new ones by editing wheel.ino.  All of them are always included in the
firmware and (in theory) can be switched in runtime.

The code right now recognises to type of inputs from the user to allow
switching between these programs, but neither is working perfectly.
Method one is based on decting sequences of braking that the rider can
issue on the go.  Current implementation turns out to be quite poor and
difficult to use.  Method two tries to detect "tapping" on the handlebar
laterally.  Basically the rider needs to punch the handlebar on a side
in a short sequence, this works quite reliably but your hand may hurt
because you need to hit quite strong in the current implementation.
No additional sensors are necessary to do that beyond the inertial sensors
used for reading the wheel position.  See "Control" below for details.

After you power the device up, the first fraction of a second the sketch
tries to collect gyroscope calibration data and assumes the wheel is
stationary for that period, so try to keep it still for a short moment when
starting the code.

The default and simplest effect is to illuminate one half of the wheel
as the wheel is spinning.  Basically each LED lights up when it passes
a line dividing the wheel "disc" in two halves, and then turns off when
passing that line again.  This actually creates and a very nice effect
and catches many other bike rider's attention when riding in a group.

There's also basic support for displaying text (default text is
"Bicicritica") using the font data from the Linux kernel framebuffer
-- probably the same font embedded in VGA cards for many years.

Program number 4 is all LEDs off and 5 all LEDs on all the time.

Program number 7 is a spinning globe, currently only shows the continent
shapes in solid colours (land = green, ocean = dark blue, white =
antarctica).  The globe rotates around the Earth's axis about one turn
every 4 seconds, in a pseudo-3D that is not very well visible.

Control
=======

The brakes based method tries to detect two types of signals.  A sequence
of two short brakes (periods between 100ms - 400ms) followed by a long
pause (1 - 3s) and then another brake causes the sketch to skip to the next
program, while three short brakes with a long pause goes to the previous
program.  To do this correctly it seems that some training is necessary to
get the timings right for the brakes to be detected and properly
distinguished from the various types of noise present in what comes in from
the sensors.  It will work better when the bike is rolling on a relatively
even surface and the braking is quite sharp.  Going too fast will also make
the braking get lost in all the noise.  A full reimplementation may be
necessary to make this method usable, the current approach may be hopeless.

The "tap" method detects two sequences:  three "taps" on the handlebar
200ms - 400ms periods apart followed by a 1s to 3s-long pause and another
"tap" (i.e. 3 + pause + 1) switch to the next program, while 4 + pause + 1
switch one program back.  Lateral acceleration is quite easy to detect
because most noise when riding is in the other two axis, however the
current naive implementation requires quite strong punches to avoid
false positives (which may still happen anyway).  This can still be
improved on with little effort in the code.

Both methods can be disabled in the code if not needed, to avoid false
positives.
