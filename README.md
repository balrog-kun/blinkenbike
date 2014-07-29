This Arduino sketch drives WS2811 RGB LED strips on bicycle wheel to
produce a Persistence of Vision effect.  The components and
electrical wiring are explained in
http://www.instructables.com/id/Bike-wheel-WS2811-LED-effects-with-Arduino/

There are a few different LED programs in the code and you can add
new ones by editing wheel.ino.  All of them are always included in the
firmware and (in theory) can be switched in runtime.

Switching between these programs is done by detecting when the rider is
braking.  No additional sensors are necessary beyond the inercial sensors
used for reading the wheel position.  A sequence of three short brakes
(periods between 100ms - 400ms) followed by a long pause (1 - 3s) and then
another brake causes the sketch to skip to the next program, while two
short brakes with a long pause go to the previous program.  This is still
work in progress, it should work when the bike is rolling on a relatively
even surface and the braking is quite sharp.  On uneven surface or when
riding really fast and the surface is not perfectly flat it'll probably
fail to detect anything but noise.  There may be false positives too.  It
should be easy to disable this mechanism in the code if desired.

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
every 4 seconds.
