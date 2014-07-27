#! /usr/bin/python
#
# Author: Andrew Zaborowski <andrew.zaborowski@intel.com>
#
# Licensed under the BSD license.

import math
import Image

striplen = 15
stripp0 = ( -32, 48 )
stripp1 = ( -51, 282 )

dist = []
for num in xrange(0, striplen):
	x = stripp0[0] + (stripp1[0] - stripp0[0]) * num * 1.0 / (striplen - 1)
	y = stripp0[1] + (stripp1[1] - stripp0[1]) * num * 1.0 / (striplen - 1)
	dist.append(math.hypot(x, y))

mdist = max(dist)

print('#include <avr/pgmspace.h>')
print('const uint8_t a_d_to_lon_data[] PROGMEM = {')
for d in dist:
	r = d / mdist
	for a in xrange(1, 128 + 1):
		# Cover one quarter only
		angle = math.pi * a / 256
		x = math.sin(angle) * r
		y = math.cos(angle) * r
		lon = math.asin(x) / math.cos(math.asin(y))
		print('\t' + str(int(lon / (math.pi / 2) * 64 + 0.49)) + ',')
print('};')

print('const uint8_t a_d_to_y_data[] PROGMEM = {')
for d in dist:
	r = d / mdist
	for a in xrange(0, 128):
		# Cover one quarter only
		angle = math.pi * a / 256
		y = math.cos(angle) * r
		print('\t' + str(int(y * 32 + 0.49)) + ',')
print('};')

img = Image.open('earth-lands.jpg')
lands = img.load()
print('const uint8_t y_lon_to_land[] PROGMEM = {')
bit = 0
for y in xrange(0, 65):
	sin = (y - 32.0) / 32.0
	lat = math.asin(sin)

	for lon in xrange(0, 256):
		imgy = ((-lat / math.pi) + 0.5) * (img.size[1] - 1)
		imgx = lon / 255.0 * (img.size[0] - 1)
		# TODO filtering
		if bit == 0:
			byte = 0
		if lands[imgx, imgy] < 100:
			byte |= 1 << bit
		bit += 1
		if bit == 8:
			bit = 0;
			print('\t' + str(byte) + ',')
print('};')
