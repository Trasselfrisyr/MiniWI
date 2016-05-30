# MiniWI
MIDI woodwind controller

2016-04-17

This MIDI controller is running on an Arduino Pro Mini with Atmega328 16MHz/5V. It makes use of breakouts for analog pins A6 and A7, so it needs a Pro Mini version with these breakouts. The pressure sensor used for the project is the Freescale MPX5010GP. Fingering is reverse engineered from Akai EWI and note values are calculated from pressed keys. For details on this, read my guest post on the Gordophone blog. 

http://gordophone.blogspot.se/2016/04/guest-post-alternative-way-of-note.html

Details on the state machine approach for handling the events is also available on Gordon's blog where I borrowed it from. He's got a step-by-step how-to on DIY wind controllers that I highly recommend.

http://gordophone.blogspot.se/2013/01/a-series-basics-of-diy-wind-controllers.html

Hardware notes for connection of MIDI, pressure sensor, keys and joysticks are available in the MiniWI.ino file.

The MIDI routines and the MIDI connections are pretty standard, but I first found them on the midikits.net website, so I'll give Tom Scarff a mention here. Kept his nice MIDI pinout ASCII sketch thing and some comment style too. 

The controller is made to work well with synthesizers using wind controller patches from Patchman Music, with the breath data sent by CC #2 (Breath). For testing I’ve been using their wind controller soundbank for Roland JV-1010.

http://www.patchmanmusic.com

Or you can make your own patches optimized for breath control. Pointers for doing that can also be found on the Patchman website.


Questions and suggestions are welcome. Just send them to johan@helgo.net.

-Johan Berglund


2016-05-17

Variations:

MiniWI.ino is the original proof of concept breadboard version with regular switches.

MiniWI-lite.ino is a simplified version (no joysticks) for my guest post on the Gordophone blog.

MiniWI-cap.ino is the capacitive touch version implemented in my first playable prototype.

MiniWI-cap-pmt.ino is a modification of the touch version to feature portamento (glide) control and a separate potentiometer for setting base octave instead of using sideways motion of joystick to increase range. This reduces instantly playable range but increases the total range of the controller.

MiniVI-cap.ino is a not yet realized EVI version (Electronic Valve Instrument) based on the Akai EVI1000 and the Steiner MIDI EVI.

2016-05-22

I’ve added some pictures from my Instagram account. Check it out for more pictures from this and my other projects.

https://www.instagram.com/trasselfrisyr/

2016-05-30

Project profile for the MiniWI added on Hackaday.io

https://hackaday.io/project/11843-miniwi-woodwind-midi-controller


