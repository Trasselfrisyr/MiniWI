# MiniWI
MIDI woodwind controller

2016-04-17

This MIDI controller is running on an Arduino Pro Mini with Atmega328 16MHz/5V. It makes use of breakouts for analog pins A6 and A7, so it needs a Pro Mini version with these breakouts. The pressure sensor used for the project is the Freescale MPX5010GP. Fingering is reverse engineered from Akai EWI and note values are calculated from pressed keys. For details on this, read my guest post on the Gordophone blog. 

http://gordophone.blogspot.se/2016/04/guest-post-alternative-way-of-note.html

Details on the state machine approach for handling the events is also available on Gordon's blog where I borrowed it from. He's got a step-by-step how-to on DIY wind controllers that I highly recommend.

http://gordophone.blogspot.se/2013/01/a-series-basics-of-diy-wind-controllers.html

Hardware notes for connection of MIDI, pressure sensor, keys and joysticks are available in the MiniWI.ino file.

Questions and suggestions are welcome. Just send them to johan@helgo.net.

-Johan Berglund
