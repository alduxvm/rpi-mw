rpi-mw
======

Connection between the MultiWii board and a Raspberry Pie

Small implementation of the MultiWii serial protocol, based on the work by: Drew Brandsen 

Right now, just ask to the MW board for Attitude, and just prints it on the screen, the purpose will be to make it work as a datalogger for systems identification of the multicopter.

It's tested on Mac and on a rpi. After changing the serial port to yours, you must be able to see something like is as output:

<code>
$ python rpi-mw.py</br>
Beginning in 8 seconds...</br>
Serial port is open at /dev/tty.usbserial-AM016WP4</br>
0 0.8 -0.9 -116</br>
0 0.8 -0.9 -116</br>
0 0.8 -0.9 -116</br>
0 0.8 -0.9 -116</br>
0 0.7 -0.9 -116</br>
</code>

Still lots of things to implement.
