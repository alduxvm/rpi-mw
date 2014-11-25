# rpi-mw

***
IMPORTANT: use rpi-mw-legacy.rpi, the other code has lots of bugs and I stopped working on it in order to rewrite the code in a more simplistic an efficent way.
***

Connection between the MultiWii board and computer via serial communication... I'm using a raspberry pie to read data onboard a multicopter.

Small implementation of the MultiWii serial protocol, based on the work by: Drew Brandsen 

Right now, just ask to the MW board for the data needed, and just prints it on the screen or saves it to a file in CSV like format, the purpose will be to make it work as a datalogger for systems identification of the multicopter.


## To configure:

- Use the ```drone``` class in order to configure what you want. (comments pretty selfexplainatory)
- Configure your serial port

... and thats it!

### Tests

It's tested on Mac and on a rpi. After changing the serial port to yours, you must be able to see something like is as output:

```
$ python rpi-mw-legacy.py
Beginning Multiwii - wait 14 seconds...
0.14 0.14 1500.0 1500.0 1500.0 1500.0 0.0 -19.0 515.0 0.0 0.0 0.0 -50.0 -194.0 465.0
0.048 0.188 1500.0 1500.0 1500.0 1500.0 -3.0 -18.0 515.0 0.0 1.0 1.0 -50.0 -194.0 465.0
0.045 0.233 1500.0 1500.0 1500.0 1500.0 2.0 -14.0 513.0 -1.0 -1.0 0.0 -49.0 -193.0 465.0
0.048 0.281 1500.0 1500.0 1500.0 1500.0 3.0 -15.0 510.0 0.0 -1.0 0.0 -49.0 -193.0 465.0
0.048 0.329 1500.0 1500.0 1500.0 1500.0 0.0 -15.0 514.0 1.0 0.0 -1.0 -49.0 -195.0 466.0
0.05 0.379 1500.0 1500.0 1500.0 1500.0 -2.0 -16.0 511.0 -1.0 1.0 0.0 -49.0 -195.0 466.0
0.046 0.425 1500.0 1500.0 1500.0 1500.0 -1.0 -16.0 514.0 0.0 0.0 0.0 -50.0 -194.0 465.0
0.046 0.472 1500.0 1500.0 1500.0 1500.0 0.0 -17.0 515.0 0.0 0.0 0.0 -50.0 -194.0 465.0
0.048 0.52 1500.0 1500.0 1500.0 1500.0 1.0 -17.0 517.0 1.0 1.0 0.0 -50.0 -195.0 466.0
0.045 0.565 1500.0 1500.0 1500.0 1500.0 -2.0 -19.0 515.0 0.0 0.0 0.0 -50.0 -195.0 466.0
0.048 0.613 1500.0 1500.0 1500.0 1500.0 1.0 -18.0 513.0 1.0 -1.0 1.0 -49.0 -195.0 467.0
0.046 0.659 1500.0 1500.0 1500.0 1500.0 -1.0 -15.0 513.0 -1.0 0.0 0.0 -49.0 -195.0 467.0
0.05 0.709 1500.0 1500.0 1500.0 1500.0 -2.0 -16.0 520.0 0.0 0.0 0.0 -49.0 -193.0 466.0
0.045 0.754 1500.0 1500.0 1500.0 1500.0 -2.0 -16.0 512.0 0.0 1.0 0.0 -49.0 -193.0 466.0
0.046 0.8 1500.0 1500.0 1500.0 1500.0 -1.0 -18.0 511.0 0.0 -1.0 -1.0 -49.0 -193.0 466.0
0.045 0.845 1500.0 1500.0 1500.0 1500.0 1.0 -16.0 515.0 -1.0 0.0 -1.0 -51.0 -193.0 465.0
0.045 0.89 1500.0 1500.0 1500.0 1500.0 0.0 -16.0 510.0 0.0 0.0 0.0 -51.0 -193.0 465.0
0.046 0.936 1500.0 1500.0 1500.0 1500.0 -1.0 -15.0 512.0 -2.0 0.0 0.0 -49.0 -194.0 465.0
0.047 0.983 1500.0 1500.0 1500.0 1500.0 0.0 -16.0 519.0 0.0 0.0 0.0 -49.0 -194.0 465.0
0.047 1.03 1500.0 1500.0 1500.0 1500.0 -4.0 -15.0 516.0 0.0 1.0 0.0 -51.0 -194.0 466.0
0.045 1.075 1500.0 1500.0 1500.0 1500.0 -4.0 -15.0 518.0 0.0 0.0 0.0 -51.0 -194.0 466.0
0.046 1.122 1500.0 1500.0 1500.0 1500.0 -2.0 -17.0 517.0 1.0 0.0 1.0 -49.0 -195.0 465.0
0.048 1.17 1500.0 1500.0 1500.0 1500.0 1.0 -19.0 515.0 0.0 0.0 -1.0 -49.0 -195.0 465.0
0.049 1.219 1500.0 1500.0 1500.0 1500.0 -1.0 -21.0 511.0 0.0 0.0 -1.0 -49.0 -195.0 465.0
0.045 1.264 1500.0 1500.0 1500.0 1500.0 -3.0 -16.0 512.0 0.0 1.0 0.0 -49.0 -195.0 466.0
0.047 1.311 1500.0 1500.0 1500.0 1500.0 -1.0 -17.0 516.0 1.0 0.0 0.0 -49.0 -195.0 466.0
0.048 1.359 1500.0 1500.0 1500.0 1500.0 0.0 -18.0 513.0 0.0 1.0 0.0 -49.0 -195.0 465.0
0.048 1.407 1500.0 1500.0 1500.0 1500.0 3.0 -14.0 515.0 0.0 0.0 -1.0 -49.0 -195.0 465.0
0.051 1.458 1500.0 1500.0 1500.0 1500.0 -3.0 -17.0 519.0 0.0 0.0 0.0 -50.0 -194.0 465.0
0.048 1.507 1500.0 1500.0 1500.0 1500.0 -1.0 -21.0 517.0 1.0 1.0 0.0 -50.0 -194.0 465.0
0.047 1.553 1500.0 1500.0 1500.0 1500.0 0.0 -16.0 509.0 0.0 0.0 0.0 -50.0 -195.0 467.0
```

The above data is 4 channels of the radio (roll, pitch, yaw, throtlle) and the raw IMU, so, (ax,ay,az,gx,gy,gz,mx,my,mz), the first values are the time in each loop, and the overall time.

## Conclusions

This code is able to do 50hz when asking for one command.

It also contains the stuff needed for asking data to a UDP server, in my case the UDP server is a motion capture system. 

