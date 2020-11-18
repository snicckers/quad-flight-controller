# quad-flight-controller
 
My own rolling quadcopter flight controller project.

Right now its self-flying capabilities are limited, it is capable of automatically levelling itself with the ground or a desired setpoint on the roll or pitch axes. 

It uses my own implementation of a Madgwick Filter for the intertial measurement unit. For now, it only uses a gyroscope and an accelerometer to derive the attitude, which means its yaw orientation cannot be trusted as it will drift over time (I plan to amend this in the near future). The program calibrates itself at startup to remove gyroscope error, and has a continous calibration cycle to adjust for temperature sensitivity on the gyroscope. 

Hardware:
* Arduino Due (you could also use an Uno)
* Flysky FS-R6B Transmitter & Reciever 
* 4-off 30-amp ESCs
* 4-off BR2212 Brushless Motors
* Some 10Kohm resistors & a diode

Future plans:
* Add magnetometer & change  - allows for accurate yaw orientation readings
* Add barometer & change control loop - allows for automatic control of quadcopter altitude 
* Add GPS and a cascade control loop - allows for automatic x & y correction (hold position, move to given position)
* Some kind of wifi or Bluetooth connection, control craft position and altitue with laptop or smartphone
