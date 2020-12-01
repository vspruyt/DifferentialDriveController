# DifferentialDriveController
This repo contains closed loop steering and speed control.
For Steering control, a PID controller tries to achieve a certain speed difference between both wheels. This makes sure that the robot can drive in a straight line, even if the motors have slightly different characteristics (i.e. motor1 needs a bit more voltage than motor 2 to get to the same speed).

To deal with the deadband (the region where an input voltage does not result in any motor action), I linearly interpolate the expected PID input, given a certain setpoint, and feed that input to the PID controller instead of the real encoder measurements. This allows smooth operation and transition from normal mode to deadband mode. When inside the deadband, the PID parameters are set to more conservative values.

I'm currently using these planetary gear motors with encoders:
https://www.servocity.com/313-rpm-hd-premium-planetary-gear-motor-w-encoder/
