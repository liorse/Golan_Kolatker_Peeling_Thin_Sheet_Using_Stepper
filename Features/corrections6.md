there is a problem with the limit switches.

the problems description is as follow:

when the instrument returns home and the home limit switch is turned on. it remains turned until the motor moves out of home. but the motor can't move because it is on. 

the solution is to detect a rising or a falling edge, whatever is needed and not query the current limit switch position.
