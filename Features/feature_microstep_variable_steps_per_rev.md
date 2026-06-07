I want to add another complexity. As of now I set the stepper driver to microstep at 1600 pulses per revolution. I want this to be a variable that can get the following values (200, 400, 800, 1600, 3200, 6400, 12800, 25600)

at the default (no microstepping) it takes 200 steps for a full revolution. i.e every step 1.8 degrees. in a full revolution the stepper moves linearly 1.5 mm ( that is before taking the angle into consideration - not actual peel transversale) -> in 1600 steps that's 1500um/1600 = 0.9375 um per step. 
I want you to recalculate the step size for the specific number of steps i choose for the motor. 

can you add this steps/revolution parameter to UI?