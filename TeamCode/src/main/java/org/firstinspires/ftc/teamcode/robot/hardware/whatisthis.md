basically this folder simply contains classes for hardware devices which implement cached writing

as stated by Tarun (not the one from our team) in the FTC discord:
"it’s pretty simple, just make a wrapper class for your motors/servos/LEDs that stores a value 
every time you set one, and if you try to set one which is the same as the last, it just doesn’t
do anything"