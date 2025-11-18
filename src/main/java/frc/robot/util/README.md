# Utility Code

Contains some useful stuff that we wind up writing over and over again. A few learnings that are embedded here ...

*Vendor support for simulation is minimal*. Being able to test code in the simulator is MUCH faster than needing time with the real robot (which might be getting built). But vendors don't always support realistic simulation of motors. The `Motor` interface here lets us write simulated versions of subsystems. 

*The DigitBoard is awesome*. We used a [REV Digit Board](https://www.revrobotics.com/rev-11-1113/) to allow an on-field operator to
pick which autonomous program to run. It's great!

*There are lots of opportunities for libraries*. We wound up writing a lot
of the same functions year over year. Some of the most useful ones show up
in the `Util` class.
