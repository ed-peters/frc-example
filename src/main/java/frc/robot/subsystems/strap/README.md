# Strap Subsystem

This is essentially the "climbing strap" from our 2025 Reefscape robot. It shows an occasionally useful trick, which is to read motor velocity and run a motor until it stalls. The basic logic is "if we're applying power, and it's not moving, stop it". 

But you probably don't want to wait until the speed is exactly 0 - this may wind up burning out your motor while you wait. Instead, you want "if we're applying power, and it's moving really slow, stop it".

But then you have a new problem - motors don't start moving instantaneously. So you will be a little slow at the beginning of the command. The solution is "if we've been applying power for a little while, and it's moving really slow, stop it".

See the implementation for more comments, and notes about using the WPILib `Debouncer`.