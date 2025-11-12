# Utility Code

Contains some useful stuff that we wind up writing over and over again.

* Generic interface for a `Motor` which lets us write subsystems that we can debug without hardware


* A subclass of `PIDController` that gets its configuration from `DoubleSupplier` objects so it can be updated without redeploying code, and includes the option for capping feedback


* Lots of sundry math, configuration and logging functions (see `Util`).