# Java coding standards

## Layout

Follow the standard FRC Java package conventions:
* `frc.robot` for Robot and Config classes
* `frc.robot.util` for shared utility code
* Each subsystem gets their own subpackage under
    * `frc.robot.subsystems` for the subsystem itself
    * `frc.robot.commands` for top-level single-subsystem commands

## Whitespace

* Use four spaces instead of tabs
* Try to collapse whitespace within methods
* Use whitespace liberally between methods to improve readability
* Use comment separators between "sections" in longer code files

## Capitalization

* Use `UpperCamelCase` for
    * Class names
    * Metric names in SmartDashboard to identify source
    * Configuration property names
    * Examples:
        * `public class ElevatorSubsystem`
        * `SmartDashboard.putNumber("ElevatorSubsystem/HeightCurrent")`
        * `Preferences.initDouble("ElevMaxHeight", 1.0)`


* Use `lowerCamelCase` for
    * Instance and local variables
    * Method names and parameters
    * Constants
    * Examples:
        * `public void calculateFeedback()`
        * `double targetPosition`
        * `public static final maxVolts = 12.0`

## Access modifiers

* Top-level classes are always `public`
* Inner classes have no modifier unless they need to be `public`
* Do not use access modifiers (`public`/`private`) on instance variables

* Make instance variables `final` where possible
    * Example: `final GenericMotor motor;`

## Naming and subclassing

* Subsystem classes
    * Are named according to their function
    * Have names ending in `Subsystem`
    * Extend from WPILib `SubsystemBase`
    * Example:
        * `public class ShooterSubsystem extends SubsystemBase`


* Command classes
    * Are named according to their function
    * Are prefixed with their subsystem if not inner classes
    * Have names ending in `Command`
    * Extend from WPILib `Command`
    * Example:
        * `public class SwerveTeleopCommand extends Command`

## Comments and documentation

- Do not copy comments on conventions & learning from backup classes into generated code
- Use class-level Javadoc comments to explain specific purpose of subsystems/commands
- Use method-level Javadoc comments on public methods
- Mention units in comments (feet, inches, degrees, etc.)
- Use inline comments sparingly to explain specific unintuitive patterns

