# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a heavily-commented example codebase for FIRST Robotics Competition (FRC) robots using WPILib and the Command-Based framework. It demonstrates common subsystems and patterns gathered from multiple years of competition experience.

## Build System & Commands

This project uses Gradle with the GradleRIO plugin (version 2025.3.2) for FRC development.

### Building and Testing
```bash
./gradlew build           # Build the project
./gradlew test            # Run JUnit 5 tests
./gradlew jar             # Create deployable JAR
```

### Deployment and Simulation
```bash
./gradlew deploy          # Deploy to RoboRIO (requires team number in .wpilib/wpilib_preferences.json)
./gradlew simulateJava    # Run robot code in simulation with GUI
```

### Code Requirements
- Java 17 (sourceCompatibility and targetCompatibility)
- Main class: `frc.robot.Main`
- Tests use JUnit 5 with auto-detection enabled

## Architecture

### Core Structure
- **Robot.java**: Standard WPILib `TimedRobot` that runs the CommandScheduler
- **RobotContainer.java**: Container for subsystems, commands, and button bindings
- **Main.java**: Entry point for the robot program

### Package Organization
```
frc.robot/
├── commands/          # Command implementations organized by subsystem
│   ├── elevator/
│   ├── intake/
│   ├── swerve/
│   └── vision/
├── subsystems/        # Subsystem implementations
│   ├── auto/          # Autonomous mode selection and management
│   ├── elevator/      # Position-based control with motion profiles
│   ├── intake/        # Velocity-based flywheel control
│   ├── strap/         # Open-loop with stall detection
│   ├── swerve/        # Swerve drive with simulation
│   └── vision/        # Limelight integration
├── testbots/          # Test configurations
└── util/              # Shared utilities and abstractions
```

### Key Subsystems

**Autonomous (`subsystems.auto`)**
- Uses REV Digit Board for on-field program selection (not dashboard-dependent)
- Lazy-loads autonomous commands (doesn't create all Command objects at startup)
- Falls back to `DashboardPicker` in simulation, `DigitBoardPicker` on real robot

**Elevator (`subsystems.elevator` + `commands.elevator`)**
- Position-based control with feedforward and feedback
- Uses trapezoidal motion profiles for smooth movement to preset positions
- Multiple safety limits (min/max height, velocity capping, feedback capping)
- Implements "mousetrapping" prevention to avoid sudden snaps when re-enabled
- Includes simulation support via `ElevatorMotorSim`
- Pattern applies equally to arms (just different feedforward equation and potential angle wraparound)

**Intake (`subsystems.intake` + `commands.intake`)**
- Velocity-based flywheel control
- Supports control via motor RPM, wheel RPM, or linear speed (feet/second)
- Simpler safety model than elevator (spinning wheels are safer at max voltage)

**Strap (`subsystems.strap`)**
- Open-loop control with stall detection
- Uses velocity monitoring with debouncing to detect when motor has stalled
- Useful pattern for climbers or other mechanisms that run until physical stop

**Swerve (`subsystems.swerve` + `commands.swerve`)**
- Hardware-agnostic implementation (kinematics, optimization, etc. as configuration)
- Publishes poses to AdvantageScope for visualization during debugging
- Includes `SwerveChassisSim` for testing command logic without hardware
- Supports rotating around objects (not heavily used but easy to implement)

**Vision (`subsystems.vision`)**
- Limelight integration with helpers
- Target tracking and pose estimation

### Utilities (`util`)

**Motor Interface**
- Generic abstraction allowing subsystem development and debugging without hardware

**Util Class**
- Math, configuration, and logging helper functions used across subsystems

## Development Patterns

### Subsystem Debugging
Many subsystems set a `currentCommand` text field that displays on the dashboard, making it easy to see which command is currently controlling each subsystem.

### Simulation-First Development
Multiple subsystems include simulation support (`SwerveChassisSim`, `ElevatorMotorSim`, `LimelightSim`). Testing in simulation is much faster than deploy-test-fix cycles on physical hardware.

### Safety Limits
Position-based subsystems (elevator, arm) need multiple layers of safety:
- Hard min/max position limits
- Velocity limits to prevent overshooting bounds
- Feedback capping to prevent aggressive corrections
- Teleop input capping

Velocity-based subsystems (intake, shooter) are generally safer at full voltage since they lack hard stops.

### Configuration vs. Implementation
Common patterns (kinematics, state optimization, motion profiles) are abstracted with hardware details as configuration only, making year-to-year reuse easier.

### Autonomous Selection
Don't rely on dashboard for autonomous selection in competition (crashes happen). Use physical selector like REV Digit Board. Don't create all autonomous Command objects at startup - lazy-load them to avoid slow initialization.
