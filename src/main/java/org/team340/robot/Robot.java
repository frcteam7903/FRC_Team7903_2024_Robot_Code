package org.team340.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team340.lib.util.Profiler;
import org.team340.lib.util.Tunable;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Elevator;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.subsystems.Elevator.ElevatorPosition;
import org.team340.robot.subsystems.Intake.IntakeSpeed;
import org.team340.robot.subsystems.Intake;

@Logged
public final class Robot extends TimedRobot {

    public final Swerve swerve;
    public final Elevator elevator;
    public final Intake intake;

    public final Routines routines;

    private final CommandXboxController driver;
    private final CommandXboxController aux; 

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SignalLogger.enableAutoLogging(false);
        Epilogue.getConfig().root = "/Telemetry";

        // Initialize subsystems
        swerve = new Swerve();
        elevator = new Elevator();
        intake = new Intake();

        // Initialize compositions
        routines = new Routines(this);

        // Initialize controllers
        driver = new CommandXboxController(Constants.kDriver);
        aux = new CommandXboxController(Constants.kCoDriver);

        // Set default commands
        elevator.setDefaultCommand(elevator.goTo(ElevatorPosition.kDown));
        swerve.setDefaultCommand(
            swerve.drive(
                driver::getLeftX,
                driver::getLeftY,
                () -> driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()
            )
        );

        // Create triggers
        RobotModeTriggers.autonomous().whileTrue(swerve.drive(() -> 0.0, () -> 0.3, () -> 0.0).withTimeout(1.0));

        // Driver bindings
        driver.povLeft().onTrue(swerve.tareRotation());
        aux.a().whileTrue(elevator.goTo(ElevatorPosition.kL1));
        aux.b().whileTrue(elevator.goTo(ElevatorPosition.kL2));
        aux.x().whileTrue(elevator.goTo(ElevatorPosition.kL3));
        aux.y().whileTrue(elevator.goTo(ElevatorPosition.kL4));
        aux.leftBumper().whileTrue(intake.setSpeed(IntakeSpeed.kIntake));
        aux.rightBumper().whileTrue(intake.setSpeed(IntakeSpeed.kBarf));

        // Set thread priority
        Threads.setCurrentThreadPriority(true, 10);
    }

    @Override
    public void robotPeriodic() {
        Profiler.start("RobotPeriodic");
        Profiler.run("CommandScheduler", () -> CommandScheduler.getInstance().run());
        Profiler.run("Epilogue", () -> Epilogue.update(this));
        Profiler.run("Tunables", Tunable::update);
        Profiler.end();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}
}
