package org.team340.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;
import org.team340.robot.subsystems.Elevator.ElevatorPosition;

@Logged
public final class Intake extends GRRSubsystem {

    public static enum IntakeSpeed {
        kStop(0.0),
        kIntake(6.0),
        kBarf(-6.0),
        kScore(12.0);
        
        private final TunableDouble speed;

        private IntakeSpeed(double rotations) {
            this.speed = Tunable.doubleValue("elevator/positions/" + name(), rotations);
        }

        public double speed() {
            return speed.value();
        }
    }

    private static final TunableDouble kZeroTolerance = Tunable.doubleValue("elevator/kZeroTolerance", 0.15);
    private static final TunableDouble kTunableVoltage = Tunable.doubleValue("elevator/kTunableVoltage", 0.0);

    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> leadVelocity;

    private final MotionMagicVoltage positionControl;
    private final VoltageOut voltageControl;

    public Intake() {
        // MOTOR SETUP
        motor = new TalonFX(RobotMap.kIntake);
        // followMotor = new TalonFX(RobotMap.kElevatorFollow);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70.0;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 200.0;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = 1.2;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kG = 0.45;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.136;
        motorConfig.Slot0.kA = 0.003;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 27.5;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;


        PhoenixUtil.run("Clear Elevator Lead Sticky Faults", () -> motor.clearStickyFaults());
        // PhoenixUtil.run("Clear Elevator Follow Sticky Faults", () -> followMotor.clearStickyFaults());
        PhoenixUtil.run("Apply Elevator Lead TalonFXConfiguration", () -> motor.getConfigurator().apply(motorConfig)
        );
        // PhoenixUtil.run("Apply Elevator Follow TalonFXConfiguration", () ->
        //     followMotor.getConfigurator().apply(motorConfig)
        // );

        position = motor.getPosition();
        // followPosition = followMotor.getPosition();
        leadVelocity = motor.getVelocity();
        // followVelocity = followMotor.getVelocity();

        PhoenixUtil.run("Set Elevator Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                position,
                // followPosition,
                leadVelocity
                // followVelocity
            )
        );
        PhoenixUtil.run("Set Elevator Signal Frequencies for Following", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                motor.getDutyCycle(),
                motor.getMotorVoltage(),
                motor.getTorqueCurrent()
            )
        );
        PhoenixUtil.run("Optimize Elevator CAN Utilization", () ->
            ParentDevice.optimizeBusUtilizationForAll(20, motor)
            // ParentDevice.optimizeBusUtilizationForAll(20, leadMotor, followMotor)
        );

        positionControl = new MotionMagicVoltage(0.0);
        voltageControl = new VoltageOut(0.0);
        // followControl = new Follower(leadMotor.getDeviceID(), false);

        // PhoenixUtil.run("Set Elevator Follow Motor Control", () -> followMotor.setControl(followControl));

        PhoenixUtil.run("Zero Elevator Lead Position", () -> motor.setPosition(0.0));
        // PhoenixUtil.run("Zero Elevator Follow Position", () -> followMotor.setPosition(0.0));

        Tunable.pidController("elevator/pid", motor);
        // Tunable.pidController("elevator/pid", followMotor);
        Tunable.motionProfile("elevator/motion", motor);
        // Tunable.motionProfile("elevator/motion", followMotor);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, leadVelocity);
        // BaseStatusSignal.refreshAll(leadPosition, followPosition, leadVelocity, followVelocity);
    }

    // *************** Helper Functions ***************

    /**
     * Gets the elevator's current position, in rotations.
     */
    private double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, leadVelocity);
        // return (
        //     (BaseStatusSignal.getLatencyCompensatedValueAsDouble(leadPosition, leadVelocity) +
        //         BaseStatusSignal.getLatencyCompensatedValueAsDouble(followPosition, followVelocity)) /
        //     2.0
        // );
    }

    // *************** Commands ***************

    public Command applyTunableVoltage() {
        return commandBuilder("Intake.applyTunableVoltage()")
            .onExecute(() -> motor.setControl(voltageControl.withOutput(kTunableVoltage.value())))
            .onEnd(motor::stopMotor);
    }

    public Command setSpeed(IntakeSpeed speed){
        return commandBuilder("Intake.setSpeed()")
            .onExecute(()-> motor.setControl(voltageControl.withOutput(speed.speed())))
            .onEnd(motor::stopMotor);
    }

    /**
     * Goes to a position.
     * @param position The position to go to.
     */
    public Command goTo(ElevatorPosition position) {
        return goTo(() -> position, () -> 0.0);
    }

    /**
     * Goes to a position.
     * @param position The position to go to.
     */
    private Command goTo(Supplier<ElevatorPosition> position, DoubleSupplier fudge) {
        Mutable<Double> holdPosition = new Mutable<>(-1.0);

        return commandBuilder("Elevator.goTo()")
            .onInitialize(() -> holdPosition.value = -1.0)
            .onExecute(() -> {
                double target = position.get().rotations();
                double currentPosition = getPosition();

                if (currentPosition - kZeroTolerance.value() <= 0.0 && target - kZeroTolerance.value() <= 0.0) {
                    motor.stopMotor();
                } else {
                    motor.setControl(positionControl.withPosition(target + fudge.getAsDouble()));
                }
            })
            .onEnd(motor::stopMotor);
    }
}
