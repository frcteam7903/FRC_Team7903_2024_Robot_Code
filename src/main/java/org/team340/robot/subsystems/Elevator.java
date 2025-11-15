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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.PhoenixUtil;
import org.team340.robot.Constants.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive
//import edu.wpi.first.wpilibj.PWMVictorSPX;




@Logged
public final class Elevator extends GRRSubsystem {

    public static enum ElevatorPosition {
        kDown(0.0),
        kIntake(2.0),
        kL1(6.0),
        kL2(9.0),
        kL3(15.0),
        kL4(25.0);

        private final TunableDouble rotations;

        private ElevatorPosition(double rotations) {
            this.rotations = Tunable.doubleValue("elevator/positions/" + name(), rotations);
        }

        public double rotations() {
            return rotations.value();
        }
    }

    private static final TunableDouble kZeroTolerance = Tunable.doubleValue("elevator/kZeroTolerance", 0.15);
    private static final TunableDouble kTunableVoltage = Tunable.doubleValue("elevator/kTunableVoltage", 0.0);

    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private DigitalInput stopSwitch;
    private final StatusSignal<Angle> leadPosition;
    private final StatusSignal<Angle> followPosition;
    private final StatusSignal<AngularVelocity> leadVelocity;
    private final StatusSignal<AngularVelocity> followVelocity;

    private final MotionMagicVoltage positionControl;
    private final VoltageOut voltageControl;
    private final Follower followControl;

    public Elevator() {
        // MOTOR SETUP
        leadMotor = new TalonFX(RobotMap.kElevatorLead);
        followMotor = new TalonFX(RobotMap.kElevatorFollow);
        stopSwitch = new DigitalInput(9);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70.0;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 15.0;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = 1.2;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.Slot0.kG = 0.65;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.136;
        motorConfig.Slot0.kA = 0.003;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 27.5;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        PhoenixUtil.run("Clear Elevator Lead Sticky Faults", () -> leadMotor.clearStickyFaults());
        PhoenixUtil.run("Clear Elevator Follow Sticky Fa ults", () -> followMotor.clearStickyFaults());
        PhoenixUtil.run("Apply Elevator Lead TalonFXConfiguration", () -> leadMotor.getConfigurator().apply(motorConfig)
        );
        PhoenixUtil.run("Apply Elevator Follow TalonFXConfiguration", () ->
            followMotor.getConfigurator().apply(motorConfig)
        );

        leadPosition = leadMotor.getPosition();
        followPosition = followMotor.getPosition();
        leadVelocity = leadMotor.getVelocity();
        followVelocity = followMotor.getVelocity();

        PhoenixUtil.run("Set Elevator Signal Frequencies", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                leadPosition,
                followPosition,
                leadVelocity,
                followVelocity
            )
        );
        PhoenixUtil.run("Set Elevator Signal Frequencies for Following", () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                500,
                leadMotor.getDutyCycle(),
                leadMotor.getMotorVoltage(),
                leadMotor.getTorqueCurrent()
            )
        );
        PhoenixUtil.run("Optimize Elevator CAN Utilization", () ->
            // ParentDevice.optimizeBusUtilizationForAll(20, leadMotor),
            ParentDevice.optimizeBusUtilizationForAll(20, leadMotor, followMotor)
        );

        positionControl = new MotionMagicVoltage(0.0);
        voltageControl = new VoltageOut(0.0);
        followControl = new Follower(leadMotor.getDeviceID(), false);

        PhoenixUtil.run("Set Elevator Follow Motor Control", () -> followMotor.setControl(followControl));

        PhoenixUtil.run("Zero Elevator Lead Position", () -> leadMotor.setPosition(0.0));
        PhoenixUtil.run("Zero Elevator Follow Position", () -> followMotor.setPosition(0.0));

        Tunable.pidController("elevator/pid", leadMotor);
        Tunable.pidController("elevator/pid", followMotor);
        Tunable.motionProfile("elevator/motion", leadMotor);
        Tunable.motionProfile("elevator/motion", followMotor);
    }

   

    @Override
    public void periodic() {
        // BaseStatusSignal.refreshAll(leadPosition, leadVelocity);
        BaseStatusSignal.refreshAll(leadPosition, followPosition, leadVelocity, followVelocity);
    }

    // *************** Helper Functions ***************

    private boolean atLimit() {
        //return stopSwitch.get();
         return !stopSwitch.get();
    }

    /**
     * Gets the elevator's current position, in rotations.
     */
    private double getPosition() {
        // return BaseStatusSignal.getLatencyCompensatedValueAsDouble(leadPosition, leadVelocity);
        return (
            (BaseStatusSignal.getLatencyCompensatedValueAsDouble(leadPosition, leadVelocity) +
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(followPosition, followVelocity)) /
            2.0
        );
    }

    // *************** Commands ***************

    public Command applyTunableVoltage() {
        return commandBuilder("Elevator.applyTunableVoltage()")
            .onExecute(() -> leadMotor.setControl(voltageControl.withOutput(kTunableVoltage.value())))
            .onEnd(leadMotor::stopMotor);
    }

    /**
     * Goes to a position.
     * @param position The position to go to.
     */
    //switch here?
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

                if (atLimit()) {
                    //leadMotor.stopMotor();
                    leadMotor.setPosition(0.0);

                } 
                    if (currentPosition - kZeroTolerance.value() <= 0.0 && target - kZeroTolerance.value() <= 0.0) {
                        leadMotor.stopMotor();
                    } else {
                        leadMotor.setControl(positionControl.withPosition(target + fudge.getAsDouble()));
                    }
                
            })
            .onEnd(leadMotor::stopMotor);
    }
}
