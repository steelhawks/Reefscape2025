package org.steelhawks.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.OperatorLock;
import org.steelhawks.subsystems.climb.ClimbConstants.DeepClimbState;
import org.steelhawks.subsystems.climb.deep.DeepClimbIO;
import org.steelhawks.subsystems.climb.deep.DeepClimbIOInputsAutoLogged;
import org.steelhawks.subsystems.climb.shallow.ShallowClimbIO;
import org.steelhawks.subsystems.climb.shallow.ShallowClimbIOInputsAutoLogged;

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {

    private static final double CURRENT_THRESHOLD = 40;

    private final ShallowClimbIOInputsAutoLogged shallowInputs = new ShallowClimbIOInputsAutoLogged();
    private final DeepClimbIOInputsAutoLogged deepInputs = new DeepClimbIOInputsAutoLogged();
    private OperatorLock mOperatorLock = OperatorLock.LOCKED;
    private final ShallowClimbIO shallowIO;
    private final DeepClimbIO deepIO;

    private final ProfiledPIDController mDeepController;
    private final ArmFeedforward mDeepFeedforward;

    private final Alert shallowMotorDisconnected;
    private final Alert topDeepMotorDisconnected;
    private final Alert bottomDeepMotorDisconnected;

    private final Debouncer mDebouncer = new Debouncer(0.005, DebounceType.kBoth);
    private boolean mEnabled = false;

    public void enable() {
        mEnabled = true;
        mDeepController.reset(getDeepPosition());
    }

    public void disable() {
        mEnabled = false;
        runDeepClimb(0, new TrapezoidProfile.State());
    }

    public Climb(ShallowClimbIO shallowIO, DeepClimbIO deepIO) {
        this.shallowIO = shallowIO;
        this.deepIO = deepIO;

        mDeepController =
            new ProfiledPIDController(
                ClimbConstants.DEEP_KP,
                ClimbConstants.DEEP_KI,
                ClimbConstants.DEEP_KD,
                new TrapezoidProfile.Constraints(
                    ClimbConstants.DEEP_MAX_VELO_PER_SECOND,
                    ClimbConstants.DEEP_MAX_ACCEL_PER_SECOND));

        mDeepFeedforward =
            new ArmFeedforward(
                ClimbConstants.DEEP_KS,
                ClimbConstants.DEEP_KG,
                ClimbConstants.DEEP_KV);

        shallowMotorDisconnected =
            new Alert("Shallow Climb Motor is Disconnected", AlertType.kError);
        topDeepMotorDisconnected =
            new Alert("Left Deep Climb Motor is Disconnected", AlertType.kError);
        bottomDeepMotorDisconnected =
            new Alert("Right Deep Climb Motor is Disconnected", AlertType.kError);

        goHome().schedule();
    }

    @Override
    public void periodic() {
//        shallowIO.updateInputs(shallowInputs);
        deepIO.updateInputs(deepInputs);
//        Logger.processInputs("ShallowClimb", shallowInputs);
        Logger.recordOutput("Climb/Enabled", mEnabled);
        Logger.processInputs("Climb", deepInputs);

        shallowMotorDisconnected.set(!shallowInputs.motorConnected);
        topDeepMotorDisconnected.set(!deepInputs.topConnected);
        bottomDeepMotorDisconnected.set(!deepInputs.bottomConnected);

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Climb/CurrentCommand", getCurrentCommand().getName());
        }

        // stop adding up pid error while disabled
        if (DriverStation.isDisabled()) {
            mDeepController.reset(getDeepPosition());
        }

        if (mEnabled) {
            runDeepClimb(mDeepController.calculate(getDeepPosition()), mDeepController.getSetpoint());
        }
    }

    private void runDeepClimb(double output, TrapezoidProfile.State setpoint) {
        double volts = output + mDeepFeedforward.calculate(setpoint.position, setpoint.velocity);
        deepIO.runClimb(volts);
    }

     @AutoLogOutput(key = "DeepClimb/AdjustedPosition")
     private double getDeepPosition() {
         return deepInputs.encoderPositionRad;
     }

    /* ------------- Shallow Climb Commands ------------- */

    public Command shallowClimbCommandWithCurrent() {
        return runShallowClimbViaSpeed(0.2)
            .until(() -> mDebouncer.calculate(shallowInputs.climbCurrentAmps > CURRENT_THRESHOLD));
    }

    public Command shallowHomeCommandWithCurrent() {
        return runShallowClimbViaSpeed(-0.2)
            .until(() -> mDebouncer.calculate(shallowInputs.climbCurrentAmps > CURRENT_THRESHOLD));
    }

    public Command runShallowClimbViaSpeed(double speed) {
        return Commands.run(
            () -> {
                shallowIO.runClimbViaSpeed(speed);
            }, this)
            .finallyDo(() -> shallowIO.stop());
    }

    public Command runShallowClimbViaVolts(double volts) {
        return Commands.run(
            () -> {
                shallowIO.runClimbViaVolts(volts);
            }, this)
            .finallyDo(() -> shallowIO.stop());
    }

    public Command runShallowClimb() {
        return shallowClimbCommandWithCurrent()
            .andThen(runShallowClimbViaVolts(-1));
    }

    /* ------------- Deep Climb Commands ------------- */

    public Command toggleManualControl(DoubleSupplier joystickAxis) {
        return Commands.runOnce(
            () -> {
                Logger.recordOutput("Climb/RequestedClimbSpeed", joystickAxis.getAsDouble());

                if (mOperatorLock == OperatorLock.LOCKED) {
                    disable();
                    setDefaultCommand(
                        deepClimbManual(
                            () -> MathUtil.clamp(
                                MathUtil.applyDeadband(joystickAxis.getAsDouble(), Constants.Deadbands.ELEVATOR_DEADBAND),
                                -0.5,
                                0.5)));
                    mOperatorLock = OperatorLock.UNLOCKED;
                } else {
                    if (getDefaultCommand() != null) {
                        getDefaultCommand().cancel();
                        removeDefaultCommand();
                    }
                    setDesiredState(DeepClimbState.HOME).schedule();
                    enable();
                    mOperatorLock = OperatorLock.LOCKED;
                }

                Logger.recordOutput("Climb/IsLocked", mOperatorLock == OperatorLock.LOCKED);
            }, this)
        .withName("Toggle Manual Control");
    }

    private Command deepClimbManual(DoubleSupplier speed) {
        final double kG = 0.0;

        return Commands.runOnce(this::disable, this)
            .andThen(
                Commands.run(
                    () -> {
                        double appliedSpeed = speed.getAsDouble();

                        if (appliedSpeed == 0.0) {
                            appliedSpeed = Math.cos(getDeepPosition()) * kG / 12.0;
                        }

                        Logger.recordOutput("Elevator/ManualAppliedSpeed", appliedSpeed);
                        deepIO.runClimbViaSpeed(appliedSpeed);
                    }, this))
            .finallyDo(deepIO::stop)
            .withName("Manual Climb Pivot");
    }

    public Command setDesiredState(DeepClimbState state) {
        return Commands.runOnce(
            () -> {
                double goal = state.getRadians();
                deepInputs.goal = goal;
                mDeepController.setGoal(goal);
            }, this);
    }

    public Command runDeepClimbViaSpeed(double speed) {
        return Commands.run(
            () -> deepIO.runClimbViaSpeed(speed))
        .finallyDo(deepIO::stop);
    }

    public Command runDeepClimb(double volts) {
        return Commands.run(
            () -> deepIO.runClimb(volts))
        .finallyDo(deepIO::stop);
    }

    public Command prepareDeepClimb() {
        return Commands.runOnce(this::enable)
            .andThen(setDesiredState(DeepClimbState.PREPARE))
            .ignoringDisable(true);
    }

    public Command goHome() {
        return Commands.runOnce(this::enable)
            .andThen(setDesiredState(DeepClimbState.HOME))
            .ignoringDisable(true);
    }
}
