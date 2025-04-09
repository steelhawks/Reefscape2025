package org.steelhawks.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {

    public enum ClimbingState {
        IDLE, CLIMBING
    }

    private ClimbingState mClimbingState = ClimbingState.IDLE;
    private final DeepClimbIOInputsAutoLogged deepInputs = new DeepClimbIOInputsAutoLogged();
    private OperatorLock mOperatorLock = OperatorLock.LOCKED;
    private final DeepClimbIO deepIO;

    private final ProfiledPIDController mDeepController;
    private final ArmFeedforward mDeepFeedforward;
    private final ArmFeedforward mClimbingDeepFeedforward;

    private final Alert topDeepMotorDisconnected;

    private boolean mEnabled = false;

    public void enable() {
        mEnabled = true;
        mDeepController.reset(getPosition());
    }

    public void disable() {
        mEnabled = false;
        runDeepClimb(0, new TrapezoidProfile.State());
    }

    public Climb(DeepClimbIO deepIO) {
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
        mClimbingDeepFeedforward =
            new ArmFeedforward(
                ClimbConstants.CLIMBING_DEEP_KS,
                ClimbConstants.CLIMBING_DEEP_KG,
                ClimbConstants.CLIMBING_DEEP_KV);

        topDeepMotorDisconnected =
            new Alert("Left Deep Climb Motor is Disconnected", AlertType.kError);

        goHome().schedule();
    }

    @Override
    public void periodic() {
        deepIO.updateInputs(deepInputs);
        Logger.recordOutput("Climb/Enabled", mEnabled);
        Logger.processInputs("Climb", deepInputs);

        topDeepMotorDisconnected.set(!deepInputs.connected);

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Climb/CurrentCommand", getCurrentCommand().getName());
        }

        // stop adding up pid error while disabled
        if (DriverStation.isDisabled()) {
            mDeepController.reset(getPosition());
        }

        if (mEnabled) {
            if (mClimbingState == ClimbingState.IDLE) {
                mDeepController.setPID(
                    ClimbConstants.DEEP_KP,
                    ClimbConstants.DEEP_KI,
                    ClimbConstants.DEEP_KD);
            } else {
                mDeepController.setPID(
                    ClimbConstants.CLIMBING_DEEP_KP,
                    ClimbConstants.CLIMBING_DEEP_KI,
                    ClimbConstants.CLIMBING_DEEP_KD);
            }

            runDeepClimb(mDeepController.calculate(getPosition()), mDeepController.getSetpoint());
        }
    }

    private void runDeepClimb(double output, TrapezoidProfile.State setpoint) {
        double ff = mClimbingState == ClimbingState.IDLE
            ? mDeepFeedforward.calculate(setpoint.position, setpoint.velocity)
            : mClimbingDeepFeedforward.calculate(setpoint.position, setpoint.velocity);
        double volts = output + ff;
        deepIO.runClimb(volts);
    }

     @AutoLogOutput(key = "DeepClimb/AdjustedPosition")
     public double getPosition() {
         return deepInputs.encoderAbsolutePositionRad;
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
                    goHome().schedule();
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
                            appliedSpeed = (Math.cos(getPosition()) * kG) / 12.0;
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
                double goal = state.getAngle().getRadians();
                deepInputs.goal = goal;
                mDeepController.setGoal(goal);
            }, this)
            .ignoringDisable(true);
    }

    public Command runDeepClimbViaSpeed(double speed) {
        return Commands.run(
            () -> deepIO.runClimbViaSpeed(speed), this)
        .finallyDo(deepIO::stop);
    }

    public Command runDeepClimb(double volts) {
        return Commands.run(
            () -> deepIO.runClimb(volts), this)
        .finallyDo(deepIO::stop);
    }

    public Command prepareDeepClimb() {
        return Commands.runOnce(this::enable)
            .andThen(setDesiredState(DeepClimbState.PREPARE));
    }

    public Command goHome() {
        return Commands.runOnce(this::enable)
            .andThen(setDesiredState(DeepClimbState.HOME));
    }
}
