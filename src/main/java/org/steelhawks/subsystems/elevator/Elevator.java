package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Clearances;
import org.steelhawks.Constants.Deadbands;
import org.steelhawks.OperatorLock;
import org.steelhawks.util.AlertUtil;

import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Volts;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private OperatorLock mOperatorLock;
    private final SysIdRoutine mSysId;
    private boolean shouldEStop = false;
    private boolean mEnabled = false;
    private final ElevatorIO io;

    private final ProfiledPIDController mController;
    private final ElevatorFeedforward mFeedforward;

    private final Alert leftMotorDisconnected =
        new AlertUtil("Left Elevator Motor Disconnected", AlertType.kError)
            .withCondition(() -> !inputs.leftConnected);
    private final Alert rightMotorDisconnected =
        new AlertUtil("Right Elevator Motor Disconnected", AlertType.kError)
            .withCondition(() -> !inputs.rightConnected);
    private final Alert canCoderDisconnected =
        new AlertUtil("Elevator CANcoder Disconnected", Alert.AlertType.kError)
            .withCondition(() -> !inputs.encoderConnected);
    private final Alert limitSwitchDisconnected =
        new AlertUtil("Elevator Limit Switch Disconnected", Alert.AlertType.kError)
            .withCondition(() -> !inputs.limitSwitchConnected);
    private final Alert canCoderMagnetBad =
        new AlertUtil("Elevator CANcoder Magnet Bad", Alert.AlertType.kError)
            .withCondition(() -> !inputs.magnetGood);
    private final Alert eStopped =
        new AlertUtil("Elevator is E-Stopped", Alert.AlertType.kError)
            .withCondition(() -> shouldEStop);

    public void enable() {
        mEnabled = true;
        mController.reset(getPosition());
    }

    public void disable() {
        mEnabled = false;
        runElevator(0, new TrapezoidProfile.State());
    }

    public boolean isEnabled() {
        return mEnabled;
    }

    public Elevator(ElevatorIO io) {
        mController =
            new ProfiledPIDController(
                ElevatorConstants.KP,
                ElevatorConstants.KI,
                ElevatorConstants.KD,
                new TrapezoidProfile.Constraints(
                    ElevatorConstants.MAX_VELOCITY_PER_SEC,
                    ElevatorConstants.MAX_ACCELERATION_PER_SEC_SQUARED));
        mController.setTolerance(ElevatorConstants.TOLERANCE);
        mFeedforward =
            new ElevatorFeedforward(
                ElevatorConstants.KS,
                ElevatorConstants.KG,
                ElevatorConstants.KV);

        mSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runElevator(voltage.in(Volts)), null, this));

        mOperatorLock = OperatorLock.LOCKED;

        this.io = io;
        disable();

        if (inputs.limitSwitchPressed) {
            io.zeroEncoders();
        } else {
            homeCommand()
                .andThen(io::zeroEncoders)
                .schedule();
        }
    }

    private boolean limitPressed() {
        return inputs.limitSwitchPressed;
    }

    public boolean willTipAtFastSpeeds() {
        return getPosition() > ElevatorConstants.MAX_RADIANS - ElevatorConstants.TIP_THRESHOLD;
    }

    @Override
    public void periodic() {
        inputs.atTopLimit = getPosition() >= ElevatorConstants.MAX_RADIANS;
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/Enabled", mEnabled);

        // stop adding up pid error while disabled
        if (DriverStation.isDisabled()) {
            mController.reset(getPosition());
        }

        Logger.recordOutput("Elevator/Error-kP", mController.getPositionError());
        Logger.recordOutput("Elevator/AccumulatedError-kI", mController.getAccumulatedError());
        Logger.recordOutput("Elevator/ErrorDerivativeTolerance-kD", mController.getVelocityError());

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Elevator/CurrentCommand", getCurrentCommand().getName());
        }

        shouldEStop =
            !Clearances.AlgaeClawClearances.isClearFromElevatorCrossbeam()
                && Math.signum(inputs.encoderVelocityRadPerSec) == -1;

        if (Clearances.AlgaeClawClearances.willCollideIntoElevator() || shouldEStop) {
            io.stop();
            disable();
            return;
        }

        if (mEnabled) {
            runElevator(mController.calculate(getPosition()), mController.getSetpoint());
        }
    }

    private void runElevator(double fb, TrapezoidProfile.State setpoint) {
        double ff = mFeedforward.calculate(setpoint.velocity);
        Logger.recordOutput("Elevator/Feedback", fb);
        Logger.recordOutput("Elevator/Feedforward", ff);
        double volts = fb + ff;

        if ((inputs.atTopLimit && volts >= 0) || (limitPressed() && volts <= 0)) {
            io.stop();
            return;
        }

        io.runElevator(volts);
    }

    @AutoLogOutput(key = "Elevator/AdjustedPosition")
    public double getPosition() {
        return inputs.encoderPositionRad;
    }

    public Trigger atGoal() {
        return new Trigger(mController::atGoal);
    }

    public Trigger atThisGoal(ElevatorConstants.State state) {
        return new Trigger(
            () -> Math.abs(getPosition() - state.getAngle().getRadians()) <= ElevatorConstants.TOLERANCE * 3.0);
    }

    public Trigger atLimit() {
        return new Trigger(() -> inputs.atTopLimit || limitPressed());
    }

    public Trigger atHome() {
        return new Trigger(this::limitPressed);
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return mSysId.quasistatic(dir)
            .finallyDo(io::stop);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return mSysId.dynamic(dir)
            .finallyDo(io::stop);
    }

    public Command setDesiredState(ElevatorConstants.State state) {
        return Commands.runOnce(
            () -> {
                double goal =
                    MathUtil.clamp(state.getAngle().getRadians(), 0, ElevatorConstants.MAX_RADIANS);
                inputs.goal = goal;
                mController.setGoal(new TrapezoidProfile.State(goal, 0));
                enable();
            }, this)
            .withName("Set Desired State");
    }

    public double getDesiredState() {
        return inputs.goal;
    }

    public Command toggleManualControl(DoubleSupplier joystickAxis) {
        return Commands.runOnce(
            () -> {
                Logger.recordOutput("Elevator/RequestedElevatorSpeed", joystickAxis.getAsDouble());

                if (mOperatorLock == OperatorLock.LOCKED) {
                    disable();
                    setDefaultCommand(
                        elevatorManual(
                            () -> MathUtil.clamp(
                                MathUtil.applyDeadband(joystickAxis.getAsDouble(), Deadbands.ELEVATOR_DEADBAND),
                                -ElevatorConstants.MANUAL_ELEVATOR_INCREMENT,
                                ElevatorConstants.MANUAL_ELEVATOR_INCREMENT)));
                    mOperatorLock = OperatorLock.UNLOCKED;
                } else {
                    if (getDefaultCommand() != null) {
                        getDefaultCommand().cancel();
                        removeDefaultCommand();
                    }
                    slamCommand().schedule();
                    mOperatorLock = OperatorLock.LOCKED;
                }

                Logger.recordOutput("Elevator/IsLocked", mOperatorLock == OperatorLock.LOCKED);
            }, this)
            .withName("Toggle Manual Control");
    }

    private Command elevatorManual(DoubleSupplier speed) {
        return Commands.runOnce(this::disable, this)
            .andThen(
                Commands.run(
                    () -> {
                        double appliedSpeed = speed.getAsDouble();

                        if (speed.getAsDouble() == 0.0) {
                            appliedSpeed = ElevatorConstants.KG / 12.0;
                        }

                        Logger.recordOutput("Elevator/ManualAppliedSpeed", appliedSpeed);
                        io.runElevatorViaSpeed(appliedSpeed);
                    }, this))
            .finallyDo(
                io::stop)
            .withName("Manual Elevator");
    }

    public Command slamCommand() {
        return Commands.runOnce(this::disable)
            .andThen(
                Commands.run(
                    () -> io.runElevatorViaSpeed(-ElevatorConstants.MANUAL_ELEVATOR_INCREMENT / 1.5), this))
        .until(this::limitPressed)
        .finallyDo(io::stop)
        .withName("Slam Elevator");
    }

    public Command noSlamCommand() {
        //                io.zeroEncoders();
        return setDesiredState(ElevatorConstants.State.HOME_ABOVE_BAR)
            .andThen(
                Commands.waitUntil(atThisGoal(ElevatorConstants.State.HOME_ABOVE_BAR)),
                Commands.runOnce(this::disable),
                Commands.run(() -> io.runElevatorViaSpeed(-0.1)))
            .until(this::limitPressed)
            .finallyDo(io::stop)
            .withName("No Slam Elevator");
    }

    public Command homeCommand() {
        //                io.zeroEncoders();
        return setDesiredState(ElevatorConstants.State.HOME)
            .until(this::limitPressed)
            .finallyDo(io::stop)
            .withName("Home Elevator");
    }

    public Command applykS() {
        return Commands.run(
            () -> io.runElevator(ElevatorConstants.KS), this)
            .finallyDo(io::stop);
    }

    public Command applykG() {
        return Commands.run(
            () -> io.runElevator(ElevatorConstants.KG), this)
            .finallyDo(io::stop);
    }

    public Command applykV() {
        return Commands.run(
            () -> {
                double volts = ElevatorConstants.KS + ElevatorConstants.KV;
                io.runElevator(volts);
            }, this)
            .finallyDo(io::stop);
    }

    public Command applyVolts(double volts) {
        return Commands.run(
            () -> {
                Logger.recordOutput("Elevator/AppliedVolts", volts);
                io.runElevator(volts);
            }, this)
            .finallyDo(io::stop);
    }
}
