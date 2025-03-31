package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants.Deadbands;
import org.steelhawks.OperatorLock;
import org.steelhawks.RobotContainer;

import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Volts;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {

    private static final double VELOCITY_TOLERANCE = 0.1; // need to find irl
    private static final double SLOW_DOWN_HOME_SPEED = -0.15;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private OperatorLock mOperatorLock;
    private final SysIdRoutine mSysId;
    private boolean mEnabled = false;
    private final ElevatorIO io;

    private final Alert leftMotorDisconnected;
    private final Alert rightMotorDisconnected;
    private final Alert canCoderDisconnected;
    private final Alert limitSwitchDisconnected;
    private final Alert canCoderMagnetBad;

    public void enable() {
        mEnabled = true;
    }

    public void disable() {
        mEnabled = false;
        io.stop();
    }

    public boolean isEnabled() {
        return mEnabled;
    }

    public Elevator(ElevatorIO io) {
        mOperatorLock = OperatorLock.LOCKED;
        mSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runElevator(voltage.in(Volts)), null, this));

        leftMotorDisconnected =
            new Alert(
                "Left Elevator Motor Disconnected", AlertType.kError);

        rightMotorDisconnected =
            new Alert(
                "Right Elevator Motor Disconnected", AlertType.kError);

        canCoderDisconnected =
            new Alert(
                "Elevator CANcoder Disconnected", AlertType.kError);

        limitSwitchDisconnected =
            new Alert(
                "Elevator Limit Switch Disconnected", AlertType.kError);

        canCoderMagnetBad =
            new Alert(
                "Elevator CANcoder Magnet Bad", AlertType.kError);

        this.io = io;
        disable();

        io.zeroEncoders();
    }

    private boolean limitPressed() {
        return inputs.limitSwitchPressed;
    }

    @Override
    public void periodic() {
        updateGains();
        inputs.atTopLimit = getPosition() >= ElevatorConstants.MAX_RADIANS;
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/Enabled", mEnabled);

        leftMotorDisconnected.set(!inputs.leftConnected);
        rightMotorDisconnected.set(!inputs.rightConnected);
        canCoderDisconnected.set(!inputs.encoderConnected);
        limitSwitchDisconnected.set(!inputs.limitSwitchConnected);
        canCoderMagnetBad.set(!inputs.magnetGood);

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Elevator/CurrentCommand", getCurrentCommand().getName());
        }

        if (mEnabled)
            runElevator();
    }

    private void updateGains() {
        if (RobotContainer.s_Claw.hasCoral().getAsBoolean()) {
            io.setPID(
                ElevatorConstants.CORAL_KP,
                ElevatorConstants.CORAL_KI,
                ElevatorConstants.CORAL_KD);
            io.setFF(
                ElevatorConstants.CORAL_KS,
                ElevatorConstants.CORAL_KG,
                ElevatorConstants.CORAL_KV,
                ElevatorConstants.CORAL_KA);
        } else {
            io.setPID(
                ElevatorConstants.NONE_KP,
                ElevatorConstants.NONE_KI,
                ElevatorConstants.NONE_KD);
            io.setFF(
                ElevatorConstants.NONE_KS,
                ElevatorConstants.NONE_KG,
                ElevatorConstants.NONE_KV,
                ElevatorConstants.NONE_KA);
        }
    }

    private double directionOfMovement() {
        LinearFilter filter = LinearFilter.movingAverage(5);
        if (Math.abs(inputs.encoderVelocityRadPerSec) < VELOCITY_TOLERANCE) // need to find irl
            return 0;
        return Math.signum(filter.calculate(inputs.encoderVelocityRadPerSec));
    }

    private void runElevator() {
        if ((inputs.atTopLimit && directionOfMovement() > 0) || (limitPressed() && directionOfMovement() < 0)) { // need to test if dir of movement works fine
            io.stop();
            return;
        }
        io.runPosition(inputs.goal);
    }

    @AutoLogOutput(key = "Elevator/AdjustedPosition")
    public double getPosition() {
        return inputs.encoderPositionRad - ElevatorConstants.CANCODER_OFFSET;
    }

    public Trigger atGoal() {
        return new Trigger(
            () -> Math.abs(getPosition() - inputs.goal) <= ElevatorConstants.TOLERANCE * 1.5);
    }

    public Trigger atThisGoal(ElevatorConstants.State state) {
        return new Trigger(
            () -> Math.abs(getPosition() - state.getAngle().getRadians()) <= ElevatorConstants.TOLERANCE * 1.5);
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
            () -> inputs.goal = MathUtil.clamp(state.getAngle().getRadians(), 0, ElevatorConstants.MAX_RADIANS), this)
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
                            appliedSpeed = (RobotContainer.s_Claw.hasCoral().getAsBoolean() ? ElevatorConstants.CORAL_KG : ElevatorConstants.NONE_KG) / 12.0;
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
                    () -> io.runElevatorViaSpeed(-ElevatorConstants.MANUAL_ELEVATOR_INCREMENT / 2.0), this))
        .until(this::limitPressed)
        .finallyDo(io::stop)
        .withName("Slam Elevator");
    }

    public Command noSlamCommand() {
        return setDesiredState(ElevatorConstants.State.HOME_ABOVE_BAR)
            .andThen(
                Commands.waitUntil(atThisGoal(ElevatorConstants.State.HOME_ABOVE_BAR)),
                Commands.runOnce(this::disable),
                Commands.run(() -> io.runElevatorViaSpeed(SLOW_DOWN_HOME_SPEED)))
            .until(this::limitPressed)
            .finallyDo(io::stop)
            .withName("No Slam Elevator");
    }

    public Command homeCommand() {
        return setDesiredState(ElevatorConstants.State.HOME)
            .until(this::limitPressed)
            .finallyDo(io::stop)
            .withName("Home Elevator");
    }

    public Command applykS() {
        return Commands.run(
            () -> io.runElevator(ElevatorConstants.CORAL_KS), this)
            .finallyDo(io::stop);
    }

    public Command applykG() {
        return Commands.run(
            () -> io.runElevator(ElevatorConstants.CORAL_KG), this)
            .finallyDo(io::stop);
    }

    public Command applykV() {
        return Commands.run(
            () -> {
                double volts = ElevatorConstants.CORAL_KS + ElevatorConstants.CORAL_KV;
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
