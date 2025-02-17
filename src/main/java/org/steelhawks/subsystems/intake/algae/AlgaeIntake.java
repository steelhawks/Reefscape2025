package org.steelhawks.subsystems.intake.algae;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Constants.Deadbands;
import org.steelhawks.OperatorLock;
import org.steelhawks.subsystems.intake.IntakeConstants;
import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class AlgaeIntake extends SubsystemBase {

    private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();
    private final IntakeConstants constants;
    private final SysIdRoutine mAlgaeSysId;
    private boolean mEnabled = false;
    private final AlgaeIntakeIO io;
    private OperatorLock mOperatorLock;

    private final ProfiledPIDController mController;
    private ArmFeedforward mFeedforward;

    private final Alert intakeMotorDisconnected;
    private final Alert pivotMotorDisconnected;
    private final Alert canCoderDisconnected;
    private final Alert limitSwitchDisconnected;
    private final Alert canCoderMagnetBad;

    public void enable() {
        mController.reset(inputs.encoderPositionRad);
        mEnabled = true;
    }

    public void disable() {
        runPivot(0, new TrapezoidProfile.State());
        mEnabled = false;
    }

    public AlgaeIntake(AlgaeIntakeIO io) {
        this.io = io;

        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = IntakeConstants.ALPHA;
            case HAWKRIDER -> constants = IntakeConstants.HAWKRIDER;
            default -> constants = IntakeConstants.OMEGA;
        }

        intakeMotorDisconnected =
            new Alert("Intake Motor is Disconnected", AlertType.kError);
        pivotMotorDisconnected =
            new Alert("Pivot Motor is Disconnected", AlertType.kError);
        canCoderDisconnected =
            new Alert("CANcoder is Disconnected", AlertType.kError);
        limitSwitchDisconnected =
            new Alert("Limit Switch is Disconnected", AlertType.kError);
        canCoderMagnetBad =
            new Alert("CANcoder Magnet is Bad", AlertType.kError);

        mAlgaeSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(.25).per(Second),
                    Volts.of(.5), // lower dynamic sysid test to .5 volts instead of 7 which slams into elevator
                    null,
                    (state) -> Logger.recordOutput("Intake/Algae/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runPivotWithVoltage(voltage.in(Volts)), null, this));

        mController =
            new ProfiledPIDController(
                constants.ALGAE_KP,
                constants.ALGAE_KI,
                constants.ALGAE_KD,
                new TrapezoidProfile.Constraints(
                    constants.ALGAE_MAX_VELOCITY_PER_SEC,
                    constants.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED));

        mController.setTolerance(constants.ALGAE_TOLERANCE);
        mController.enableContinuousInput(0, 2 * Math.PI);
        mController.setGoal(inputs.encoderPositionRad);

        mFeedforward =
            new ArmFeedforward(
                constants.ALGAE_KS,
                constants.ALGAE_KG,
                constants.ALGAE_KV);

        enable();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeIntake", inputs);
        Logger.recordOutput("AlgaeIntake/Enabled", mEnabled);

        intakeMotorDisconnected.set(!inputs.intakeConnected);
        pivotMotorDisconnected.set(!inputs.pivotConnected);
        canCoderDisconnected.set(!inputs.encoderConnected);
        limitSwitchDisconnected.set(!inputs.limitSwitchConnected);
        canCoderMagnetBad.set(!inputs.magnetGood);

        // stop adding up pid error while disabled
        if (DriverStation.isDisabled()) {
            mController.reset(getPosition());
        }

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Algae/CurrentCommand", getCurrentCommand().getName());
        }

        if (inputs.limitSwitchPressed) {
            io.zeroEncoders();
        }

        if (mEnabled) {
            // runPivot(mController.calculate(getPosition()), mController.getSetpoint());
            runPivot(0, mController.getSetpoint());
        }
    }

    @AutoLogOutput(key = "Algae/AdjustedPosition")
    private double getPosition() {
        return inputs.encoderPositionRad;
    }

    private void runPivot(double fb, TrapezoidProfile.State setpoint) {
        double ff = mFeedforward.calculate(setpoint.position, setpoint.velocity);
        Logger.recordOutput("Algae/Feedback", fb);
        Logger.recordOutput("Algae/Feedforward", ff);

//        io.runPivot(fb + ff);
    }

    public Trigger atGoal() {
        return new Trigger(mController::atGoal);
    }

    public Trigger atLimit() {
        return new Trigger(() -> inputs.limitSwitchPressed);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return mAlgaeSysId.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return mAlgaeSysId.dynamic(dir);
    }

    public Command toggleManualControl(DoubleSupplier joystickAxis) {
        return Commands.runOnce(
            () -> {
                if (mOperatorLock == OperatorLock.LOCKED) {
                    disable();
                    setDefaultCommand(
                        pivotManual(
                            () -> MathUtil.clamp(
                                MathUtil.applyDeadband(joystickAxis.getAsDouble(), Deadbands.PIVOT_DEADBAND),
                                -constants.ALGAE_MANUAL_PIVOT_INCREMENT,
                                constants.ALGAE_MANUAL_PIVOT_INCREMENT)));
                    mOperatorLock = OperatorLock.UNLOCKED;
                } else {
                    if (getDefaultCommand() != null) {
                        getDefaultCommand().cancel();
                        removeDefaultCommand();
                    }
                    homeCommand().schedule();
                    mOperatorLock = OperatorLock.LOCKED;
                }

                Logger.recordOutput("Algae/IsLocked", mOperatorLock == OperatorLock.LOCKED);
            }, this)
            .withName("Toggle Manual Control");
    }

    public Command pivotManual(DoubleSupplier speed) {
        return Commands.runOnce(this::disable)
            .andThen(
                Commands.run(
                    () -> {
                        double appliedSpeed = speed.getAsDouble();
                        Logger.recordOutput("Algae/ManualPivotSpeed", appliedSpeed);

                        if (appliedSpeed == 0.0) {
                            // convert to percentoutput
                            appliedSpeed = (Math.cos(getPosition()) * constants.ALGAE_KG) / 12.0;
                        }

                        io.runPivotWithSpeed(appliedSpeed);
                    }, this));
    }

    public Command homeCommand() {
        return Commands.run(
            () -> io.runPivotWithSpeed(.1), this)
            .until(() -> inputs.limitSwitchPressed)
            .finallyDo(() -> io.stopPivot());
    }

    public Command intake() {
        return Commands.run(
            () -> io.runIntake(-.6), this)
            .finallyDo(() -> io.stopIntake());
    }

    public Command outtake() {
        return Commands.run(
            () -> io.runIntake(.6), this)
            .finallyDo(() -> io.stopIntake());
    }

    // private static final double kS = 0.3525;
    // private static final double kG = 0.4;
    // private static final double kV = 0.2 * (1.0 / 1.15) * (1.0 / 1.1) * (1.0 / 1.4);

    public Command applykS() {
        return Commands.run(
            () -> io.runPivotWithVoltage(constants.ALGAE_KS), this)
            .finallyDo(() -> io.stopPivot());
    }

    public Command runPivotManual(boolean isUp) {
        return Commands.run(
            () -> io.runPivotWithSpeed(isUp ? -.1   : .1))
            .finallyDo(() -> io.stopPivot());
    }

    public Command runPivotManualUp() {
        return Commands.run(
            () -> io.runPivotWithSpeed(.1))
            .finallyDo(() -> io.stopPivot());
    }

    public Command runPivotManualDown() {
        return Commands.run(
            () -> io.runPivotWithSpeed(-.1))
            .finallyDo(() -> io.stopPivot());
    }


    public Command applykG() {
        return Commands.run(
            () -> io.runPivotWithVoltage(Math.cos(inputs.encoderPositionRad) * constants.ALGAE_KG), this)
            .finallyDo(() -> io.stopPivot());
    }

    public Command applykV() {
        return Commands.run(
            () -> io.runPivotWithVoltage(constants.ALGAE_KS + (Math.cos(inputs.encoderPositionRad) * constants.ALGAE_KG) + constants.ALGAE_KV))
            .finallyDo(() -> io.stopPivot());
    }

    public Command applyVolts(double volts) {
        return Commands.run(
            () -> {
                io.runPivotWithVoltage(volts);
            }, this)
            .finallyDo(() -> io.stopPivot());
    }

    public Command setDesiredState(IntakeConstants.AlgaeIntakeState state) {
        return Commands.runOnce(
            () -> {
                double goal =
                    MathUtil.clamp(state.getRadians(), 0, constants.ALGAE_MAX_RADIANS);
                inputs.goal = goal;
                // mController.setGoal(goal);
                mController.setGoal(new TrapezoidProfile.State(goal, 0));
                enable();
            }, this);
    }
}
