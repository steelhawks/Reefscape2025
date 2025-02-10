package org.steelhawks.subsystems.intake.algae;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeConstants;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class AlgaeIntake extends SubsystemBase {

    private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();
    private final IntakeConstants constants;
    private final SysIdRoutine mAlgaeSysId;
    private boolean mEnabled = false;
    private final AlgaeIntakeIO io;

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
                    null,
                    Volts.of(2), // lower dynamic sysid test to 2 volts instead of 7 which slams into elevator
                    null,
                    (state) -> Logger.recordOutput("Intake/Algae/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runPivot(voltage.in(Volts)), null, this));

        mController =
            new ProfiledPIDController(
                constants.ALGAE_KP,
                constants.ALGAE_KI,
                constants.ALGAE_KD,
                new TrapezoidProfile.Constraints(
                    constants.ALGAE_MAX_VELOCITY_PER_SEC,
                    constants.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED));

        mController.setTolerance(constants.ALGAE_TOLERANCE);

        mController.setGoal(inputs.encoderPositionRad);

        mFeedforward =
            new ArmFeedforward(
                constants.ALGAE_KS,
                constants.ALGAE_KG,
                constants.ALGAE_KV);

        disable();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeIntake", inputs);

        intakeMotorDisconnected.set(!inputs.intakeConnected);
        pivotMotorDisconnected.set(!inputs.pivotConnected);
        canCoderDisconnected.set(!inputs.encoderConnected);
        limitSwitchDisconnected.set(!inputs.limitSwitchConnected);
        canCoderMagnetBad.set(!inputs.magnetGood);

//        if (DriverStation.isDisabled()) {
//            mController.setGoal(inputs.encoderPositionRad);
//        }

        if (!mEnabled) return;

        double pid =  mController.calculate(inputs.pivotPositionRad);
        TrapezoidProfile.State setpoint = mController.getSetpoint();
        double ff = mFeedforward.calculate(setpoint.position, setpoint.velocity);

        io.runPivot(pid + ff);
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

    public Command homeCommand() {
        return Commands.run(
            () -> io.runPivotManual(.05), this)
            .until(() -> inputs.limitSwitchPressed)
            .finallyDo(() -> io.stopPivot());
    }

    public Command intake() {
        return Commands.run(
            () -> io.runIntake(-.4), this)
            .finallyDo(() -> io.stopIntake());
        // return Commands.print("RUNNING ALGAE INTAKE COMMAND!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    public Command outtake() {
        return Commands.run(
            () -> io.runIntake(.4), this)
            .finallyDo(() -> io.stopIntake());
    }

    private static final double kS = 0.37;
    private static final double kG = 0.4;
    private static final double kV = 0.5;

    public Command applykS() {
        return Commands.run(
            () -> io.runPivot(kS), this)
            .finallyDo(() -> io.stopPivot());
    }

    public Command runPivotManual(boolean isUp) {
        return Commands.run(
            () -> io.runPivotManual(isUp ? -.1   : .1))
            .finallyDo(() -> io.stopPivot());
    }

    public Command runPivotManualUp() {
        return Commands.run(
            () -> io.runPivotManual(.1))
            .finallyDo(() -> io.stopPivot());
    }

    public Command runPivotManualDown() {
        return Commands.run(
            () -> io.runPivotManual(-.1))
            .finallyDo(() -> io.stopPivot());
    }


    public Command applykG() {
        return Commands.run(
            () -> io.runPivot(Math.cos(inputs.encoderPositionRad) * kG), this)
            .finallyDo(() -> io.stopPivot());
    }

    public Command applykV() {
        return Commands.run(
            () -> io.runPivot(kS + (Math.cos(inputs.encoderPositionRad) * kG) + kV))
            .finallyDo(() -> io.stopPivot());
    }

    public Command applyVolts(double volts) {
        return Commands.run(
            () -> {
                io.runPivot(volts);
            }, this)
            .finallyDo(() -> io.stopPivot());
    }

    public void setDesiredState(IntakeConstants.AlgaeIntakeState state) {
        double goal =
            MathUtil.clamp(state.getRadians(), 0, constants.ALGAE_MAX_RADIANS);
            inputs.setpoint = goal;
        mController.setGoal(goal);
        enable();
    }
}
