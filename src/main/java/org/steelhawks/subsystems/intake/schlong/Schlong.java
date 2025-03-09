package org.steelhawks.subsystems.intake.schlong;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
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
import org.steelhawks.subsystems.intake.IntakeConstants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;


public class Schlong extends SubsystemBase {
    private final SchlongIOInputsAutoLogged inputs = new SchlongIOInputsAutoLogged();
    // private final Trigger shouldEStop;
    private final SysIdRoutine mSysId;
    private boolean mEnabled = false;
    private final SchlongIO io;

    private final ProfiledPIDController mController;
    private ArmFeedforward mFeedforward;

    private final Alert pivotMotorDisconnected;
    private final Alert spinMotorDisconnected;
    private final Alert limitSwitchDisconnected;

    public void enable() {
        mEnabled = true;
        mController.reset(getPivotPosition());
    }

    public void disable() {
        mEnabled = false;
        runPivot(0, new TrapezoidProfile.State());
    }

    public Schlong(SchlongIO io) {
//        shouldEStop = new Trigger(
//            () -> !RobotContainer.s_Elevator.atGoal().getAsBoolean()
//                && getPivotPosition() >= constants.SCHLONG_MAX_RADIANS);
        // shouldEStop =
        //     new Trigger(
        //         RobotContainer.s_Elevator.atGoal().negate()
        //         .and(atLimit()));

        mController = 
            new ProfiledPIDController(
                IntakeConstants.SCHLONG_KP,
                IntakeConstants.SCHLONG_KI,
                IntakeConstants.SCHLONG_KD,
                new TrapezoidProfile.Constraints(
                    IntakeConstants.SCHLONG_MAX_VELOCITY_PER_SEC,
                    IntakeConstants.SCHLONG_MAX_ACCELERATION_PER_SEC_SQUARED));
        mController.setTolerance(IntakeConstants.SCHLONG_TOLERANCE);
        
        mFeedforward = 
            new ArmFeedforward(
                IntakeConstants.SCHLONG_KS,
                IntakeConstants.SCHLONG_KG,
                IntakeConstants.SCHLONG_KV);

        mSysId = 
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(.25).per(Second),
                    Volts.of(.5),
                    null,
                    (state) -> Logger.recordOutput("Schlong/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runPivotWithVoltage("SYSID", voltage.in(Volts)), null, this));

        pivotMotorDisconnected = 
            new Alert(
                "Schlong Pivot Motor Disconnected", AlertType.kError);
        
        spinMotorDisconnected =
            new Alert(
                "Schlong Spin Motor Disconnected", AlertType.kError);

        limitSwitchDisconnected =
            new Alert(
                "Schlong Limit Switch Disconnected", AlertType.kError);

        this.io = io;

        disable();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Schlong", inputs);
        Logger.recordOutput("Schlong/Enabled", mEnabled);

        pivotMotorDisconnected.set(!inputs.pivotConnected);
        spinMotorDisconnected.set(!inputs.spinConnected);
        limitSwitchDisconnected.set(!inputs.limitSwitchConnected && IntakeConstants.SCHLONG_LIMIT_SWITCH_ID != -1);

        // stop adding up pid error while disabled
        if (DriverStation.isDisabled()) {
            mController.reset(getPivotPosition());
        }

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Schlong/CurrentCommand", getCurrentCommand().getName());
        }
//
//        if (shouldEStop.getAsBoolean()) {
//            disable();
//            return;
//        }

//        if (mEnabled) {
//            runPivot(mController.calculate(getPivotPosition()), mController.getSetpoint());
//        }
    }

    private void runPivot(double fb, TrapezoidProfile.State setpoint) {
        double ff = mFeedforward.calculate(setpoint.position, setpoint.velocity);
        Logger.recordOutput("Schlong/Feedback", fb);
        Logger.recordOutput("Schlong/Feedforward", ff);
        double volts = fb + ff;

        if ((atLimit().getAsBoolean() && volts <= 0)) {
            io.stopPivot();
            return;
        }

        // io.runPivotWithVoltage("Subsystem", volts);
    }

    @AutoLogOutput(key = "Schlong/AdjustedPosition")
    public double getPivotPosition() {
       final double armOffsetToZero = -6.270913460876501 - 0.15646604036433587;
       return ((inputs.pivotPositionRad + armOffsetToZero) * ((-Math.PI / 2) / (-4.3933209765044765)));
        // final double armOffsetToZero = -.22;
        // return Units.degreesToRadians(Conversions.convert360To180(inputs.pivotPositionRad + armOffsetToZero));
    }

    public Trigger atGoal() {
        return new Trigger(mController::atGoal);
    }

    public Trigger atThisGoal(IntakeConstants.SchlongState state) {
        return new Trigger(
            () -> Math.abs(getPivotPosition() - state.getRadians()) <= IntakeConstants.SCHLONG_TOLERANCE);
    }

    public Trigger atLimit() {
        return new Trigger(() -> getPivotPosition() >= IntakeConstants.SCHLONG_MAX_RADIANS);
    }


    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return mSysId.quasistatic(dir)
            .finallyDo(() -> io.stopPivot());
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return mSysId.dynamic(dir)
            .finallyDo(() -> io.stopPivot());
    }

    public Command setDesiredState(IntakeConstants.SchlongState state) {
        return Commands.runOnce(
            () -> {
                double goal =
                    MathUtil.clamp(state.getRadians(), 0, IntakeConstants.SCHLONG_MAX_RADIANS);
                inputs.goal = goal;
                mController.setGoal(new TrapezoidProfile.State(goal, 0));
                enable();
            }, this)
            .withName("Set Desired State");
    }

    public Command applyPivotSpeed(double speed) {
        return Commands.run(() -> {
            io.runPivotWithSpeed(speed);
        }, this)
        .finallyDo(() -> io.stopPivot());
    }

    public Command applyPivotVolts(double volts) {
        return Commands.run(
            () -> io.runPivotWithVoltage("Volts testing linear", volts))
            .finallyDo(()-> io.stopPivot());
    }

    public Command applySpinSpeed(double speed) {
        return Commands.run(
            () -> {
                io.runSpinWithSpeed(speed);
            })
            .finallyDo(() -> io.stopSpin());
    }

    public Command applySpinVolts(double speed) {
        return Commands.run(
            () -> {
                io.runSpinWithVoltage(speed);
            }, this)
            .finallyDo(() -> io.stopSpin());
    }

    public double getDesiredState() {
        return inputs.goal;
    }

    public Command applykS() {
        return Commands.run(
            () -> io.runPivotWithVoltage("Apply Ks", IntakeConstants.SCHLONG_KS))
            .finallyDo(() -> io.stopPivot());
    }

    public Command applykG() {
        return Commands.run(
            () -> {
               double volts = IntakeConstants.SCHLONG_KG * Math.cos(getPivotPosition());
                // double volts = mFeedforward.calculate(getPivotPosition(), 0);
                Logger.recordOutput("Schlong/GravityCompensation", volts);
                io.runPivotWithVoltage("Apply KG", volts);
            }, this)
            .finallyDo(() -> io.stopPivot());
    }
    
    public Command applykV() {
        return Commands.run(
            () -> io.runPivotWithVoltage("Apply KV",
                IntakeConstants.SCHLONG_KS * Math.signum(getPivotPosition()) + IntakeConstants.SCHLONG_KG * Math.cos(getPivotPosition()) + IntakeConstants.SCHLONG_KV))
                .finallyDo(() -> io.stopPivot());
    }
}
