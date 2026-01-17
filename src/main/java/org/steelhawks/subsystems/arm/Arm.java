package org.steelhawks.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
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
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;


public class Arm extends SubsystemBase {
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    // private final Trigger shouldEStop;
    private final SysIdRoutine mSysId;
    private boolean mEnabled = false;
    private final ArmIO io;

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

    public Arm(ArmIO io) {
//        shouldEStop = new Trigger(
//            () -> !RobotContainer.s_Elevator.atGoal().getAsBoolean()
//                && getPivotPosition() >= constants.SCHLONG_MAX_RADIANS);
        // shouldEStop =
        //     new Trigger(
        //         RobotContainer.s_Elevator.atGoal().negate()
        //         .and(atLimit()));

        mController = 
            new ProfiledPIDController(
                ArmConstants.ARM_KP,
                ArmConstants.ARM_KI,
                ArmConstants.ARM_KD,
                new TrapezoidProfile.Constraints(
                    ArmConstants.ARM_MAX_VELOCITY_PER_SEC,
                    ArmConstants.ARM_MAX_ACCELERATION_PER_SEC_SQUARED));
        mController.setTolerance(ArmConstants.ARM_TOLERANCE);
        
        mFeedforward = 
            new ArmFeedforward(
                ArmConstants.ARM_KS,
                ArmConstants.ARM_KG,
                ArmConstants.ARM_KV);

        mSysId = 
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(.25).per(Second),
                    Volts.of(.5),
                    null,
                    (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runPivotWithVoltage(voltage.in(Volts)), null, this));

        pivotMotorDisconnected = 
            new Alert(
                "Arm Pivot Motor Disconnected", AlertType.kError);
        
        spinMotorDisconnected =
            new Alert(
                "Arm Spin Motor Disconnected", AlertType.kError);

        limitSwitchDisconnected =
            new Alert(
                "Arm Limit Switch Disconnected", AlertType.kError);

        this.io = io;

        home().schedule();
        disable();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/Enabled", mEnabled);

        pivotMotorDisconnected.set(!inputs.pivotConnected);
        spinMotorDisconnected.set(!inputs.spinConnected);
        limitSwitchDisconnected.set(!inputs.limitSwitchConnected && ArmConstants.ARM_LIMIT_SWITCH_ID != -1);

        // stop adding up pid error while disabled
        if (DriverStation.isDisabled()) {
            mController.reset(getPivotPosition());
        }

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Arm/CurrentCommand", getCurrentCommand().getName());
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
        Logger.recordOutput("Arm/Feedback", fb);
        Logger.recordOutput("Arm/Feedforward", ff);
        double volts = fb + ff;

        if ((atLimit().getAsBoolean() && volts <= 0)) {
            io.stopPivot();
            return;
        }

//         io.runPivotWithVoltage(volts);
    }

    @AutoLogOutput(key = "Arm/AdjustedPosition")
    public double getPivotPosition() {
       final double armOffsetToZero = -6.270913460876501 - 0.15646604036433587;
       return ((inputs.pivotPositionRad + armOffsetToZero) * ((-Math.PI / 2) / (-4.3933209765044765)));
        // final double armOffsetToZero = -.22;
        // return Units.degreesToRadians(Conversions.convert360To180(inputs.pivotPositionRad + armOffsetToZero));
    }

    public double getDesiredState() {
        return inputs.goal;
    }

    public Trigger atGoal() {
        return new Trigger(mController::atGoal);
    }

    public Trigger atThisGoal(ArmConstants.ArmState state) {
        return new Trigger(
            () -> Math.abs(getPivotPosition() - state.getRadians()) <= ArmConstants.ARM_TOLERANCE);
    }

    public Trigger atLimit() {
        return new Trigger(() -> getPivotPosition() >= ArmConstants.ARM_MAX_RADIANS);
    }

    public Trigger armClearFromReef() {
        return new Trigger(
            () -> FieldConstants.REEF_CENTER.getDistance(RobotContainer.s_Swerve.getPose().getTranslation())
                > (RobotConstants.DIST_CLEAR_FROM_REEF
                + FieldConstants.CENTER_OF_REEF_TO_REEF_FACE
                + RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0));
    }

    // TEST THIS
    public Trigger armClearFromElevator() {
        return new Trigger(
            () -> RobotContainer.s_Elevator.isEnabled() && !RobotContainer.s_Elevator.atGoal().getAsBoolean()
                && getPivotPosition() >= ArmConstants.ARM_MIN_RADIANS
                && getPivotPosition() <= ArmConstants.ARM_MAX_RADIANS
                && Math.abs(getPivotPosition() - ArmConstants.ArmState.AVOID_ELEVATOR.getRadians()) <= ArmConstants.ARM_TOLERANCE);
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

    public Command setDesiredState(ArmConstants.ArmState state) {
        return Commands.runOnce(
            () -> {
                double goal =
                    MathUtil.clamp(state.getRadians(), 0, ArmConstants.ARM_MAX_RADIANS);
                inputs.goal = goal;
                mController.setGoal(new TrapezoidProfile.State(goal, 0));
                enable();
            }, this)
            .withName("Set Desired State");
    }

    public Command home() {
        return setDesiredState(ArmConstants.ArmState.HOME);
    }

    public Command erect() {
        return setDesiredState(ArmConstants.ArmState.ERECT);
    }

    public Command applyPivotSpeed(double speed) {
        return Commands.run(() -> {
                io.runPivotWithSpeed(speed);
        })
        .finallyDo(() -> io.stopPivot());
    }

    public Command applyPivotVolts(double volts) {
        return Commands.run(
            () -> io.runPivotWithVoltage(volts))
            .finallyDo(()-> io.stopPivot());
    }

    public Command applySpinSpeed(double speed) {
        return Commands.run(
            () -> {
                io.runSpinWithSpeed(speed);
            }, this)
            .finallyDo(() -> io.stopSpin());
    }

    public Command applySpinVolts(double speed) {
        return Commands.run(
            () -> {
                io.runSpinWithVoltage(speed);
            }, this)
            .finallyDo(() -> io.stopSpin());
    }

    public Command applykS() {
        return Commands.run(
            () -> io.runPivotWithVoltage(ArmConstants.ARM_KS))
            .finallyDo(() -> io.stopPivot());
    }

    public Command applykG() {
        return Commands.run(
            () -> {
               double volts = ArmConstants.ARM_KG * Math.cos(getPivotPosition());
                // double volts = mFeedforward.calculate(getPivotPosition(), 0);
                Logger.recordOutput("Arm/GravityCompensation", volts);
                io.runPivotWithVoltage(volts);
            }, this)
            .finallyDo(() -> io.stopPivot());
    }
    
    public Command applykV() {
        return Commands.run(
            () -> io.runPivotWithVoltage(ArmConstants.ARM_KS * Math.signum(getPivotPosition()) + ArmConstants.ARM_KG * Math.cos(getPivotPosition()) + ArmConstants.ARM_KV))
                .finallyDo(() -> io.stopPivot());
    }
}
