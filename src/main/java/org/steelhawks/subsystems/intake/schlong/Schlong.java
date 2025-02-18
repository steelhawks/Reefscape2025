package org.steelhawks.subsystems.intake.schlong;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.elevator.ElevatorIO;
import org.steelhawks.subsystems.intake.IntakeConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;


public class Schlong extends SubsystemBase {
    private final SchlongIOInputsAutoLogged inputs = new SchlongIOInputsAutoLogged(); 
    private final IntakeConstants constants;
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
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = IntakeConstants.ALPHA;
            case HAWKRIDER -> constants = IntakeConstants.HAWKRIDER;
            default -> constants = IntakeConstants.OMEGA;
        }

        mController = 
            new ProfiledPIDController(
                constants.SCHLONG_KP, 
                constants.SCHLONG_KI, 
                constants.SCHLONG_KD, 
                new TrapezoidProfile.Constraints(
                    constants.SCHLONG_MAX_VELOCITY_PER_SEC, 
                    constants.SCHLONG_MAX_ACCELERATION_PER_SEC_SQUARED));
        mController.setTolerance(constants.SCHLONG_TOLERANCE);
        
        mFeedforward = 
            new ArmFeedforward(
                constants.SCHLONG_KS, 
                constants.SCHLONG_KG, 
                constants.SCHLONG_KV);

        mSysId = 
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Schlong/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runPivotWithVoltage(voltage.in(Volts)), null, this));

        pivotMotorDisconnected = 
            new Alert(
                "Schlong Pivot Motor Disconnected", AlertType.kError);
        
        spinMotorDisconnected =
            new Alert(
                "Right Elevator Motor Disconnected", AlertType.kError);

        limitSwitchDisconnected =
            new Alert(
                "Elevator Limit Switch Disconnected", AlertType.kError);

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
        limitSwitchDisconnected.set(!inputs.limitSwitchConnected);

        // stop adding up pid error while disabled
        if (DriverStation.isDisabled()) {
            mController.reset(getPivotPosition());
        }

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Schlong/CurrentCommand", getCurrentCommand().getName());
        }

        if (mEnabled) {
            runElevator(mController.calculate(getPivotPosition()), mController.getSetpoint());
        }
    }

    private void runPivot(double fb, TrapezoidProfile.State setpoint) {
        double ff = mFeedforward.calculate(setpoint.position, setpoint.velocity);
        Logger.recordOutput("Schlong/Feedback", fb);
        Logger.recordOutput("Schlong/Feedforward", ff);
        double volts = fb + ff;

        if ((inputs.limitSwitchPressed && volts <= 0)) {
            io.stopPivot();
            return;
        }

        io.runPivotWithVoltage(volts);
    }

    @AutoLogOutput(key = "Schlong/AdjustedPosition")
    public double getPosition() {
        return inputs.encoderPositionRad;
    }

    public Trigger atGoal() {
        return new Trigger(mController::atGoal);
    }

    public Trigger atThisGoal(ElevatorConstants.State state) {
        return new Trigger(
            () -> Math.abs(getPosition() - state.getRadians()) <= constants.TOLERANCE);
    }

    public Trigger atLimit() {
        return new Trigger(() -> inputs.atTopLimit || inputs.limitSwitchPressed);
    }



}
