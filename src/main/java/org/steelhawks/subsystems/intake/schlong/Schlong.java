package org.steelhawks.subsystems.intake.schlong;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private final Alert leftMotorDisconnected;
    private final Alert rightMotorDisconnected;
    private final Alert canCoderDisconnected;
    private final Alert limitSwitchDisconnected;
    private final Alert canCoderMagnetBad;

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

    }
}
