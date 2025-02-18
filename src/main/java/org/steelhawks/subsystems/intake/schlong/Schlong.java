package org.steelhawks.subsystems.intake.schlong;

import org.steelhawks.subsystems.elevator.ElevatorIO;
import org.steelhawks.subsystems.intake.IntakeConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
        runElevator(0, new TrapezoidProfile.State());
    }


}
