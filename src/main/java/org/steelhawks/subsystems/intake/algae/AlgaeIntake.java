package org.steelhawks.subsystems.intake.algae;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.elevator.ElevatorIO;
import org.steelhawks.subsystems.intake.IntakeConstants;

public class AlgaeIntake {

    private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();
    private final IntakeConstants constants;
    private boolean mEnabled = false;
    private final SysIdRoutine mSysId;
    private final AlgaeIntakeIO io;


    public final ProfiledPIDController mController;
    private ArmFeedforward mFeedforward;

    private final Alert intakeMotorDisconnected;
    private final Alert pivotMotorDisconnected;
    private final Alert canCoderDisconnected;
    private final Alert limitSwitchDisconnected;
    private final Alert canCoderMagnetBad;

    public void enable() {
        mController.reset(inputs.pivotPositionRad);
        mEnabled = true;
    }

    public void disable() {
        mEnabled = false;
    }

    public AlgaeIntake(AlgaeIntakeIO io, IntakeConstants constants) {
        this.io = io;
        this.constants = constants;

        mController =
            new ProfiledPIDController(
                constants.ALGAE_KP.getAsDouble(),
                constants.ALGAE_KI.getAsDouble(),
                constants.ALGAE_KD.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    constants.ALGAE_MAX_VELOCITY_PER_SEC.getAsDouble(),
                    constants.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED.getAsDouble()));

        mController.setTolerance(constants.ALGAE_TOLERANCE);

        mFeedforward =
            new ArmFeedforward(
                constants.ALGAE_KS.getAsDouble(),
                constants.ALGAE_KG.getAsDouble(),
                constants.ALGAE_KV.getAsDouble());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeIntake", inputs);

        if (constants.ALGAE_KP.hasChanged(hashCode()) ||
            constants.ALGAE_KI.hasChanged(hashCode()) ||
            constants.ALGAE_KD.hasChanged(hashCode()) ||
            constants.ALGAE_MAX_VELOCITY_PER_SEC.hasChanged(hashCode()) ||
            constants.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED.hasChanged(hashCode())
        ) {
            disable();
            mController.setPID(
                constants.ALGAE_KP.getAsDouble(),
                constants.ALGAE_KI.getAsDouble(),
                constants.ALGAE_KD.getAsDouble());

            enable();
        }

        if (constants.ALGAE_KS.hasChanged(hashCode()) ||
            constants.ALGAE_KG.hasChanged(hashCode()) ||
            constants.ALGAE_KV.hasChanged(hashCode())
        ) {
            mFeedforward = new ArmFeedforward(
                constants.ALGAE_KS.getAsDouble(),
                constants.ALGAE_KG.getAsDouble(),
                constants.ALGAE_KV.getAsDouble());
        }

        if (!mEnabled) return;

        double pid =  mController.calculate(inputs.pivotPositionRad);
        TrapezoidProfile.State setpoint = mController.getSetpoint();
        double ff = mFeedforward.calculate(setpoint.position, setpoint.velocity);

        io.runPivot(pid + ff);
    }

    public void runCharacterization(double volts) {
        io.runPivot(volts);
    }

}
