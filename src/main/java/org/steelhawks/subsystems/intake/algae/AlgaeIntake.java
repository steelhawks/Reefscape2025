package org.steelhawks.subsystems.intake.algae;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.intake.IntakeConstants;

public class AlgaeIntake {

    private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
    private final AlgaeIntakeIO io;
    private boolean mEnabled;

    private final ProfiledPIDController mController;
    private ArmFeedforward mFeedforward;

    public void enable() {
        mController.reset(inputs.pivotPositionRad);
        mEnabled = true;
    }

    public void disable() {
        mEnabled = false;
    }

    public AlgaeIntake(AlgaeIntakeIO io) {
        this.io = io;

        mController =
            new ProfiledPIDController(
                IntakeConstants.ALGAE_KP.getAsDouble(),
                IntakeConstants.ALGAE_KI.getAsDouble(),
                IntakeConstants.ALGAE_KD.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    IntakeConstants.ALGAE_MAX_VELOCITY_PER_SEC.getAsDouble(),
                    IntakeConstants.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED.getAsDouble()));

        mController.setTolerance(IntakeConstants.ALGAE_TOLERANCE.getAsDouble());

        mFeedforward =
            new ArmFeedforward(
                IntakeConstants.ALGAE_KS.getAsDouble(),
                IntakeConstants.ALGAE_KG.getAsDouble(),
                IntakeConstants.ALGAE_KV.getAsDouble());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeIntake", inputs);

        if (IntakeConstants.ALGAE_KP.hasChanged(hashCode()) ||
            IntakeConstants.ALGAE_KI.hasChanged(hashCode()) ||
            IntakeConstants.ALGAE_KD.hasChanged(hashCode()) ||
            IntakeConstants.ALGAE_MAX_VELOCITY_PER_SEC.hasChanged(hashCode()) ||
            IntakeConstants.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED.hasChanged(hashCode())
        ) {
            disable();
            mController.setPID(
                IntakeConstants.ALGAE_KP.getAsDouble(),
                IntakeConstants.ALGAE_KI.getAsDouble(),
                IntakeConstants.ALGAE_KD.getAsDouble());

            enable();
        }

        if (IntakeConstants.ALGAE_KS.hasChanged(hashCode()) ||
            IntakeConstants.ALGAE_KG.hasChanged(hashCode()) ||
            IntakeConstants.ALGAE_KV.hasChanged(hashCode())
        ) {
            mFeedforward = new ArmFeedforward(
                IntakeConstants.ALGAE_KS.getAsDouble(),
                IntakeConstants.ALGAE_KG.getAsDouble(),
                IntakeConstants.ALGAE_KV.getAsDouble());
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
