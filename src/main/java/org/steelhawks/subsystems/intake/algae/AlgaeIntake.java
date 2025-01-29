package org.steelhawks.subsystems.intake.algae;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.intake.KIntake;

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
                KIntake.ALGAE_KP.getAsDouble(),
                KIntake.ALGAE_KI.getAsDouble(),
                KIntake.ALGAE_KD.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    KIntake.ALGAE_MAX_VELOCITY_PER_SEC.getAsDouble(),
                    KIntake.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED.getAsDouble()));

        mController.setTolerance(KIntake.TOLERANCE.getAsDouble());

        mFeedforward =
            new ArmFeedforward(
                KIntake.ALGAE_KS.getAsDouble(),
                KIntake.ALGAE_KG.getAsDouble(),
                KIntake.ALGAE_KV.getAsDouble());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeIntake", inputs);

        if (KIntake.ALGAE_KP.hasChanged(hashCode()) ||
            KIntake.ALGAE_KI.hasChanged(hashCode()) ||
            KIntake.ALGAE_KD.hasChanged(hashCode()) ||
            KIntake.ALGAE_MAX_VELOCITY_PER_SEC.hasChanged(hashCode()) ||
            KIntake.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED.hasChanged(hashCode())
        ) {
            disable();
            mController.setPID(
                KIntake.ALGAE_KP.getAsDouble(),
                KIntake.ALGAE_KI.getAsDouble(),
                KIntake.ALGAE_KD.getAsDouble());

            enable();
        }

        if (KIntake.ALGAE_KS.hasChanged(hashCode()) ||
            KIntake.ALGAE_KG.hasChanged(hashCode()) ||
            KIntake.ALGAE_KV.hasChanged(hashCode())
        ) {
            mFeedforward = new ArmFeedforward(
                KIntake.ALGAE_KS.getAsDouble(),
                KIntake.ALGAE_KG.getAsDouble(),
                KIntake.ALGAE_KV.getAsDouble());
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
