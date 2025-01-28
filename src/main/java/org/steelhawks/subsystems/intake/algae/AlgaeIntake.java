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
    private final ArmFeedforward mFeedforward;

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
                KIntake.ALGAE_KP,
                KIntake.ALGAE_KI,
                KIntake.ALGAE_KD,
                new TrapezoidProfile.Constraints(
                    KIntake.ALGAE_MAX_VELO,
                    KIntake.ALGAE_MAX_ACCEL));

        mFeedforward =
            new ArmFeedforward(
                KIntake.ALGAE_KS,
                KIntake.ALGAE_KG,
                KIntake.ALGAE_KV);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeIntake", inputs);

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
