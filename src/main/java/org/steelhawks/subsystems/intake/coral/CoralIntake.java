package org.steelhawks.subsystems.intake.coral;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.intake.IntakeConstants;

public class CoralIntake {

    private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
    private final CoralIntakeIO io;
    private boolean mEnabled;

    public void enable() {
        mEnabled = true;
    }

    public void disable() {
        mEnabled = false;
    }

    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralIntake", inputs);

        if (!mEnabled) return;

        io.runIntake();
    }
}
