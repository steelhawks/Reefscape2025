package org.steelhawks.subsystems.claw;

import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.steelhawks.subsystems.swerve.Swerve;

public class ClawIOSim implements ClawIO {

    public static IntakeSimulation mIntakeSim = null;

    public ClawIOSim() {
        mIntakeSim =
            new IntakeSimulation(
                "Coral",
                Swerve.getDriveSimulation(),
                new Triangle(new Vector2(0.0, 0.0), new Vector2(0.2, 0.0), new Vector2(0.0, 0.2)),
                1);
    }

    @Override
    public void updateInputs(ClawIntakeIOInputs inputs) {
        inputs.connected = true;
        inputs.positionRad = 0.0;
        inputs.velocityRadPerSec = 0.0;
        inputs.appliedVolts = 0.0;
        inputs.currentAmps = 0.0;
        inputs.tempCelsius = 0.0;

    }

    @Override
    public void runIntake(double percentageOutput) {
        mIntakeSim.startIntake();
    }

    @Override
    public void stop() {
        mIntakeSim.stopIntake();
    }
}
