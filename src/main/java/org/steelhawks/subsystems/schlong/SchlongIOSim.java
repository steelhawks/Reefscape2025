package org.steelhawks.subsystems.schlong;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SchlongIOSim implements SchlongIO {
    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d pivotArm;
    private double pivotPosition = 0.0;
    private double pivotSpeed = 0.0;
    private double spinSpeed = 0.0;

    public SchlongIOSim() {
        mechanism = new LoggedMechanism2d(2, 2);
        root = mechanism.getRoot("Base", 1, 1);
        pivotArm = root.append(new LoggedMechanismLigament2d("PivotArm", 0.5, 0));
        Logger.recordOutput("Schlong/Mechanism", mechanism);
    }

    @Override
    public void updateInputs(SchlongIOInputs inputs) {
        pivotPosition += pivotSpeed * 0.02;
        pivotPosition = Math.max(0, Math.min(Math.PI / 2, pivotPosition));
        pivotArm.setAngle(new Rotation2d(pivotPosition));

        inputs.pivotPositionRad = pivotPosition;
        inputs.pivotVelocityRadPerSec = pivotSpeed;
        inputs.spinVelocityRadPerSec = spinSpeed;
        inputs.pivotConnected = true;
        inputs.spinConnected = true;
        inputs.limitSwitchConnected = pivotPosition >= (Math.PI / 2);
    }

    @Override
    public void runSpinWithSpeed(double speed) {
        spinSpeed = speed;
    }

    @Override
    public void runSpinWithVoltage(double volts) {
        spinSpeed = volts / 12.0;
    }

    @Override
    public void stopSpin() {
        spinSpeed = 0;
    }

    @Override
    public void runPivotWithSpeed(double speed) {
        pivotSpeed = speed;
    }

    @Override
    public void runPivotWithVoltage(double volts) {
        pivotSpeed = volts / 12.0;
    }

    @Override
    public void stopPivot() {
        pivotSpeed = 0;
    }

    @Override
    public void zeroEncoders() {
        pivotPosition = 0;
    }
}