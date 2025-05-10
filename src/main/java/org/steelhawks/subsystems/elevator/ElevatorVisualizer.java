package org.steelhawks.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import java.util.function.Supplier;

public class ElevatorVisualizer {

    private final Supplier<Double> mElevatorPosition;
    private final LoggedMechanism2d mElevator;

    private final LoggedMechanismLigament2d leftLig, rightLig;

    public ElevatorVisualizer(
        Supplier<Double> elevatorPosition, double elevatorWidth, double elevatorMaxHeight) {
        this.mElevatorPosition = elevatorPosition;

        mElevator =
            new LoggedMechanism2d(
                elevatorWidth,
                elevatorMaxHeight);
        double width = 0.1;

        var leftRoot = mElevator.getRoot("Left Root", 0.33 - width / 2, 0);
        var rightRoot = mElevator.getRoot("Right Root", 0.33 + width / 2, 0);

        leftLig = leftRoot.append(
            new LoggedMechanismLigament2d("Left Side", mElevatorPosition.get(), 90));
        rightLig = rightRoot.append(
            new LoggedMechanismLigament2d("Right Side", mElevatorPosition.get(), 90));
    }

    public void update() {
        leftLig.setAngle(90);
        leftLig.setLength(mElevatorPosition.get());
        rightLig.setAngle(90);
        rightLig.setLength(mElevatorPosition.get());

        Logger.recordOutput("Elevator/Mechanism2d", mElevator);
    }
}
