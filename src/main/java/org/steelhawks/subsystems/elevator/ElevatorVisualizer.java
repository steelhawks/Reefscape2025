package org.steelhawks.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import java.util.function.Supplier;

public class ElevatorVisualizer {

    private final Supplier<Double> mElevatorPosition;
    private final LoggedMechanism2d mElevator;

    private final LoggedMechanismRoot2d mMech2dRoot;
    private final LoggedMechanismLigament2d mElevatorMech2d;

    public ElevatorVisualizer(
        Supplier<Double> elevatorPosition, double elevatorWidth, double elevatorMaxHeight) {
        this.mElevatorPosition = elevatorPosition;

        mElevator =
            new LoggedMechanism2d(
                elevatorWidth,
                elevatorMaxHeight);
        mMech2dRoot = mElevator.getRoot("Elevator Root", 0.33, 0);
        mElevatorMech2d =
            mMech2dRoot.append(
                new LoggedMechanismLigament2d("Elevator", mElevatorPosition.get(), 90));
    }

    public void update() {
        mElevatorMech2d.setAngle(90);
        mElevatorMech2d.setLength(mElevatorPosition.get());

        Logger.recordOutput("Elevator/Mechanism2d", mElevator);
    }
}
