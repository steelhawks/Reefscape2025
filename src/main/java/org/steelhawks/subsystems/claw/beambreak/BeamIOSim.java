package org.steelhawks.subsystems.claw.beambreak;

import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.util.FieldBoundingBox;

import java.util.function.BooleanSupplier;

public class BeamIOSim implements BeamIO {
    private static boolean hasCoral = false;
    private static boolean wasCoralDetected = false;

    private final BooleanSupplier topCoralStationTrigger =
        new FieldBoundingBox(
            "Top Coral Station",
            0.0, 2.0, 6.2, 8.0,
            RobotContainer.s_Swerve::getPose);
    private final BooleanSupplier bottomCoralStationTrigger =
        new FieldBoundingBox(
            "Bottom Coral Station",
            0.0, 2.0, 0.0, 8.0 - 6.2,
            RobotContainer.s_Swerve::getPose);

    @Override
    public void updateInputs(BeamIOInputs inputs) {
        boolean currentlyDetected = topCoralStationTrigger.getAsBoolean() || bottomCoralStationTrigger.getAsBoolean() || Robot.isFirstRun();
        if (currentlyDetected) {
            wasCoralDetected = true;
        }
        BeamIOSim.hasCoral = wasCoralDetected;

        inputs.connected = true;
        inputs.distance = BeamIOSim.hasCoral ? Claw.DIST_TO_HAVE_CORAL - 0.05 : 10.0;
        inputs.broken = BeamIOSim.hasCoral;
    }

    public static void hasShot() {
        hasCoral = false;
        wasCoralDetected = false;
    }
}
