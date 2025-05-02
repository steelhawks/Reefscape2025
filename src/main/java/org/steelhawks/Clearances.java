package org.steelhawks;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.subsystems.algaeclaw.AlgaeClaw;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.FieldBoundingBox;

public class Clearances {

    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Claw s_Claw = RobotContainer.s_Claw;
    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final AlgaeClaw s_AlgaeClaw = RobotContainer.s_AlgaeClaw;

    public final static class ClawClearances {
        private static final double DIST_TO_BE_CLEAR_FROM_REEF = Units.inchesToMeters(3.0);
        public static boolean hasShot = false;

//        public static boolean isClearFromReef() {
//            return s_Claw.hasCoral().getAsBoolean() || !hasShot;
//        }

        public static boolean isClearFromReef() {
            return ReefUtil.getClosestAlgae().getRetrievePose().getTranslation()
                .getDistance(s_Swerve.getPose().getTranslation()) >= DIST_TO_BE_CLEAR_FROM_REEF;
        }
    }

    public final static class AlgaeClawClearances {
        private static final double DIST_TO_BE_CLEAR_FROM_REEF =
            ReefUtil.Algae.TR.getClearancePose().getTranslation()
                .minus(ReefUtil.Algae.TR.getRetrievePose().getTranslation())
                .getNorm();
        private static final double MIN_ANGLE_CLEAR_FROM_HOME = -1.1274758790959463;
        private static final double ELEVATOR_COLLIDE_ANGLE = -0.5;
        private static final double[] COLLISION_INTERVAL =
            new double[] {
                20, 30
            };

        public static boolean isClearFromReef() {
            return ReefUtil.getClosestAlgae().getRetrievePose().getTranslation()
                .getDistance(s_Swerve.getPose().getTranslation()) >= DIST_TO_BE_CLEAR_FROM_REEF;
        }

        public static boolean willCollideIntoElevator() {
            return s_AlgaeClaw.getPivotPosition() < MIN_ANGLE_CLEAR_FROM_HOME && s_Elevator.getPosition() < ELEVATOR_COLLIDE_ANGLE;
        }

        public static boolean safeToGoHome() {
            return s_AlgaeClaw.getPivotPosition() > COLLISION_INTERVAL[0] && s_AlgaeClaw.getPivotPosition() < COLLISION_INTERVAL[1];
        }

        public static boolean isClearFromElevatorCrossbeam() {
            return s_AlgaeClaw.getPivotPosition() >= MIN_ANGLE_CLEAR_FROM_HOME;
        }

        public static boolean isClearFromBarge() {
            return !new FieldBoundingBox(
                "Under Barge",
                new Translation2d(8.294717, 7.831129),
                new Translation2d(9.253538, 4.242398),
                s_Swerve::getPose).getAsBoolean();
        }
    }

    public final static class ClimbClearances {
        private static final double MIN_HEIGHT_TO_CLEAR = 1.1259418983080607;

        public static boolean isClearFromClaw() {
            return s_Elevator.getPosition() >= MIN_HEIGHT_TO_CLEAR;
        }
    }

}
