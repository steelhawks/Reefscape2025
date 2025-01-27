package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import org.steelhawks.generated.TunerConstants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

public final class Constants {

    public static final boolean TUNING_MODE = false;
    public static final boolean IN_REPLAY_MODE = false;

    public static final PowerDistribution.ModuleType PD_MODULE_TYPE = PowerDistribution.ModuleType.kRev;
    public static final CANBus CAN_BUS = new CANBus(TunerConstants.DrivetrainConstants.CANBusName);
    public static final String ROBOT_NAME = "ReefscapePreAlpha";
    public static final double SIM_UPDATE_LOOP = 0.020;

    private static final Mode SIM_MODE = IN_REPLAY_MODE ? Mode.REPLAY : Mode.SIM;
    /**
     * This defines the runtime mode used by AdvantageKit. The mode is always "real" when running
     * on a roboRIO. Change the value of "SIM_MODE" to switch between "sim" (physics sim) and "replay"
     * (log replay from a file).
     */
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    /**
     * Constants for the operator interface.
     */
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    /**
     * Constants for the field.
     */
    public static final class FieldConstants {

        public static final Distance FIELD_LENGTH = Feet.of(57 + (6.0 / 12.0) + ((7.0 / 8.0) / 12.0));
        public static final Distance FIELD_WIDTH = Feet.of(26 + (5.0 / 12.0));

        /*
         * To properly use the auto flip feature, the poses MUST be for the blue alliance.
         * The auto flip feature will automatically flip the poses for the red alliance.
         */
        public static final Pose2d BLUE_STARTING_POSE =
            new Pose2d(new Translation2d(0, 0), new Rotation2d());
        public static final Pose2d REEF_POSE =
            new Pose2d(4.459432, 4.023238, new Rotation2d());

        // Reef Sections
        public static final Pose2d LEFT_SECTION =
            new Pose2d(3.657600, 4.025900, new Rotation2d(Math.PI));
        public static final Pose2d TOP_LEFT_SECTION =
            new Pose2d(4.073906, 4.745481, new Rotation2d(2 * Math.PI / 3));
        public static final Pose2d BOTTOM_LEFT_SECTION =
            new Pose2d(4.073906, 3.306318, new Rotation2d(-2 * Math.PI / 3));

        public static final Pose2d RIGHT_SECTION =
            new Pose2d(5.321046, 4.025900, new Rotation2d());
        public static final Pose2d TOP_RIGHT_SECTION =
            new Pose2d(4.904739, 4.745481, new Rotation2d(Math.PI / 3));
        public static final Pose2d BOTTOM_RIGHT_SECTION =
            new Pose2d(4.904739, 3.306318, new Rotation2d(-Math.PI / 3));
    }

    /**
     * Constants for the physical auton selector.
     */
    public static final class SelectorConstants {}

    /**
     * Constants for controller deadbands.
     */
    public static final class Deadbands {
        public static final double DRIVE_DEADBAND = 0.3;
    }

    /**
     * Constants for autonomous driving functions.
     */
    public static final class AutonConstants {

        public static final double TRANSLATION_KP = 5;
        public static final double TRANSLATION_KI = 0;
        public static final double TRANSLATION_KD = 0;

        public static final double ROTATION_KP = 5;
        public static final double ROTATION_KI = 0;
        public static final double ROTATION_KD = 0;

        public static final double AUTO_ALIGN_KP = 3.0;
        public static final double AUTO_ALIGN_KI = 0.0;
        public static final double AUTO_ALIGN_KD = 0.0;
        public static final double ANGLE_MAX_VELOCITY = 15.0;
        public static final double ANGLE_MAX_ACCELERATION = 20.0;

        // Pathfinder
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.0;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.0;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 0.0;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 0.0;

        public static final PathConstraints CONSTRAINTS =
            new PathConstraints(
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    /**
     * Function to validate that all constants are set.
     */
    @SuppressWarnings("unused")
    public static void validate() {

        if (CURRENT_MODE == Mode.REAL) {
            if (CAN_BUS == null) {
                throw new IllegalStateException("CAN Bus must be set in Constants.java");
            }
        }

        if (ROBOT_NAME == "NAME YOUR ROBOT") {
            throw new IllegalStateException("Robot name must be set in Constants.java");
        }

        if (FieldConstants.FIELD_LENGTH.in(Meters) == 0) {
            throw new IllegalStateException("Field length must be set in Constants.java");
        }

        if (AutonConstants.TRANSLATION_KP == 0
            || AutonConstants.TRANSLATION_KI == 0
            || AutonConstants.TRANSLATION_KD == 0) {
            throw new IllegalStateException("Translation PID must be set in Constants.java");
        }

        if (AutonConstants.ROTATION_KP == 0
            || AutonConstants.ROTATION_KI == 0
            || AutonConstants.ROTATION_KD == 0) {
            throw new IllegalStateException("Rotation PID must be set in Constants.java");
        }

        if (AutonConstants.AUTO_ALIGN_KP == 0
            || AutonConstants.AUTO_ALIGN_KI == 0
            || AutonConstants.AUTO_ALIGN_KD == 0) {
            throw new IllegalStateException("Auto Align PID must be set in Constants.java");
        }

        if (AutonConstants.MAX_VELOCITY_METERS_PER_SECOND == 0
            || AutonConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED == 0
            || AutonConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND == 0
            || AutonConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED == 0) {
            throw new IllegalStateException("Pathfinder constraints must be set in Constants.java");
        }
    }
}
