package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import org.steelhawks.subsystems.elevator.ElevatorConstants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

public final class Constants {

    public static final boolean TUNING_MODE = false;
    public static final boolean IN_REPLAY_MODE = false;
    public static final boolean USE_MOTION_MAGIC = true;

    public static final PowerDistribution.ModuleType PD_MODULE_TYPE = PowerDistribution.ModuleType.kRev;
    public static final String ROBOT_NAME = "ReefscapePreAlpha";
    public static final double SIM_UPDATE_LOOP = 0.020;

    private static final Mode SIM_MODE = IN_REPLAY_MODE ? Mode.REPLAY : Mode.SIM;
    /**
     * This defines the runtime mode used by AdvantageKit. The mode is always "real" when running
     * on a roboRIO. Change the value of "SIM_MODE" to switch between "sim" (physics sim) and "replay"
     * (log replay from a file).
     */
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public enum RobotType {
        ALPHABOT,
        OMEGABOT,
        HAWKRIDER,
        SIMBOT
    }

    public static final RobotType ROBOT_TYPE = RobotType.ALPHABOT;

    public static RobotType getRobot() {
        if (CURRENT_MODE == Mode.REAL) {
            return ROBOT_TYPE;
        }

        return RobotType.SIMBOT;
    }

    public static CANBus getCANBus() {
        return switch (getRobot()) {
            case ALPHABOT, SIMBOT -> new CANBus("");
            case OMEGABOT -> new CANBus();
            case HAWKRIDER -> new CANBus("canivore");
        };
    }

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

        public static final AutonConstants OMEGA = new AutonConstants(
            5,
            0.0,
            0.0,
            5,
            0.0,
            0.0,
            3,
            0.0,
            0.0,
            15,
            20,
            3,
            4,
            5,
            8);

        public static final AutonConstants ALPHA = new AutonConstants(
            5,
            0.0,
            0.0,
            5,
            0.0,
            0.0,
            3,
            0.0,
            0.0,
            15,
            20,
            3,
            4,
            5,
            8);

        public static final AutonConstants HAWKRIDER = new AutonConstants(
            5,
            0.0,
            0.0,
            5,
            0.0,
            0.0,
            3,
            0.0,
            0.0,
            15,
            20,
            3,
            4,
            5,
            8);

        public final double TRANSLATION_KP;
        public final double TRANSLATION_KI;
        public final double TRANSLATION_KD;

        public final double ROTATION_KP;
        public final double ROTATION_KI;
        public final double ROTATION_KD;

        public final double AUTO_ALIGN_KP;
        public final double AUTO_ALIGN_KI;
        public final double AUTO_ALIGN_KD;
        public final double ANGLE_MAX_VELOCITY;
        public final double ANGLE_MAX_ACCELERATION;

        // Pathfinder
        public final double MAX_VELOCITY_METERS_PER_SECOND;
        public final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
        public final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        public final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED;

        public final PathConstraints CONSTRAINTS;

        public AutonConstants(
            double translationKp,
            double translationKi,
            double translationKd,
            double rotationKp,
            double rotationKi,
            double rotationKd,
            double autoAlignKp,
            double autoAlignKi,
            double autoAlignKd,
            double angleMaxVelocity,
            double angleMaxAcceleration,
            double maxVelocityMetersPerSecond,
            double maxAccelerationMetersPerSecondSquared,
            double maxAngularVelocityRadiansPerSecond,
            double maxAngularAccelerationRadiansPerSecondSquared
        ) {
            TRANSLATION_KP = translationKp;
            TRANSLATION_KI = translationKi;
            TRANSLATION_KD = translationKd;

            ROTATION_KP = rotationKp;
            ROTATION_KI = rotationKi;
            ROTATION_KD = rotationKd;

            AUTO_ALIGN_KP = autoAlignKp;
            AUTO_ALIGN_KI = autoAlignKi;
            AUTO_ALIGN_KD = autoAlignKd;

            ANGLE_MAX_VELOCITY = angleMaxVelocity;
            ANGLE_MAX_ACCELERATION = angleMaxAcceleration;

            MAX_VELOCITY_METERS_PER_SECOND = maxVelocityMetersPerSecond;
            MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = maxAccelerationMetersPerSecondSquared;
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = maxAngularVelocityRadiansPerSecond;
            MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = maxAngularAccelerationRadiansPerSecondSquared;

            CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY_METERS_PER_SECOND,
                    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
                    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
        }
    }

    public static void main(String[] args) {
        if (CURRENT_MODE == Mode.SIM && RobotBase.isReal()) {
            throw new IllegalStateException("Robot is in sim mode but running on real hardware. Check your code.");
        }
    }
}
