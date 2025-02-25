package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.AllianceFlip;

public final class Constants {

    public static final double ENDGAME_PERIOD = 20;
    public static final double MATCH_TIME_SECONDS = 150;

    public static final boolean USE_MOTION_MAGIC = false;
    public static final boolean TUNING_MODE = false;

    public static final PowerDistribution.ModuleType PD_MODULE_TYPE = PowerDistribution.ModuleType.kRev;
    public static final double SIM_UPDATE_LOOP = 0.020;

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public enum RobotType {
        OMEGABOT,
        ALPHABOT,
        HAWKRIDER,
        SIMBOT
    }

    // Change this based on what robot is being used.
    private static final RobotType ROBOT = RobotType.HAWKRIDER;

    /**
     * The robot type.
     *
     * <p>
     *     To run a physics simulator make sure you set it to RobotType.SIMBOT
     *     </p>If you want to replay a log file set it to the robot type you want to replay and just run the simulator.
     * </p>
     */
    private static final RobotType ROBOT_TYPE =
        isCI() ?
            RobotType.SIMBOT : // set to simbot when doing CI check on GitHub
            ROBOT; // actual mode you want

    private static boolean isCI() {
        return System.getenv("CI") != null;
    }

    public static final String ROBOT_NAME =
        switch (ROBOT) {
            case OMEGABOT -> "Omega";
            case ALPHABOT -> "Alpha";
            case HAWKRIDER -> "Hawk Rider";
            case SIMBOT -> "Simulation";
        };

    public static Mode getMode() {
        return switch (ROBOT_TYPE) {
            case ALPHABOT, OMEGABOT, HAWKRIDER ->
                RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMBOT -> Mode.SIM;
        };
    }

    public static RobotType getRobot() {
        if (RobotBase.isReal() && ROBOT_TYPE == RobotType.SIMBOT) {
            new Alert("Invalid robot selected, using omega robot as default.", AlertType.kError)
                .set(true);
            return RobotType.OMEGABOT;
        }

        return ROBOT_TYPE;
    }

    public static CANBus getCANBus() {
        return switch (getRobot()) {
            case OMEGABOT, HAWKRIDER, SIMBOT -> new CANBus("canivore");
            case ALPHABOT -> new CANBus("");
        };
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

//        public static final Distance FIELD_LENGTH = Inches.of(690.876);
//        public static final Distance FIELD_WIDTH = Inches.of(317);

        public static final double FIELD_LENGTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldLength();
        public static final double FIELD_WIDTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldWidth();

        /*
         * To properly use the auto flip feature, the poses MUST be for the blue alliance.
         * The auto flip feature will automatically flip the poses for the red alliance.
         */
        public static final Pose2d BLUE_STARTING_POSE =
            new Pose2d(new Translation2d(0, 0), new Rotation2d());
        public static final Pose2d REEF_POSE =
            new Pose2d(4.459432, 4.023238, new Rotation2d());

        // Reef Sections
//        public static final Pose2d LEFT_SECTION =
//            new Pose2d(3.657600, 4.025900, new Rotation2d(Math.PI));
//        public static final Pose2d TOP_LEFT_SECTION =
//            new Pose2d(4.073906, 4.745481, new Rotation2d(2 * Math.PI / 3));
//        public static final Pose2d BOTTOM_LEFT_SECTION =
//            new Pose2d(4.073906, 3.306318, new Rotation2d(-2 * Math.PI / 3));

//        public static final Pose2d RIGHT_SECTION =
//            new Pose2d(5.321046, 4.025900, new Rotation2d());
//        public static final Pose2d TOP_RIGHT_SECTION =
//            new Pose2d(4.904739, 4.745481, new Rotation2d(Math.PI / 3));
//        public static final Pose2d BOTTOM_RIGHT_SECTION =
//            new Pose2d(4.904739, 3.306318, new Rotation2d(-Math.PI / 3));

//        public static final Pose2d PROCESSOR =
//            new Pose2d(5.987542, 0.407114, new Rotation2d(Math.PI / 2));

//        public static final Pose2d CORAL_STATION_BOTTOM =
//            new Pose2d(1.007676, 1, new Rotation2d(0.94 + Math.PI));
        // y: 0.955011
        // .94rad, -.94rad

//        public static final Pose2d CORAL_STATION_TOP =
//            new Pose2d(1.007676, 6.96480, new Rotation2d(-0.94 + Math.PI));

        public enum Position {
            LEFT_SECTION(new Pose2d(3.657600, 4.025900, new Rotation2d(Math.PI))),
            TOP_LEFT_SECTION(new Pose2d(4.073906, 4.745481, new Rotation2d(2 * Math.PI / 3))),
            BOTTOM_LEFT_SECTION(new Pose2d(4.073906, 3.306318, new Rotation2d(-2 * Math.PI / 3))),
            RIGHT_SECTION(new Pose2d(5.321046, 4.025900, new Rotation2d())),
            TOP_RIGHT_SECTION(new Pose2d(4.904739, 4.745481, new Rotation2d(Math.PI / 3))),
            BOTTOM_RIGHT_SECTION(new Pose2d(4.904739, 3.306318, new Rotation2d(-Math.PI / 3))),
            PROCESSOR(new Pose2d(5.987542, 0.407114, new Rotation2d(Math.PI / 2))),
            CORAL_STATION_BOTTOM(new Pose2d(1.007676, 1, new Rotation2d(0.94 + Math.PI))),
            CORAL_STATION_TOP(new Pose2d(1.007676, 6.96480, new Rotation2d(-0.94 + Math.PI)));

            private final Pose2d pose;

            Position(Pose2d pose) {
                this.pose = pose;
            }

            public Pose2d getPose() {
                return pose;
            }
        }
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
        public static final double ELEVATOR_DEADBAND = 0.05;
        public static final double PIVOT_DEADBAND = 0.1;
    }

    public static final class LEDConstants {

        public static final LEDConstants DEFAULT =
            new LEDConstants(0, 40);

        public static final LEDConstants OMEGA =
            new LEDConstants(0, 82);
        public static final LEDConstants ALPHA = DEFAULT;
        public static final LEDConstants HAWKRIDER = DEFAULT;

        public final int PORT;
        public final int LENGTH;

        private LEDConstants(
            int port, int length
        ) {
            PORT = port;
            LENGTH = length;
        }
    }

    /**
     * Constants for autonomous driving functions.
     */
    public static final class AutonConstants {

        public static final AutonConstants OMEGA =
            new AutonConstants(
                5,
                0.0,
                0.0,
                5,
                0.0,
                0.0,
                0.0,
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
            10,
            5.0,
            0.0,
            5,
            0.0,
            0.0,
            3,
            0.0,
            0.0,
            15,
            20,
            1,
            2,
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
}
