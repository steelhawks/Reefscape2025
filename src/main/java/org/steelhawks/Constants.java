package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public static final double ENDGAME_PERIOD = 20;
    public static final double MATCH_TIME_SECONDS = 150;

    public static final boolean USE_MOTION_MAGIC = false;
    public static final boolean TUNING_MODE = false;

    public static final int POWER_DISTRIBUTION_CAN_ID =
        getRobot() == RobotType.ALPHABOT
            ? 0
            : 1;
    public static final PowerDistribution.ModuleType PD_MODULE_TYPE =
        getRobot() == RobotType.ALPHABOT
            ? PowerDistribution.ModuleType.kCTRE
            : PowerDistribution.ModuleType.kRev;
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
    private static final RobotType ROBOT = RobotType.SIMBOT;

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

    public static final class RobotConstants {
        public static final double ROBOT_LENGTH_WITH_BUMPERS = Units.inchesToMeters(36.0);

        // for distance between robot center and claw
        public static final double CLAW_OFFSET = -Units.inchesToMeters(9.836467);
        public static final double CLAW_OFFSET_SMALL_COMPONENT = 0.1249231309;
        public static final double CLAW_OFFSET_BIG_COMPONENT = 0.21637320975937756890536690206266;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class Deadbands {
        public static final double DRIVE_DEADBAND = 0.3;
        public static final double ELEVATOR_DEADBAND = 0.05;
        public static final double PIVOT_DEADBAND = 0.1;
        public static final double REVERSE_CORAL_DEADBAND = 0.1;
    }

    public static final class LEDConstants {
        public static final int PORT;
        public static final int LENGTH;

        static {
            switch (getRobot()) {
                case ALPHABOT, HAWKRIDER -> {
                    PORT = 0;
                    LENGTH = 40;
                }
                default -> {
                    PORT = 0;
                    LENGTH = 42;
                }
            }
        }
    }

    public static final class AutonConstants {
        public static final AutonConstants OMEGA =
            new AutonConstants(
                5.0,
                0.0,
                0.0,
                5.0,
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
            4,
            5,
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
