package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
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
    private static final RobotType ROBOT = RobotType.OMEGABOT;

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
        public static final double ROBOT_LENGTH_WITH_BUMPERS = Units.inchesToMeters(30.0 + (3.125 * 2.0));

        // for distance between robot center and claw
        public static final double CLAW_OFFSET = -Units.inchesToMeters(9.836467);
        public static final double CLAW_OFFSET_SMALL_COMPONENT = 0.1249231309;
        public static final double CLAW_OFFSET_BIG_COMPONENT = 0.21637320975937756890536690206266;

        public static final double SCHLONG_OFFSET = -Units.inchesToMeters(9.836467);
        public static final double DIST_CLEAR_FROM_REEF = Units.inchesToMeters(4);
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
        private static final double TRANSLATION_KP;
        private static final double TRANSLATION_KI;
        private static final double TRANSLATION_KD;
        public static final PIDConstants TRANSLATION_PID;

        private static final double ROTATION_KP;
        private static final double ROTATION_KI;
        private static final double ROTATION_KD;
        public static final PIDConstants ROTATION_PID;

        // Pathfinder
        public static final double MAX_VELOCITY_METERS_PER_SECOND;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED;

        public static final PathConstraints CONSTRAINTS;

        static {
            switch (getRobot()) {
                case ALPHABOT -> {
                    TRANSLATION_KP = 5.0;
                    TRANSLATION_KI = 0.0;
                    TRANSLATION_KD = 0.0;
                    ROTATION_KP = 5.0;
                    ROTATION_KI = 0.0;
                    ROTATION_KD = 0.0;
                    MAX_VELOCITY_METERS_PER_SECOND = 4.0;
                    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5.0;
                    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 5.0;
                    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 8.0;
                }
                case HAWKRIDER -> {
                    TRANSLATION_KP = 10.0;
                    TRANSLATION_KI = 0.0;
                    TRANSLATION_KD = 0.0;
                    ROTATION_KP = 5.0;
                    ROTATION_KI = 0.0;
                    ROTATION_KD = 0.0;
                    MAX_VELOCITY_METERS_PER_SECOND = 1.0;
                    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
                    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 5.0;
                    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 8.0;
                }
                default -> {
                    TRANSLATION_KP = 3.5;
                    TRANSLATION_KI = 0.0;
                    TRANSLATION_KD = 0.2;
                    ROTATION_KP = 3.0;
                    ROTATION_KI = 0.0;
                    ROTATION_KD = 0.1;
                    MAX_VELOCITY_METERS_PER_SECOND = 3.0;
                    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.5;
                    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 6.0;
                    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 15.0;
                }
            }

            CONSTRAINTS = new PathConstraints(
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
            TRANSLATION_PID = new PIDConstants(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);
            ROTATION_PID = new PIDConstants(ROTATION_KP, ROTATION_KI, ROTATION_KD);
        }
    }
}
