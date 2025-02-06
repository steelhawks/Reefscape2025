package org.steelhawks.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.steelhawks.Constants.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Constants.Mode;
import org.steelhawks.RobotContainer;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsHawkRider;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.HolonomicController;
import org.steelhawks.util.LocalADStarAK;

public class Swerve extends SubsystemBase {

    private static final double SLOW_SPEED_MULTIPLIER = 0.2;
    private static double SPEED_MULTIPLIER = 1;
    private boolean isPathfinding = false;

    public static final double ODOMETRY_FREQUENCY =
        Constants.getCANBus().isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS;

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG;
    private static final double ROBOT_MOI;
    private static final double WHEEL_COF;
    private static final RobotConfig PP_CONFIG;

    public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG;

    public static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] swerveModules = new SwerveModule[4]; // FL, FR, BL, BR
    private final SysIdRoutine driveSysId;
    private final SysIdRoutine turnSysId;
    private final SysIdRoutine angularSysId;
    private final Alert gyroDisconnectedAlert =
        new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

    private final SwerveDrivePoseEstimator mPoseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    static {
        switch (Constants.getRobot()) {
            case OMEGABOT -> {
                DRIVE_BASE_RADIUS =
                    Math.max(
                        Math.max(
                            Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                            Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
                        Math.max(
                            Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                            Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
                ROBOT_MASS_KG = Units.lbsToKilograms(51.2);
                ROBOT_MOI = (1.0 / 12.0) * ROBOT_MASS_KG * (2 * Math.pow(25, 2));
                WHEEL_COF = COTS.WHEELS.COLSONS.cof;
                PP_CONFIG =
                    new RobotConfig(
                        ROBOT_MASS_KG,
                        ROBOT_MOI,
                        new ModuleConfig(
                            TunerConstants.FrontLeft.WheelRadius,
                            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DCMotor.getKrakenX60Foc(1)
                                .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                            TunerConstants.FrontLeft.SlipCurrent,
                            1),
                        getModuleTranslations());
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                        .withCustomModuleTranslations(getModuleTranslations())
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(
                            new SwerveModuleSimulationConfig(
                                DCMotor.getKrakenX60(1),
                                DCMotor.getKrakenX60(1),
                                TunerConstants.FrontLeft.DriveMotorGearRatio,
                                TunerConstants.FrontLeft.SteerMotorGearRatio,
                                Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                                Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                                Meters.of(TunerConstants.FrontLeft.WheelRadius),
                                KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                                WHEEL_COF));
            }
            case ALPHABOT -> {
                DRIVE_BASE_RADIUS =
                    Math.max(
                        Math.max(
                            Math.hypot(TunerConstantsAlpha.FrontLeft.LocationX, TunerConstantsAlpha.FrontLeft.LocationY),
                            Math.hypot(TunerConstantsAlpha.FrontRight.LocationX, TunerConstantsAlpha.FrontRight.LocationY)),
                        Math.max(
                            Math.hypot(TunerConstantsAlpha.BackLeft.LocationX, TunerConstantsAlpha.BackLeft.LocationY),
                            Math.hypot(TunerConstantsAlpha.BackRight.LocationX, TunerConstantsAlpha.BackRight.LocationY)));
                ROBOT_MASS_KG = Units.lbsToKilograms(138 + (6.0 / 16.0));
                ROBOT_MOI = (1.0 / 12.0) * ROBOT_MASS_KG * (2 * Math.pow(Units.inchesToMeters(30), 2));
                WHEEL_COF = COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof;
                PP_CONFIG =
                    new RobotConfig(
                        ROBOT_MASS_KG,
                        ROBOT_MOI,
                        new ModuleConfig(
                            TunerConstantsAlpha.FrontLeft.WheelRadius,
                            TunerConstantsAlpha.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DCMotor.getKrakenX60Foc(1)
                                .withReduction(TunerConstantsAlpha.FrontLeft.DriveMotorGearRatio),
                            TunerConstantsAlpha.FrontLeft.SlipCurrent,
                            1),
                        getModuleTranslations());
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                        .withCustomModuleTranslations(getModuleTranslations())
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(
                            new SwerveModuleSimulationConfig(
                                DCMotor.getKrakenX60(1),
                                DCMotor.getFalcon500(1),
                                TunerConstantsAlpha.FrontLeft.DriveMotorGearRatio,
                                TunerConstantsAlpha.FrontLeft.SteerMotorGearRatio,
                                Volts.of(TunerConstantsAlpha.FrontLeft.DriveFrictionVoltage),
                                Volts.of(TunerConstantsAlpha.FrontLeft.SteerFrictionVoltage),
                                Meters.of(TunerConstantsAlpha.FrontLeft.WheelRadius),
                                KilogramSquareMeters.of(TunerConstantsAlpha.FrontLeft.SteerInertia),
                                WHEEL_COF));
            }
            case HAWKRIDER -> {
                DRIVE_BASE_RADIUS =
                    Math.max(
                        Math.max(
                            Math.hypot(TunerConstantsHawkRider.FrontLeft.LocationX, TunerConstantsHawkRider.FrontLeft.LocationY),
                            Math.hypot(TunerConstantsHawkRider.FrontRight.LocationX, TunerConstantsHawkRider.FrontRight.LocationY)),
                        Math.max(
                            Math.hypot(TunerConstantsHawkRider.BackLeft.LocationX, TunerConstantsHawkRider.BackLeft.LocationY),
                            Math.hypot(TunerConstantsHawkRider.BackRight.LocationX, TunerConstantsHawkRider.BackRight.LocationY)));
                ROBOT_MASS_KG = Units.lbsToKilograms(138 + (6.0 / 16.0));
                ROBOT_MOI = (1.0 / 12.0) * ROBOT_MASS_KG * (2 * Math.pow(Units.inchesToMeters(30), 2));
                WHEEL_COF = COTS.WHEELS.COLSONS.cof;
                PP_CONFIG =
                    new RobotConfig(
                        ROBOT_MASS_KG,
                        ROBOT_MOI,
                        new ModuleConfig(
                            TunerConstantsHawkRider.FrontLeft.WheelRadius,
                            TunerConstantsHawkRider.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DCMotor.getKrakenX60Foc(1)
                                .withReduction(TunerConstantsHawkRider.FrontLeft.DriveMotorGearRatio),
                            TunerConstantsHawkRider.FrontLeft.SlipCurrent,
                            1),
                        getModuleTranslations());
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                        .withCustomModuleTranslations(getModuleTranslations())
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(
                            new SwerveModuleSimulationConfig(
                                DCMotor.getKrakenX60(1),
                                DCMotor.getFalcon500(1),
                                TunerConstantsHawkRider.FrontLeft.DriveMotorGearRatio,
                                TunerConstantsHawkRider.FrontLeft.SteerMotorGearRatio,
                                Volts.of(TunerConstantsHawkRider.FrontLeft.DriveFrictionVoltage),
                                Volts.of(TunerConstantsHawkRider.FrontLeft.SteerFrictionVoltage),
                                Meters.of(TunerConstantsHawkRider.FrontLeft.WheelRadius),
                                KilogramSquareMeters.of(TunerConstantsHawkRider.FrontLeft.SteerInertia),
                                WHEEL_COF));
            }
            default -> {
                DRIVE_BASE_RADIUS = 1.0;
                ROBOT_MASS_KG = 0;
                ROBOT_MOI = 0;
                WHEEL_COF = 0;
                PP_CONFIG =
                    new RobotConfig(
                        ROBOT_MASS_KG,
                        ROBOT_MOI,
                        new ModuleConfig(
                            TunerConstantsHawkRider.FrontLeft.WheelRadius,
                            TunerConstantsHawkRider.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DCMotor.getKrakenX60Foc(1)
                                .withReduction(TunerConstantsHawkRider.FrontLeft.DriveMotorGearRatio),
                            TunerConstantsHawkRider.FrontLeft.SlipCurrent,
                            1),
                        getModuleTranslations());
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default();
            }
        }
    }

    public Swerve(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        swerveModules[0] = new SwerveModule(flModuleIO, 0, TunerConstantsHawkRider.FrontLeft);
        swerveModules[1] = new SwerveModule(frModuleIO, 1, TunerConstantsHawkRider.FrontRight);
        swerveModules[2] = new SwerveModule(blModuleIO, 2, TunerConstantsHawkRider.BackLeft);
        swerveModules[3] = new SwerveModule(brModuleIO, 3, TunerConstantsHawkRider.BackRight);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        final AutonConstants constants;

        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = AutonConstants.ALPHA;
            case HAWKRIDER -> constants = AutonConstants.HAWKRIDER;
            default -> constants = AutonConstants.OMEGA;
        }

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(
                new PIDConstants(
                    constants.TRANSLATION_KP,
                    constants.TRANSLATION_KI, constants.TRANSLATION_KD),
                new PIDConstants(
                    constants.ROTATION_KP,
                    constants.ROTATION_KI, constants.ROTATION_KD)),
            PP_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) ->
                Logger.recordOutput(
                    "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) ->
                Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        driveSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Swerve/DriveSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runDriveCharacterization(voltage.in(Volts)), null, this));

        turnSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Swerve/TurnSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runTurnCharacterization(voltage.in(Volts)), null, this));

        angularSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Swerve/AngularSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runAngularCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (var module : swerveModules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : swerveModules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
        }

        // Update odometry
        double[] sampleTimestamps =
            swerveModules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                            - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            mPoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
    }

    public void followChoreoTrajectory(SwerveSample sample) {
        runVelocity(HolonomicController.calculate(sample));
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstantsHawkRider.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        for (int i = 0; i < 4; i++) {
            swerveModules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Accepts a vision measurement and updates the pose estimator.
     *
     * @param visionRobotPoseMeters The robot pose measurement from the vision system.
     * @param timestampSeconds The timestamp of the vision measurement.
     * @param visionMeasurementStdDevs The standard deviations of the vision measurement.
     */
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {

        if (!RobotContainer.useVision) return;

        mPoseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /**
     * Runs the drive in a straight line with the specified drive output.
     */
    public void runDriveCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runCharacterization(output);
        }
    }

    /**
     * Runs the robot in a circular motion with the specified turn output.
     */
    public void runTurnCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runTurnCharacterization(output);
        }
    }

    /**
     * Runs the robot in a circular motion with the specified turn output.
     */
    public void runAngularCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runAngularCharacterization(output);
        }
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the measured chassis speeds of the robot.
     */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Swerve/Is Slow Mode")
    public boolean isSlowMode() {
        return SPEED_MULTIPLIER == SLOW_SPEED_MULTIPLIER;
    }

    /**
     * Returns the position of each module in radians.
     */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = swerveModules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += swerveModules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return gyroInputs.connected ?
            gyroInputs.yawPosition : getPose().getRotation();
    }

    /**
     * Calculates the angle the robot needs to turn in order to face
     * a specified target position. The angle is returned as a Rotation2d object
     * and is normalized to the range [-π, π] radians for easier use in rotation-based calculations.
     */
    public Rotation2d calculateTurnAngle(Pose2d targetAngle) {
        double dx = targetAngle.getX() - getPose().getX();
        double dy = targetAngle.getY() - getPose().getY();

        double angleToTarget = Math.atan2(dy, dx);
        double calculatedAngle = angleToTarget - getRotation().getRadians();
        return new Rotation2d(Math.IEEEremainder(calculatedAngle, 2 * Math.PI));
    }

    public Pose2d findClosestReef() {
        Pose2d[] reefPoses = {
            FieldConstants.LEFT_SECTION,
            FieldConstants.TOP_LEFT_SECTION,
            FieldConstants.BOTTOM_LEFT_SECTION,
            FieldConstants.RIGHT_SECTION,
            FieldConstants.TOP_RIGHT_SECTION,
            FieldConstants.BOTTOM_RIGHT_SECTION
        };

        double closestDistance = Double.MAX_VALUE;
        Pose2d closestPose = null;

        for (Pose2d reefPose : reefPoses) {
            Pose2d validated = AllianceFlip.validate(reefPose);
            double distance = getPose().getTranslation().getDistance(validated.getTranslation());

            if (distance < closestDistance) {
                closestDistance = distance;
                closestPose = validated;
            }
        }

        return closestPose;
    }

    public boolean shouldContinuePathfinding(BooleanSupplier stopCondition) {
        Logger.recordOutput("Swerve/InterruptPathfinding", stopCondition.getAsBoolean());
        return !stopCondition.getAsBoolean();
    }

    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        mPoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstantsHawkRider.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /**
     * Returns the speed multiplier.
     */
    public double getSpeedMultiplier() {
        return SPEED_MULTIPLIER;
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /**
     * Returns an array of module translations.
     */
    public static Translation2d[] getModuleTranslations() {
        return switch (Constants.getRobot()) {
            case OMEGABOT ->
                new Translation2d[]{
                    new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                    new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
            };
            case ALPHABOT -> new Translation2d[]{
                new Translation2d(TunerConstantsAlpha.FrontLeft.LocationX, TunerConstantsAlpha.FrontLeft.LocationY),
                new Translation2d(TunerConstantsAlpha.FrontRight.LocationX, TunerConstantsAlpha.FrontRight.LocationY),
                new Translation2d(TunerConstantsAlpha.BackLeft.LocationX, TunerConstantsAlpha.BackLeft.LocationY),
                new Translation2d(TunerConstantsAlpha.BackRight.LocationX, TunerConstantsAlpha.BackRight.LocationY)
            };
            case HAWKRIDER, SIMBOT -> new Translation2d[]{
                new Translation2d(TunerConstantsHawkRider.FrontLeft.LocationX, TunerConstantsHawkRider.FrontLeft.LocationY),
                new Translation2d(TunerConstantsHawkRider.FrontRight.LocationX, TunerConstantsHawkRider.FrontRight.LocationY),
                new Translation2d(TunerConstantsHawkRider.BackLeft.LocationX, TunerConstantsHawkRider.BackLeft.LocationY),
                new Translation2d(TunerConstantsHawkRider.BackRight.LocationX, TunerConstantsHawkRider.BackRight.LocationY)
            };
        };
    }

    public void setPathfinding(boolean pathfinding) {
        isPathfinding = pathfinding;
    }

    public Trigger isPathfinding() {
        return new Trigger(() -> isPathfinding);
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command toggleMultiplier() {
        return Commands.runOnce(() ->
            SPEED_MULTIPLIER = isSlowMode() ? 1.0 : SLOW_SPEED_MULTIPLIER);
    }

    public Command zeroHeading(Pose2d pose) {
        return Commands.runOnce(
            () -> setPose(pose), this)
                .ignoringDisable(true);
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     */
    public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runDriveCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(driveSysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command driveSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runDriveCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(driveSysId.dynamic(direction));
    }

    /**
     * Returns a command to run a quasistatic test for the turn motors.
     */
    public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runVelocity(
            new ChassisSpeeds(
                0.0,
                0.0,
                .01)))
            .withTimeout(1.0)
            .andThen(turnSysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test for the turn motors.
     */
    public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runVelocity(
            new ChassisSpeeds(
                0.0,
                0.0,
                0.01)))
            .withTimeout(1.0)
            .andThen(turnSysId.dynamic(direction));
    }

    /**
     * Returns a command to run a quasistatic test to characterize the robot's angular motion.
     */
    public Command angularSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runAngularCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(angularSysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test to characterize the robot's angular motion.
     */
    public Command angularSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runAngularCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(angularSysId.dynamic(direction));
    }

    /**
     * Returns a command that keeps the module zeroed. Useful for testing if the CANcoder offsets are valid.
     */
    public Command testZeroedModules() {
        return Commands.run(
            () -> runDriveCharacterization(0.0), this);
    }
}
