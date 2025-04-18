package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.steelhawks.Autos.Misalignment;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsHawkRider;
import org.steelhawks.subsystems.LED;
import org.steelhawks.Constants.Mode;
import org.steelhawks.subsystems.LED.LEDColor;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.Elastic;
import org.steelhawks.util.OperatorDashboard;
import org.steelhawks.util.VirtualSubsystem;

import static org.steelhawks.Constants.RobotType.*;

public class Robot extends LoggedRobot {

    private static RobotState mState = RobotState.DISABLED;
    private final RobotContainer robotContainer;
    private static boolean isFirstRun = true;
    private Command autonomousCommand;

    public enum RobotState {
        DISABLED,
        TELEOP,
        AUTON,
        TEST
    }

    private void setState(RobotState state) {
        mState = state;
        Logger.recordOutput("RobotState", state.toString());
    }

    public static RobotState getState() {
        return mState;
    }

    public static boolean isFirstRun() {
        return isFirstRun;
    }

    @SuppressWarnings("resource")
    public Robot() {
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        for (int i = 5800; i < 5810; i++) {
            PortForwarder.add(i, "10.26.1.11", i);
            PortForwarder.add(i, "10.26.1.12", i);
        }

        // record GIT data
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Language,
            FRCNetComm.tInstances.kLanguage_Kotlin,
            0,
            WPILibVersion.Version);
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Framework,
            FRCNetComm.tInstances.kFramework_AdvantageKit,
            0,
            WPILibVersion.Version);
        DriverStation.silenceJoystickConnectionWarning(true);
        Logger.recordMetadata("Robot", Constants.ROBOT_NAME);
        Logger.recordMetadata("Robot Mode", Constants.getMode().toString());
        Logger.recordMetadata("Robot Type", Constants.getRobot().toString());
        Logger.recordMetadata("Robot in Tuning Mode", String.valueOf(Constants.TUNING_MODE));

        // Set up data receivers & replay source
        switch (Constants.getMode()) {
            case REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                new PowerDistribution(
                    Constants.POWER_DISTRIBUTION_CAN_ID, Constants.PD_MODULE_TYPE); // Enables power distribution logging
            }
            case SIM -> // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
            case REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        Logger.start();

        // Check for valid swerve config
        var modules =
            switch (Constants.getRobot()) {
                case OMEGABOT, SIMBOT ->
                    new SwerveModuleConstants[]{
                        TunerConstants.FrontLeft,
                        TunerConstants.FrontRight,
                        TunerConstants.BackLeft,
                        TunerConstants.BackRight
                    };
                case ALPHABOT ->
                    new SwerveModuleConstants[]{
                        TunerConstantsAlpha.FrontLeft,
                        TunerConstantsAlpha.FrontRight,
                        TunerConstantsAlpha.BackLeft,
                        TunerConstantsAlpha.BackRight
                    };
                case HAWKRIDER ->
                    new SwerveModuleConstants[]{
                        TunerConstantsHawkRider.FrontLeft,
                        TunerConstantsHawkRider.FrontRight,
                        TunerConstantsHawkRider.BackLeft,
                        TunerConstantsHawkRider.BackRight
                    };
            };

        for (var constants : modules) {
            if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
                || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
                throw new RuntimeException(
                    "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
            }
        }

        robotContainer = new RobotContainer();
        OperatorDashboard.INSTANCE.initialize();
    }

    @Override
    public void robotPeriodic() {
        VirtualSubsystem.periodicAll();

        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run();
        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);

//        if (Constants.getRobot() != ALPHABOT)
//            Logger.recordOutput("CANbus/CANivore", new CANBus("canivore").getStatus().BusUtilization);
//        Logger.recordOutput("CANbus/Rio", new CANBus().getStatus().BusUtilization);

        if (Constants.getRobot() == SIMBOT)
            updateSimPoseVisualizer();
    }

    private void updateSimPoseVisualizer() {
        Logger.recordOutput("Align/ClosestReef", ReefUtil.getClosestCoralBranch().getScorePose(ElevatorConstants.State.L1));
        Logger.recordOutput("Align/ClosestAlgae", ReefUtil.getClosestAlgae().getRetrievePose());
        Logger.recordOutput("Align/ClosestAlgaeClearance", ReefUtil.getClosestAlgae().getClearancePose());
        Logger.recordOutput("Align/ClosestCoralStation", FieldConstants.getClosestCoralStation().getIntakePoseViaPointToLine());
        Logger.recordOutput("Align/Left", FieldConstants.Cage.LEFT.getClimbPose());
        Logger.recordOutput("Align/Right", FieldConstants.Cage.RIGHT.getClimbPose());
        Logger.recordOutput("Align/Center", FieldConstants.Cage.CENTER.getClimbPose());
        Logger.recordOutput("Align/ClosestBargePoint", FieldConstants.Barge.SCORE.getClearancePose());

        Logger.recordOutput("Clearances/ClearFromElevator", Clearances.AlgaeClawClearances.isClearFromElevatorCrossbeam());
        Logger.recordOutput("Clearances/ClearFromBarge", Clearances.AlgaeClawClearances.isClearFromBarge());
    }

    @Override
    public void disabledInit() {
        setState(RobotState.DISABLED);

        if (Constants.getMode() == Mode.SIM) {
            RobotContainer.s_Swerve.resetSimulation(
                new Pose2d(
                    6,
                    1.5,
                    new Rotation2d()));
        }
    }

    @Override
    public void disabledPeriodic() {
        Misalignment currentState = Autos.getMisalignment();
        if (!isFirstRun)
            LED.getInstance().rainbow();


        if (isFirstRun) {
            Logger.recordOutput("Align/AutonMisalignment", currentState);

            switch (currentState) {
                case NONE -> {
                    if (LED.getInstance().getCurrentCommand() != null)
                        LED.getInstance().getCurrentCommand().cancel();
                    LED.getInstance().setColor(LEDColor.GREEN);
                }
                case ROTATION_CCW ->
                    LED.getInstance().flashUntilCommand(LEDColor.BLUE, 0.3, () -> false).schedule();
                case ROTATION_CW ->
                    LED.getInstance().flashUntilCommand(LEDColor.BLUE, 1.0, () -> false).schedule();
                case X_RIGHT ->
                    LED.getInstance().flashUntilCommand(LEDColor.YELLOW, 0.3, () -> false).schedule();
                case X_LEFT ->
                    LED.getInstance().flashUntilCommand(LEDColor.YELLOW, 1.0, () -> false).schedule();
                case Y_FORWARD ->
                    LED.getInstance().flashUntilCommand(LEDColor.PURPLE, 0.3, () -> false).schedule();
                case Y_BACKWARD ->
                    LED.getInstance().flashUntilCommand(LEDColor.PURPLE, 1.0, () -> false).schedule();
                case MULTIPLE ->
                    LED.getInstance().flashUntilCommand(LEDColor.RED, 0.2, () -> false).schedule();
            }
        }
    }



    @Override
    public void disabledExit() {
        isFirstRun = false;
    }

    @Override
    public void autonomousInit() {
        setState(RobotState.AUTON);
        Elastic.selectTab("Autonomous");
        autonomousCommand = Autos.getAuto();

        if (autonomousCommand != null)
            autonomousCommand.schedule();
        if (LED.getInstance().getCurrentCommand() != null)
            LED.getInstance().getCurrentCommand().cancel();
        LED.getInstance().setDefaultLighting(LED.getInstance().getBlockyRainbowCommand());
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        setState(RobotState.TELEOP);
        Elastic.selectTab("Teleoperated");
        if (autonomousCommand != null)
            autonomousCommand.cancel();
        if (LED.getInstance().getCurrentCommand() != null)
            LED.getInstance().getCurrentCommand().cancel();
        if (DriverStation.isDSAttached())
            robotContainer.waitForDs();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        setState(RobotState.TEST);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        if (Constants.getMode() == Mode.SIM) {
            RobotContainer.s_Swerve.resetSimulation(new Pose2d(3, 3, new Rotation2d()));
        }
    }

    @Override
    public void simulationPeriodic() {
        if (Constants.getMode() == Mode.SIM) {
            RobotContainer.s_Swerve.updatePhysicsSimulation();
        }
    }
}
