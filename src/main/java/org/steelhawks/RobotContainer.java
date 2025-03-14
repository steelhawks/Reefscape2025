package org.steelhawks;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Robot.RobotState;
import org.steelhawks.commands.VibrateController;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsHawkRider;
import org.steelhawks.subsystems.LED.LEDColor;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.align.Align;
import org.steelhawks.subsystems.align.AlignIO;
import org.steelhawks.subsystems.align.AlignIOCANrange;
import org.steelhawks.subsystems.align.AlignIOSim;
import org.steelhawks.subsystems.arm.*;
import org.steelhawks.subsystems.climb.Climb;
import org.steelhawks.subsystems.climb.deep.DeepClimbIO;
import org.steelhawks.subsystems.climb.deep.DeepClimbIOTalonFX;
import org.steelhawks.subsystems.climb.shallow.ShallowClimbIO;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.claw.ClawIO;
import org.steelhawks.subsystems.elevator.*;
import org.steelhawks.subsystems.elevator.ElevatorConstants.State;
import org.steelhawks.subsystems.claw.ClawIOSim;
import org.steelhawks.subsystems.claw.ClawIOTalonFX;
import org.steelhawks.subsystems.arm.ArmIO;
import org.steelhawks.subsystems.arm.ArmIOTalonFX;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.DashboardTrigger;
import org.steelhawks.util.FieldBoundingBox;

public class RobotContainer {

    public static final boolean useVision = false;

    private final Trigger interruptPathfinding;
    private final Trigger isShallowEndgame;
    private final Trigger notifyAtEndgame;
    private final Trigger isDeepEndgame;
    private final Trigger modifierTrigger;
    private boolean shallowClimbMode = false;
    private boolean deepClimbMode = false;

    private final LED s_LED = LED.getInstance();
    public static Swerve s_Swerve;
    public static Vision s_Vision;
    public static Elevator s_Elevator;
    public static Claw s_Claw;
    public static Align s_Align;
    public static Climb s_Climb;
    public static Arm s_Arm;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    public void waitForDs() {
        boolean isRed = AllianceFlip.shouldFlip();
        Color c1 = isRed ? Color.kBlue : Color.kRed;
        Color c2 = isRed ? Color.kRed : Color.kBlue;

        s_LED.setDefaultLighting(
            s_LED.movingDiscontinuousGradient(
                c1, c2));
    }

    public RobotContainer() {
        interruptPathfinding =
            new Trigger(() ->
                Math.abs(driver.getLeftY()) > Deadbands.DRIVE_DEADBAND ||
                    Math.abs(driver.getLeftX()) > Deadbands.DRIVE_DEADBAND ||
                    Math.abs(driver.getRightX()) > Deadbands.DRIVE_DEADBAND);
        isShallowEndgame = new Trigger(() -> shallowClimbMode);
        isDeepEndgame = new Trigger(() -> deepClimbMode);
        notifyAtEndgame = new Trigger(() -> {
//            When connected to the real field, this number only changes in full integer increments, and always counts down.
//                When the DS is in practice mode, this number is a floating point number, and counts down.
//            When the DS is in teleop or autonomous mode, this number is a floating point number, and counts up.
//            Simulation matches DS behavior without an FMS connected.

            double matchTime = DriverStation.getMatchTime();

            // If connected to FMS, matchTime counts down in whole numbers
            // If in Practice Mode, matchTime is a floating-point number and counts down
            if (DriverStation.isFMSAttached()) {
                return Robot.getState() == RobotState.TELEOP && matchTime <= Constants.ENDGAME_PERIOD;
            }

            // If in Teleop/Autonomous mode (not connected to FMS), matchTime counts up
            if (Robot.getState() == RobotState.TELEOP) {
                return matchTime >= (Constants.MATCH_TIME_SECONDS - Constants.ENDGAME_PERIOD);
            }

            return false;
        });
        modifierTrigger = operator.rightTrigger();

        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case OMEGABOT -> {
                    s_Swerve =
                        new Swerve(
                            new GyroIOPigeon2(
                                TunerConstants.DrivetrainConstants.Pigeon2Id,
                                TunerConstants.DrivetrainConstants.CANBusName),
                            new ModuleIOTalonFX(TunerConstants.FrontLeft),
                            new ModuleIOTalonFX(TunerConstants.FrontRight),
                            new ModuleIOTalonFX(TunerConstants.BackLeft),
                            new ModuleIOTalonFX(TunerConstants.BackRight));
                    s_Vision =
                        new Vision(
                            s_Swerve::accept,
                            new VisionIOPhoton(
                                VisionConstants.cameraNames()[0],
                                VisionConstants.robotToCamera()[0]),
                            new VisionIOPhoton(
                                VisionConstants.cameraNames()[1],
                                VisionConstants.robotToCamera()[1]),
                            new VisionIOPhoton(
                                VisionConstants.cameraNames()[2],
                                VisionConstants.robotToCamera()[2]),
                            new VisionIOPhoton(
                                VisionConstants.cameraNames()[3],
                                VisionConstants.robotToCamera()[3]));
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOTalonFX());
                    s_Claw =
                        new Claw(
                            new ClawIOTalonFX());
                    s_Align =
                        new Align(
                            new AlignIOCANrange());
                    s_Climb =
                        new Climb(
                            new ShallowClimbIO() {},
                            new DeepClimbIO() {});
                    s_Arm =
                        new Arm(
                            new ArmIOTalonFX());
                }
                case ALPHABOT -> {
                    s_Swerve =
                        new Swerve(
                            new GyroIOPigeon2(
                                TunerConstantsAlpha.DrivetrainConstants.Pigeon2Id,
                                TunerConstantsAlpha.DrivetrainConstants.CANBusName),
                            new ModuleIOTalonFX(TunerConstantsAlpha.FrontLeft),
                            new ModuleIOTalonFX(TunerConstantsAlpha.FrontRight),
                            new ModuleIOTalonFX(TunerConstantsAlpha.BackLeft),
                            new ModuleIOTalonFX(TunerConstantsAlpha.BackRight));
                    s_Vision =
                        new Vision(
                            s_Swerve::accept,
                            new VisionIOLimelight(VisionConstants.cameraNames()[0], () -> s_Swerve.getRotation()));
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOTalonFX());
                    s_Claw =
                        new Claw(
                            new ClawIOTalonFX());
                    s_Align =
                        new Align(
                            new AlignIO() {});
                    s_Climb =
                        new Climb(
                            new ShallowClimbIO() {},
                            new DeepClimbIOTalonFX());
                    s_Arm =
                        new Arm(
                            new ArmIOTalonFX());
                }
                case HAWKRIDER -> {
                    s_Swerve =
                        new Swerve(
                            new GyroIOPigeon2(
                                TunerConstantsHawkRider.DrivetrainConstants.Pigeon2Id,
                                TunerConstantsHawkRider.DrivetrainConstants.CANBusName),
                            new ModuleIOTalonFX(TunerConstantsHawkRider.FrontLeft),
                            new ModuleIOTalonFX(TunerConstantsHawkRider.FrontRight),
                            new ModuleIOTalonFX(TunerConstantsHawkRider.BackLeft),
                            new ModuleIOTalonFX(TunerConstantsHawkRider.BackRight));
                    s_Vision =
                        new Vision(
                            s_Swerve::accept,
                            new VisionIOLimelight(VisionConstants.cameraNames()[0], () -> s_Swerve.getRotation()),
                            new VisionIOLimelight(VisionConstants.cameraNames()[1], () -> s_Swerve.getRotation()));
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOTalonFX());
                    s_Claw =
                        new Claw(
                            new ClawIO() {});
                    s_Align =
                        new Align(
                            new AlignIO() {});
                    s_Climb =
                        new Climb(
                            new ShallowClimbIO() {},
                            new DeepClimbIO() {});
                    s_Arm =
                        new Arm(
                            new ArmIO() {});
                }
                case SIMBOT -> {
                    Logger.recordOutput("Pose/CoralStationTop", FieldConstants.Position.CORAL_STATION_TOP.getPose());
                    Logger.recordOutput("Pose/CoralStationBottom", FieldConstants.Position.CORAL_STATION_BOTTOM.getPose());
                    Logger.recordOutput("Swerve/ModuleTranslations", Swerve.getModuleTranslations());

                    for (int i = 0; i < VisionConstants.cameraNames().length; i++) {
                        Logger.recordOutput("Camera/" + VisionConstants.cameraNames()[i], VisionConstants.robotToCamera()[i]);
                    }

                    for (ReefUtil.CoralBranch branch : ReefUtil.CoralBranch.values()) {
                        Logger.recordOutput("Pose/" + branch.name(), branch.getScorePose(State.L4));
                    }

                    s_Swerve =
                        new Swerve(
                            new GyroIOSim(Swerve.getDriveSimulation().getGyroSimulation()),
                            new ModuleIOSim(Swerve.getDriveSimulation().getModules()[0]),
                            new ModuleIOSim(Swerve.getDriveSimulation().getModules()[1]),
                            new ModuleIOSim(Swerve.getDriveSimulation().getModules()[2]),
                            new ModuleIOSim(Swerve.getDriveSimulation().getModules()[3]));
                    s_Vision =
                        new Vision(
                            s_Swerve::accept,
                            new VisionIOPhotonSim(
                                VisionConstants.cameraNames()[0],
                                VisionConstants.robotToCamera()[0],
                                Swerve.getDriveSimulation()::getSimulatedDriveTrainPose),
                            new VisionIOPhotonSim(
                                VisionConstants.cameraNames()[1],
                                VisionConstants.robotToCamera()[1],
                                Swerve.getDriveSimulation()::getSimulatedDriveTrainPose),
                            new VisionIOPhotonSim(
                                VisionConstants.cameraNames()[2],
                                VisionConstants.robotToCamera()[2],
                                Swerve.getDriveSimulation()::getSimulatedDriveTrainPose),
                            new VisionIOPhotonSim(
                                VisionConstants.cameraNames()[3],
                                VisionConstants.robotToCamera()[3],
                                Swerve.getDriveSimulation()::getSimulatedDriveTrainPose));
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOSim());
                    s_Claw =
                        new Claw(
                            new ClawIOSim());
                    s_Align =
                        new Align(
                            new AlignIOSim());
                    s_Climb =
                        new Climb(
                            new ShallowClimbIO() {},
                            new DeepClimbIO() {});
                    s_Arm =
                        new Arm(
                            new ArmIOSim());
                }
            }
        }

        if (Constants.getMode() == Mode.REPLAY) {
            s_Swerve =
                new Swerve(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});

            switch (Constants.getRobot()) {
                case OMEGABOT, ALPHABOT -> {
                    s_Vision =
                        new Vision(
                            s_Swerve::accept,
                            new VisionIO() {});
                    s_Claw =
                        new Claw(
                            new ClawIO() {});
                    s_Align =
                        new Align(
                            new AlignIO() {});
                    s_Climb =
                        new Climb(
                            new ShallowClimbIO() {},
                            new DeepClimbIO() {});
                    s_Arm =
                        new Arm(
                            new ArmIO() {});
                }

                case HAWKRIDER -> // hawkrider has 2 limelights and an orange pi running pv
                    s_Vision =
                        new Vision(
                            s_Swerve::accept,
                            new VisionIO() {},
                            new VisionIO() {});
            }

            if (Constants.getRobot() == RobotType.OMEGABOT) {
                s_Vision =
                    new Vision(
                        s_Swerve::accept,
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {});
            }

            s_Elevator =
                new Elevator(
                    new ElevatorIO() {});
        }

        new Alert("Tuning mode enabled", AlertType.kInfo).set(Constants.TUNING_MODE);
        Autos.init();

        configureShallowClimbEndgame();
        configurePathfindingCommands();
        configureDeepClimbEndgame();
        configureDefaultCommands();
        checkIfDevicesConnected();
        configureTestBindings();
        configureTriggers();
        configureOperator();
        configureDriver();
    }

    private void checkIfDevicesConnected() {
        boolean orangePi1Connected = false;
        boolean orangePi2Connected = false;
        for (ConnectionInfo info : NetworkTableInstance.getDefault().getConnections()) {
            if (info.remote_ip.equals("10.26.1.11")) {
                orangePi1Connected = true;
            }

            if (info.remote_ip.equals("10.26.1.12")) {
                orangePi2Connected = true;
            }
        }

        new Alert("Orange Pi 1 is not connected", AlertType.kError).set(orangePi1Connected);
        new Alert("Orange Pi 2 is not connected", AlertType.kError).set(orangePi2Connected);
    }

    private void configurePathfindingCommands() {
        /* ------------- Pathfinding Poses ------------- */
        driver.leftBumper()
            .whileTrue(
                s_Align.alignToClosestReef(State.L4));
    }

    private void configureDefaultCommands() {
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));
    }

    private void configureTestBindings() {}

    private void configureShallowClimbEndgame() {}

    private void configureDeepClimbEndgame() {
        operator.rightStick()
            .and(isDeepEndgame)
            .onTrue(
                s_Climb.toggleManualControl(() -> -operator.getRightY()));
    }

    private void configureTriggers() {
        s_Swerve.isPathfinding()
            .whileTrue(
                s_LED.rainbowFlashCommand());

        s_Elevator.atLimit()
            .onTrue(
                s_LED.flashCommand(LEDColor.PURPLE, 0.1, 1))
            .whileFalse(
                s_LED.setColorCommand(LEDColor.WHITE).repeatedly());

        s_Claw.hasCoral()
            .onTrue(
                Commands.parallel(
                    s_LED.flashCommand(LEDColor.GREEN, 0.1, 3),
                    new VibrateController(1.0, 2, driver, operator)));

        isShallowEndgame
            .onTrue(
                Commands.runOnce(() ->
                    s_LED.setDefaultLighting(
                        s_LED.rainbowFlashCommand())));

        isDeepEndgame
            .onTrue(
                Commands.runOnce(() ->
                    s_LED.setDefaultLighting(
                        s_LED.fadeCommand(LEDColor.BLUE)))
                .andThen(
                    s_Climb.prepareDeepClimb()))
            .onFalse(
                s_Climb.goHome()
                    .andThen(
                        Commands.runOnce(this::waitForDs)));

        notifyAtEndgame
            .whileTrue(
                new VibrateController(1.0, 5.0, driver, operator));

        new FieldBoundingBox(
            "Top Coral Station",
            0.0, 2.0, 6.2, 8.0,
            s_Swerve::getPose)
            .whileTrue(
                s_Claw.intakeCoral());
    }

    private void configureDriver() {
        /* ------------- Swerve Controls ------------- */
        driver.rightTrigger().onTrue(s_Swerve.toggleMultiplier()
            .alongWith(
                Commands.either(
                    s_LED.flashCommand(LEDColor.GREEN, 0.2, 2),
                    s_LED.flashCommand(LEDColor.RED, 0.2, 2),
                    () -> s_Swerve.isSlowMode())));

        driver.b().onTrue(
            s_Swerve.zeroHeading());
    }

    private void configureOperator() {
        /* ------------- End Game Toggles ------------- */
        operator.start()
        .and(operator.back())
            .onTrue(
                Commands.runOnce(() -> deepClimbMode = !deepClimbMode));

        /* ------------- Elevator Controls ------------- */

        operator.leftStick().onTrue(
            s_Elevator.toggleManualControl(
                () -> -operator.getLeftY()));

        operator.leftBumper()
            .or(new DashboardTrigger("l1"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.L1));

        operator.x()
            .and(modifierTrigger.negate())
            .or(new DashboardTrigger("l2"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.L2));

        operator.x()
            .and(modifierTrigger)
            .onTrue(
                s_Elevator.setDesiredState(State.KNOCK_L2));

        operator.y()
            .and(modifierTrigger.negate())
            .or(new DashboardTrigger("l3"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.L3));

        operator.y()
            .and(modifierTrigger)
            .onTrue(
                s_Elevator.setDesiredState(State.KNOCK_L3));

        operator.a()
            .and(modifierTrigger.negate())
            .or(new DashboardTrigger("l4"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.L4));

        operator.b()
            .or(new DashboardTrigger("elevatorHome"))
            .onTrue(
                s_Elevator.noSlamCommand());

        operator.rightBumper()
            .whileTrue(
                Commands.parallel(
                    s_Arm.applySpinSpeed(-0.2),
                    s_Arm.applyPivotSpeed(0.15)));

        /* ------------- Intake Controls ------------- */
        operator.leftTrigger()
            .or(new DashboardTrigger("scoreCoral"))
            .whileTrue(
                Commands.either(
                    s_Claw.shootPulsatingCoral(),
                    s_Claw.shootCoral(),
                    () -> (s_Elevator.getDesiredState() == ElevatorConstants.State.L4.getRadians() ||
                        s_Elevator.getDesiredState() == ElevatorConstants.State.L1.getRadians()) && s_Elevator.isEnabled())
                .alongWith(LED.getInstance().flashCommand(LEDColor.WHITE, 0.2, 2)));

        operator.povLeft()
            .or(new DashboardTrigger("intakeCoral")) // rename to reverseCoral on app
            .whileTrue(
                s_Claw.reverseCoral()
                    .alongWith(LED.getInstance().flashCommand(LEDColor.PINK, 0.2, 2)));

        operator.povRight()
            .whileTrue(
                s_Claw.intakeCoral()
                    .alongWith(LED.getInstance().flashCommand(LEDColor.GREEN, 0.2, 2)));
    }
}
