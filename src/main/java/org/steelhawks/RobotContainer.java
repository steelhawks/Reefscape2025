package org.steelhawks;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Robot.RobotState;
import org.steelhawks.commands.*;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsHawkRider;
import org.steelhawks.subsystems.LED.LEDColor;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.algaeclaw.*;
import org.steelhawks.subsystems.align.Align;
import org.steelhawks.subsystems.align.AlignIO;
import org.steelhawks.subsystems.align.AlignIOSim;
import org.steelhawks.subsystems.claw.*;
import org.steelhawks.subsystems.claw.beambreak.BeamIO;
import org.steelhawks.subsystems.claw.beambreak.BeamIOCANrange;
import org.steelhawks.subsystems.claw.beambreak.BeamIOSim;
import org.steelhawks.subsystems.elevator.*;
import org.steelhawks.subsystems.elevator.ElevatorConstants.State;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.util.*;

import java.util.Objects;


public class RobotContainer {

    public static final boolean useVision = true;
    private static final boolean usingButtonBoard = false;

    private final Trigger notifyAtEndgame;
    private final Trigger topCoralStationTrigger;
    private final Trigger bottomCoralStationTrigger;
    private final Trigger endGameMode;
    private boolean deepClimbMode = false;

    public static final ReefState s_ReefState = new ReefState();
    private final LED s_LED = LED.getInstance();
    public static Swerve s_Swerve;
    public static Vision s_Vision;
    public static Elevator s_Elevator;
    public static Claw s_Claw;
    public static Align s_Align;
    public static AlgaeClaw s_AlgaeClaw;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);
    private final ButtonBoard buttonBoard
        = new ButtonBoard(OIConstants.BUTTON_BOARD_PORT);

    public RobotContainer() {
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Field", FieldConstants.FIELD_2D);
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
        endGameMode = new Trigger(() -> deepClimbMode);

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
                            new BeamIOCANrange(),
                            new ClawIOSparkFlex());
                    s_Align =
                        new Align(
                            new AlignIO() {});
                    s_AlgaeClaw =
                        new AlgaeClaw(
                            new AlgaeClawIO() {});
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
                            new BeamIO() {},
                            new ClawIOTalonFX());
                    s_Align =
                        new Align(
                            new AlignIO() {});
                    s_AlgaeClaw =
                        new AlgaeClaw(
                            new AlgaeClawIO() {});
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
                            new BeamIO() {},
                            new ClawIO() {});
                    s_Align =
                        new Align(
                            new AlignIO() {});
                    s_AlgaeClaw =
                        new AlgaeClaw(
                            new AlgaeClawIO() {});
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
                            new GyroIOSim(Objects.requireNonNull(Swerve.getDriveSimulation()).getGyroSimulation()),
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
                                Swerve.getDriveSimulation()::getSimulatedDriveTrainPose));
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOSim());
                    s_Claw =
                        new Claw(
                            new BeamIOSim(),
                            new ClawIOSim());
                    s_Align =
                        new Align(
                            new AlignIOSim());
                    s_AlgaeClaw =
                        new AlgaeClaw(
                            new AlgaeClawIOSim());
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
                            new BeamIO() {},
                            new ClawIO() {});
                    s_Align =
                        new Align(
                            new AlignIO() {});
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
                        new VisionIO() {},
                        new VisionIO() {});
            }
            s_Elevator =
                new Elevator(
                    new ElevatorIO() {});
            s_AlgaeClaw =
                new AlgaeClaw(
                    new AlgaeClawIO() {});
        }

        new Alert("Tuning mode enabled", AlertType.kInfo).set(Constants.TUNING_MODE);
        new Alert("Use Vision is Off", AlertType.kWarning).set(!useVision);
        Autos.init();

        topCoralStationTrigger =
            new FieldBoundingBox(
                "Top Coral Station",
                0.0, 2.0, 6.2, 8.0,
                s_Swerve::getPose);
        bottomCoralStationTrigger =
            new FieldBoundingBox(
                "Bottom Coral Station",
                0.0, 2.0, 0.0, 8.0 - 6.2,
                s_Swerve::getPose);

        checkIfDevicesConnected();
        configureTriggers();
        configureOperator();
        configureDriver();

        s_LED.setDefaultCommand(new LEDDefaultCommand());
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

        new Alert("Orange Pi 1 is not connected", AlertType.kError).set(!orangePi1Connected);
        new Alert("Orange Pi 2 is not connected", AlertType.kError).set(!orangePi2Connected);
    }

    private void configureTriggers() {
        s_Elevator.atLimit()
            .onTrue(
                s_LED.flashCommand(LEDColor.PURPLE, 0.1, 1).ignoringDisable(false))
            .whileFalse(
                s_LED.setColorCommand(LEDColor.WHITE).repeatedly());

        s_Claw.hasCoral()
            .onTrue(
                Commands.parallel(
                    s_LED.flashCommand(LEDColor.GREEN, 0.1, 1.0),
                    new VibrateController(1.0, 1.0, driver, operator))
                .ignoringDisable(false));

        s_AlgaeClaw.hasAlgae()
            .onTrue(
                Commands.parallel(
                    s_LED.flashCommand(LEDColor.GREEN, 0.1, 1.0),
                    new VibrateController(1.0, 1.0, driver, operator))
                .ignoringDisable(false));

        notifyAtEndgame
            .whileTrue(
                new VibrateController(1.0, 5.0, driver, operator));

        topCoralStationTrigger
        .or(bottomCoralStationTrigger)
        .and(() -> Robot.getState() != RobotState.AUTON) // due to the nature of triggers, if this runs during auton, the autonomous command will be interrupted
            .whileTrue(
                s_Claw.intakeCoral()
            .until(s_Claw.hasCoral()));
    }

    private void configureDriver() {
        /* ------------- Swerve Controls ------------- */

//        s_Swerve.setDefaultCommand(
//            DriveCommands.joystickDrive(
//                () -> -driver.getLeftY(),
//                () -> -driver.getLeftX(),
//                () -> -driver.getRightX()));

        driver.x().onTrue(s_Swerve.toggleMultiplier()
            .alongWith(
                Commands.either(
                    s_LED.flashCommand(LEDColor.GREEN, 0.2, 2),
                    s_LED.flashCommand(LEDColor.RED, 0.2, 2),
                    () -> s_Swerve.isSlowMode()).withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        driver.b().onTrue(
            s_Swerve.zeroHeading());

        driver.leftBumper()
            .whileTrue(
                s_Align.alignToClosestReefWithFusedInput(State.L4, driver::getLeftX));

        driver.leftTrigger()
            .whileTrue(
                s_Align.alignToClosestCoralStation(() -> -driver.getLeftY(), () -> -driver.getLeftX()));
    }

    private void configureOperator() {
        /* ------------- End Game Toggles ------------- */

        operator.start()
        .and(operator.back())
            .onTrue(
                Commands.runOnce(() -> deepClimbMode = !deepClimbMode));

        /* ------------- Manual Controls ------------- */

        operator.leftStick()
            .and(endGameMode.negate())
            .onTrue(s_Elevator.toggleManualControl(() -> -operator.getLeftY()));

        /* ------------- Elevator Controls ------------- */

        operator.leftBumper()
            .or(new DashboardTrigger("l1"))
            .or(buttonBoard.getL1().and(() -> usingButtonBoard))
            .onTrue(s_Elevator.setDesiredState((State.L1)));

        operator.x()
            .or(new DashboardTrigger("l2"))
            .or(buttonBoard.getL2().and(() -> usingButtonBoard))
            .onTrue(s_Elevator.setDesiredState((State.L2)));

        operator.y()
            .or(new DashboardTrigger("l3"))
            .or(buttonBoard.getL3().and(() -> usingButtonBoard))
            .onTrue(s_Elevator.setDesiredState(State.L3));


        operator.a()
            .or(new DashboardTrigger("l4"))
            .or(buttonBoard.getL4().and(() -> usingButtonBoard))
            .onTrue(
                s_Elevator.setDesiredState(State.L4));

        operator.b()
            .or(new DashboardTrigger("elevatorHome"))
            .or(buttonBoard.getHome().and(() -> usingButtonBoard))
            .onTrue(s_Elevator.noSlamCommand());

        /* ------------- Intake Controls ------------- */

        operator.leftTrigger()
            .or(new DashboardTrigger("scoreCoral"))
            .or(buttonBoard.getShoot().and(() -> usingButtonBoard))
            .whileTrue(
                Commands.either(
                    s_Claw.shootCoralSlow(),
                    s_Claw.shootCoral(),
                    () ->
                        (s_Elevator.getDesiredState() == ElevatorConstants.State.L1.getAngle().getRadians() ||
                            s_Elevator.getDesiredState() == ElevatorConstants.State.L4.getAngle().getRadians()) && s_Elevator.isEnabled())
                .alongWith(LED.getInstance().flashCommand(LEDColor.WHITE, 0.2, 2.0).repeatedly()));

        operator.povLeft()
            .or(new DashboardTrigger("intakeCoral")) // rename to reverseCoral on app
            .whileTrue(
                s_Claw.reverseCoral()
                    .alongWith(LED.getInstance().flashCommand(LEDColor.PINK, 0.2, 2.0).repeatedly()));
    }
}
