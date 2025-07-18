package org.steelhawks;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
import org.steelhawks.subsystems.elevator.ElevatorIOSim;
import org.steelhawks.subsystems.elevator.*;
import org.steelhawks.subsystems.elevator.ElevatorConstants.State;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.util.DoublePressTrigger;

import java.util.Objects;
import java.util.Set;


public class RobotContainer {

    private final LED s_LED = LED.getInstance();
    public static Swerve s_Swerve = null;
    public static Vision s_Vision = null;
    public static Elevator s_Elevator = null;
    public static Claw s_Claw = null;
    public static Align s_Align = null;
    public static AlgaeClaw s_AlgaeClaw = null;

    private final Alert autoMarkingDisabled = new Alert("Auto-Marking is currently disabled", AlertType.kWarning);
    private final Alert manualModeToggled = new Alert("Manual Mode is currently toggled", AlertType.kWarning);
    private boolean manualToggled = false;
    private boolean toggleTriggered = false;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Field", FieldConstants.FIELD_2D);
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
        new Alert("Use Vision is Off", AlertType.kWarning).set(!Toggles.Vision.visionEnabled.get());
        Autos.init();

        checkIfDevicesConnected();
        setManualToggled(false);
        configureTriggers();
        configureDriver();

        s_LED.setDefaultCommand(new LEDDefaultCommand(() -> manualToggled));
        s_AlgaeClaw.setDefaultCommand(new AlgaeClawDefaultCommand());
        s_Claw.setDefaultCommand(s_Claw.indexCoral());
    }

    private void setManualToggled(boolean value) {
        manualToggled = value;
        Logger.recordOutput("Toggles/ManualMode", manualToggled);
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
                s_LED.flashCommand(LEDColor.PURPLE, 0.1, 1).ignoringDisable(false));

        new Trigger(() -> s_Claw.hasCoral())
            .onTrue(
                Commands.parallel(
                    s_LED.flashCommand(LEDColor.GREEN, 0.1, 0.5),
                    new VibrateController(1.0, 1.0, driver))
                .ignoringDisable(false));

        if (s_AlgaeClaw != null) {
            new Trigger(() -> s_AlgaeClaw.hasAlgae())
                .onTrue(
                    Commands.parallel(
                        s_LED.flashCommand(LEDColor.GREEN, 0.1, 1.0),
                        new VibrateController(1.0, 1.0, driver))
                    .ignoringDisable(false));
        }
    }

    private void configureDriver() {
        /* ------------- Swerve Controls ------------- */

        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

        new DoublePressTrigger(driver.start())
            .onDoubleTap(
                Commands.runOnce(() -> {
                    // if (!s_Elevator.isLocked() || (!s_AlgaeClaw.isLocked() && s_AlgaeClaw != null)) {
                    if (!s_Elevator.isLocked()) {
                        Commands.parallel(
                            new VibrateController(1.0, 1.0, driver),
                            s_LED.flashCommand(LEDColor.RED, 0.1, 1.0)).schedule();
                        return;
                    }

                    setManualToggled(!manualToggled);
                    manualModeToggled.set(manualToggled);
                    s_Swerve.getDefaultCommand().cancel();
                    s_Swerve.removeDefaultCommand();
                    if (s_AlgaeClaw != null) {
                        s_AlgaeClaw.getDefaultCommand().cancel();
                        s_AlgaeClaw.removeDefaultCommand();
                    }
                    if (manualToggled) {
                        s_Swerve.setDefaultCommand(Commands.idle(s_Swerve));
                        if (s_AlgaeClaw != null) {
                            s_AlgaeClaw.setDefaultCommand(s_AlgaeClaw.pivotManual(() -> -driver.getRightY()));
                        }
                    } else {
                        s_Swerve.setDefaultCommand(
                            DriveCommands.joystickDrive(
                                () -> -driver.getLeftY(),
                                () -> -driver.getLeftX(),
                                () -> -driver.getRightX()));
                        if (s_AlgaeClaw != null) {
                            s_AlgaeClaw.setDefaultCommand(new AlgaeClawDefaultCommand());
                        }
                    }
                }));

        driver.leftStick()
            .and(() -> manualToggled)
            .onTrue(
                s_AlgaeClaw.avoid()
                    .andThen(
                        Commands.waitUntil(Clearances.AlgaeClawClearances::isClearFromElevatorCrossbeam),
                        s_Elevator.toggleManualControl(() -> -driver.getLeftY())));

        driver.leftBumper()
            .whileTrue(
                Commands.defer(
                    () -> Commands.sequence(
                        Commands.either(
                            Align.directPathFollow(ReefUtil.getCoralBranchWithFusedDriverInput(driver::getLeftX).get().getScorePose(State.L4), true),
                            Align.alignWithSetpoint(ReefState.dynamicScoreRoutine().branch(), ReefState.dynamicScoreRoutine().state(), true),
                            ReefState::hasOverriden)
                        .unless(() -> Robot.getState() == RobotState.TEST), // so it doesnt drive when doing systems check
                        Commands.runOnce(() -> LEDDefaultCommand.isAligned = true), // set led state true, align command ended
                        s_Elevator.setDesiredState(ReefState.dynamicScoreRoutine().state()),
                        Commands.waitUntil(s_Elevator.atThisGoal(ReefState.dynamicScoreRoutine().state())),
                        s_Claw.shootCoralEnd(),
                        Commands.waitUntil(Clearances.ClawClearances::isClearFromReef),
                        Commands.runOnce(() -> LEDDefaultCommand.isAligned = false),
                        s_Elevator.homeCommand()
                            .onlyWhile(() -> Math.abs((ReefState.hasOverriden() ? 0 : 1 * driver.getLeftX()) + driver.getLeftY()) < 0.6)),
                    Set.of()))
            .onFalse(
                Commands.waitUntil(Clearances.ClawClearances::isClearFromReef)
                    .andThen(s_Elevator.setDesiredState(State.HOME)));

        driver.back().onTrue(s_Swerve.toggleMultiplier()
            .alongWith(
                new VibrateController(driver),
                Commands.either(
                    s_LED.flashCommand(LEDColor.GREEN, 0.2, 2),
                    s_LED.flashCommand(LEDColor.RED, 0.2, 2),
                    () -> s_Swerve.isSlowMode()).withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        driver.rightStick()
            .and(() -> !manualToggled)
            .onTrue(
                Commands.defer(() ->
                    DriveCommands.joystickDriveAtAngle(
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> FieldConstants.getClosestCoralStation().getIntakePose().getRotation()),
                    Set.of(s_Swerve))
                .until(() -> Math.abs(driver.getRightX()) > 0.3)
                .withName("Angle to Coral Station"));

        driver.rightStick().debounce(0.25)
            .and(() -> !manualToggled)
            .whileTrue(
                s_Align.alignToClosestCoralStation(() -> -driver.getLeftY(), () -> -driver.getLeftX()));

        /* ------------- Elevator Controls ------------- */

        SuperStructure.smartScoreTrigger(driver.rightBumper(), State.L1, driver::getLeftX, driver::getLeftY);
        SuperStructure.smartScoreTrigger(driver.x(), State.L2, driver::getLeftX, driver::getLeftY);
        SuperStructure.smartScoreTrigger(driver.y(), State.L3, driver::getLeftX, driver::getLeftY);
        SuperStructure.smartScoreTrigger(driver.a(), State.L4, driver::getLeftX, driver::getLeftY);

        driver.b()
            .onTrue(s_Elevator.homeCommand());

        /* ------------- Intake Controls ------------- */

        driver.leftTrigger()
            .whileTrue(
                s_Claw.shootCoral()
                .alongWith(LED.getInstance().flashCommand(LEDColor.WHITE, 0.2, 2.0).repeatedly()));

        driver.povLeft()
            .whileTrue(
                s_Claw.reverseCoral()
                .alongWith(LED.getInstance().flashCommand(LEDColor.PINK, 0.2, 2.0).repeatedly()));

        driver.povDown()
            .whileTrue(
                s_AlgaeClaw.outtakeAlgae());

//        driver.rightTrigger()
//            .whileTrue(
//                Commands.either(
//                    Commands.sequence(
//                        Commands.defer(
//                            () -> Align.directPathFollow(FieldConstants.Barge.SCORE.getClearancePose(), true),
//                            Set.of(s_Swerve)),
//                        s_AlgaeClaw.catapult(),
//                        Commands.waitUntil(Clearances.AlgaeClawClearances::isClearFromElevatorCrossbeam),
//                        s_Elevator.setDesiredState(State.BARGE_SCORE),
//                        Commands.waitUntil(s_Elevator.atThisGoal(State.BARGE_SCORE)),
//                        new SwerveDriveAlignment(FieldConstants.Barge.SCORE.getCatapultPose(), true),
//                        s_AlgaeClaw.outtakeAlgae().withTimeout(0.5)),
//
//                    Commands.sequence(
//                        Commands.defer(
//                            () -> Align.directPathFollow(ReefUtil.getClosestAlgae().getClearancePose(), true),
//                            Set.of(s_Swerve)),
//                        s_AlgaeClaw.intake(),
//                        Commands.waitUntil(Clearances.AlgaeClawClearances::isClearFromElevatorCrossbeam),
//                        Commands.either(
//                            s_Elevator.setDesiredState(State.KNOCK_L3),
//                            s_Elevator.setDesiredState(State.KNOCK_L2),
//                            () -> ReefUtil.getClosestAlgae().isOnL3()),
//                        Commands.defer(
//                            () -> Commands.waitUntil(
//                                s_Elevator.atThisGoal(
//                                    ReefUtil.getClosestAlgae().isOnL3()
//                                        ? State.KNOCK_L3
//                                        : State.KNOCK_L2)),
//                            Set.of()),
//                        Commands.defer(
//                            () -> new SwerveDriveAlignment(() -> ReefUtil.getClosestAlgae().getRetrievePose()),
//                            Set.of(s_Swerve)))
//                        .alongWith(s_AlgaeClaw.intakeAlgae())
//                        .until(s_AlgaeClaw.hasAlgae()),
//
//                    s_AlgaeClaw.hasAlgae())
//                    .until(() -> Math.abs(driver.getLeftX() + driver.getLeftY()) <= 0.3))
//            .onFalse(
//                Commands.sequence(
//                    Commands.waitUntil(Clearances.AlgaeClawClearances::isClearFromReef),
//                    s_Elevator.homeCommand()));

        driver.povUp().debounce(0.5)
            .onTrue(
                Commands.runOnce(() -> {
                    toggleTriggered = true;
                    ReefState.ScoreGoal lastPosition = ReefState.getLastScoredPosition();
                    if (lastPosition != null) {
                        ReefState.removeScoredCoral(lastPosition);
                        Commands.parallel(
                            new VibrateController(1.0, 0.25, driver),
                            s_LED.flashCommand(LEDColor.GREEN, 0.2, 2.0)).schedule();
                    }
                }))
            .onFalse(
                Commands.runOnce(() -> {
                    if (!toggleTriggered) {
                        Toggles.autoMark.set(!Toggles.autoMark.get());
                        autoMarkingDisabled.set(!Toggles.autoMark.get());
                        if (Toggles.autoMark.get()) {
                            s_LED.flashCommand(LEDColor.GREEN, 0.2, 2.0).schedule();
                        } else {
                            s_LED.flashCommand(LEDColor.RED, 0.2, 2.0).schedule();
                        }
                    }
                    toggleTriggered = false;
                }));

        driver.povRight() // use if aligns to an already taken branch part, marks it as taken on reefstate
            .onTrue(
                Commands.runOnce(() -> {
                    if (s_Claw.hasCoral()
                        && s_Elevator.isScoringLevel()
                        && ReefUtil.getClosestCoralBranch()
                            .getScorePose(s_Elevator.getState())
                            .getTranslation()
                        .getDistance(s_Swerve.getPose().getTranslation()) <= 1.5
                    ) {
                        ReefState.scoreCoral(ReefUtil.getClosestCoralBranch(), s_Elevator.getState());
                        s_LED.flashCommand(LEDColor.BLUE, 0.2, 2.0).schedule();
                    }
                }));
    }
}
