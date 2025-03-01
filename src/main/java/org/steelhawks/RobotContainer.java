package org.steelhawks;

import edu.wpi.first.math.geometry.*;
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
import org.steelhawks.subsystems.climb.Climb;
import org.steelhawks.subsystems.climb.deep.DeepClimbIO;
import org.steelhawks.subsystems.climb.deep.DeepClimbIO775Pro;
import org.steelhawks.subsystems.climb.deep.DeepClimbIOTalonFX;
import org.steelhawks.subsystems.climb.shallow.ShallowClimbIO;
import org.steelhawks.subsystems.climb.shallow.ShallowClimbIOTalonFX;
import org.steelhawks.subsystems.elevator.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIO;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIOSim;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIOTalonFX;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIO;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIOSim;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIOTalonFX;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.DashboardTrigger;
import org.steelhawks.util.DoublePressTrigger;

public class RobotContainer {

    public static final boolean useVision = true;

    private final Trigger interruptPathfinding;
    private final Trigger isShallowEndgame;
    private final Trigger notifyAtEndgame;
    private final Trigger isDeepEndgame;
    private final Trigger nearCoralStation;
    private boolean shallowClimbMode = false;
    private boolean deepClimbMode = false;

    private final LED s_LED = LED.getInstance();
    public static AutonSelector s_Selector;
    public static Swerve s_Swerve;
    public static Vision s_Vision;
    public static Elevator s_Elevator;
    public static Intake s_Intake;
    public static Align s_Align;
    public static Climb s_Climb;

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
        nearCoralStation = new Trigger(() ->
            s_Swerve.getPose().getTranslation().getDistance(AllianceFlip.apply(FieldConstants.Position.CORAL_STATION_TOP.getPose()).getTranslation()) <= 3.0 ||
            s_Swerve.getPose().getTranslation().getDistance(AllianceFlip.apply(FieldConstants.Position.CORAL_STATION_BOTTOM.getPose()).getTranslation()) <= 3.0);

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
                            new VisionIO() {});
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOTalonFX());
                    s_Intake =
                        new Intake(
                            new AlgaeIntakeIO() {},
                            new CoralIntakeIOTalonFX());
                    s_Align =
                        new Align(
                            new AlignIOCANrange());
                    s_Climb =
                        new Climb(
                            new ShallowClimbIOTalonFX() {},
                            new DeepClimbIOTalonFX() {});
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
                    s_Intake = 
                        new Intake(
                            new AlgaeIntakeIO() {},
                            new CoralIntakeIOTalonFX());
                    s_Align =
                        new Align(
                            new AlignIO() {});
                    s_Climb =
                        new Climb(
                            new ShallowClimbIOTalonFX(),
                            new DeepClimbIOTalonFX());
    
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
                    s_Intake =
                        new Intake(
                            new AlgaeIntakeIO() {},
                            new CoralIntakeIO() {});
                    s_Align =
                        new Align(
                            new AlignIO() {});
                    s_Climb =
                        new Climb(
                            new ShallowClimbIO() {},
                            new DeepClimbIO() {});
    
                }
                case SIMBOT -> {
                    Logger.recordOutput("Pose/CoralStationTop", FieldConstants.Position.CORAL_STATION_TOP.getPose());
                    Logger.recordOutput("Pose/CoralStationBottom", FieldConstants.Position.CORAL_STATION_BOTTOM.getPose());

//                    for (Transform3d camTransform : VisionConstants.robotToCamera()) {
//                        Logger.recordOutput("Camera/" + cameraName, camTransform);
//                    }
                    for (int i = 0; i < VisionConstants.cameraNames().length; i++) {
                        Logger.recordOutput("Camera/" + VisionConstants.cameraNames()[i], VisionConstants.robotToCamera()[i]);
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
                                Swerve.getDriveSimulation()::getSimulatedDriveTrainPose));
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOSim());
                    s_Intake =
                        new Intake(
                            new AlgaeIntakeIOSim(),
                            new CoralIntakeIOSim());
                    s_Align =
                        new Align(
                            new AlignIOSim());
                    s_Climb =
                        new Climb(
                            new ShallowClimbIO() {},
                            new DeepClimbIO() {});
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
                    s_Intake =
                        new Intake(
                            new AlgaeIntakeIO() {},
                            new CoralIntakeIO() {});
                    s_Align =
                        new Align(
                            new AlignIO() {});
                    s_Climb =
                        new Climb(
                            new ShallowClimbIO() {},
                            new DeepClimbIO() {});
                }
                case HAWKRIDER -> { // hawkrider has 2 limelights and an orange pi running pv
                    s_Vision =
                        new Vision(
                            s_Swerve::accept,
                            new VisionIO() {},
                            new VisionIO() {},
                            new VisionIO() {});
                }
            }

            s_Elevator =
                new Elevator(
                    new ElevatorIO() {});
        }

        s_Selector =
            new AutonSelector("Auton Selector");

        new Alert("Tuning mode enabled", AlertType.kInfo).set(Constants.TUNING_MODE);

        configureShallowClimbEndgame();
        configurePathfindingCommands();
        configureDeepClimbEndgame();
        configureDefaultCommands();
        configureTestBindings();
        configureTriggers();
        configureOperator();
        configureDriver();
    }

    private void configurePathfindingCommands() {
        /* ------------- Pathfinding Poses ------------- */
        driver.leftTrigger()
            .whileTrue(
                DriveCommands.driveToPosition(
                    Reefstate.getClosestReef(s_Swerve.getPose()), interruptPathfinding));
    }

    private void configureDefaultCommands() {}
    private void configureTestBindings() {}

    private void configureShallowClimbEndgame() {

    }

    private void configureDeepClimbEndgame() {

    }

    private void configureTriggers() {
        s_Swerve.isPathfinding()
            .whileTrue(
                s_LED.fadeCommand(LEDColor.PURPLE));

        s_Elevator.atLimit()
            .onTrue(
                s_LED.flashCommand(LEDColor.PURPLE, 0.1, 1))
            .whileFalse(
                s_LED.setColorCommand(LEDColor.WHITE));

        s_Intake.algaeAtLimit()
            .onTrue(
                s_LED.flashCommand(LEDColor.BLUE, 0.1, 1));

        s_Intake.hasCoral()
            .onTrue(
                s_LED.flashCommand(LEDColor.GREEN, 0.1, 3));

        isShallowEndgame
            .onTrue(
                Commands.runOnce(() ->
                    s_LED.setDefaultLighting(
                        s_LED.rainbowFlashCommand())));

        isDeepEndgame
            .onTrue(
                Commands.runOnce(() ->
                    s_LED.setDefaultLighting(
                        s_LED.fadeCommand(LEDColor.BLUE))));

        notifyAtEndgame
            .whileTrue(
                new VibrateController(1.0, 5.0, driver, operator));

        nearCoralStation
            .whileTrue(
                s_Intake.intakeCoral()
                    .unless(s_Intake.hasCoral()));
    }

    private void configureDriver() {
        /* ------------- Swerve Controls ------------- */
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

//        driver.leftTrigger().whileTrue(
//            s_Align.forwardUntil(new Rotation2d()));

//        driver.leftBumper().whileTrue(
//            s_Align.alignLeft(new Rotation2d()));

        driver.rightBumper().whileTrue(
            s_Align.alignRight(new Rotation2d()));

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
        /* ------------- TEST THIS!!!!! ------------- */

        new DoublePressTrigger(operator.start())
            .onDoubleTap(
                Commands.runOnce(
                    () -> {
                        deepClimbMode = false;
                        shallowClimbMode = !shallowClimbMode;
                    }));

        new DoublePressTrigger(operator.back())
            .onDoubleTap(
                Commands.runOnce(
                    () -> {
                        shallowClimbMode = false;
                        deepClimbMode = !deepClimbMode;
                    }));

        /* ------------- Elevator Controls ------------- */

        operator.leftStick().onTrue(
            s_Elevator.toggleManualControl(
                () -> -operator.getLeftY()));

        operator.leftBumper()
            .or(new DashboardTrigger("l1"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.L1));
//
        operator.x()
            .or(new DashboardTrigger("l2"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.L2));

        operator.y()
            .or(new DashboardTrigger("l3"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.L3));

        operator.a()
            .or(new DashboardTrigger("l4"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.L4));

        // operator.b()
        //     .or(new DashboardTrigger("elevatorHome"))
        //     .onTrue(
        //         s_Elevator.setDesiredState(ElevatorConstants.State.HOME));

        operator.b()
            .or(new DashboardTrigger("elevatorHome"))
            .onTrue(
                s_Elevator.noSlamCommand());


        /* ------------- Intake Controls ------------- */

        operator.rightStick().onTrue(
            s_Intake.mAlgaeIntake.toggleManualControl(
                () -> -operator.getRightY()));

        operator.leftTrigger()
            .or(new DashboardTrigger("scoreCoral"))
            .whileTrue(
                Commands.either(
                    s_Intake.shootPulsatingCoral(),
                    s_Intake.shootCoral(),
                    () -> (s_Elevator.getDesiredState() == ElevatorConstants.State.L4.getRadians() ||
                        s_Elevator.getDesiredState() == ElevatorConstants.State.L1.getRadians()) && s_Elevator.isEnabled()));

        operator.povRight()
            .whileTrue(
                s_Intake.shootCoral());

        operator.povLeft()
            .or(new DashboardTrigger("intakeCoral"))
            .whileTrue(
                s_Intake.reverseCoral());

        operator.povRight()
            .whileTrue(
                s_Intake.intakeCoral());

        // intake algae
        operator.rightBumper().whileTrue(
            s_Intake.intakeAlgae());

        // shoot algae
        operator.rightTrigger().whileTrue(
            s_Intake.shootAlgae());

        // operator.povUp().whileTrue(
        //     s_Intake.pivotManualAlgaeUp());

        // operator.povDown().whileTrue(
        //     s_Intake.pivotManualAlgaeDown());

        // operator.povUp().onTrue(
        //     s_Climb.runShallowClimb());

        // operator.povDown().onTrue(
        //     s_Climb.runClimbViaSpeed(-0.2));
    
        // operator.povLeft().whileTrue(
        //     s_Intake.mAlgaeIntake.setDesiredState(IntakeConstants.AlgaeIntakeState.OUTTAKE));

        // operator.povLeft().whileTrue(
        //     s_Intake.mAlgaeIntake.applykS());

        // operator.povRight().whileTrue(
        //     s_Intake.mAlgaeIntake.applykV());

        // operator.povUp().whileTrue(
        //     s_Intake.mAlgaeIntake.runPivotManualUp());
            
        // operator.povDown().whileTrue(
        //     s_Intake.mAlgaeIntake.runPivotManualDown());

        // operator.povLeft().whileTrue(
        //     s_Climb.runDeepClimbViaSpeed(1));

        // operator.povRight().whileTrue(
        //     s_Climb.runDeepClimbViaSpeed(-1));
        
        // operator.povUp().whileTrue(
        //     s_Climb.runDeepClimbViaSpeed(0.2));

        // operator.povDown().whileTrue(
        //     s_Climb.runDeepClimbViaSpeed(-0.2));

        // operator.povLeft().whileTrue(
        //     s_Climb.shallowClimbCommandWithCurrent()
        //         .andThen(s_Climb.runShallowClimbViaVolts(-1)));

        // operator.povRight().whileTrue(
        //     s_Climb.shallowHomeCommandWithCurrent());

        
//        operator.povRight().whileTrue(
//            s_Intake.mAlgaeIntake.setDesiredState(IntakeConstants.AlgaeIntakeState.INTAKE));
    }
}
