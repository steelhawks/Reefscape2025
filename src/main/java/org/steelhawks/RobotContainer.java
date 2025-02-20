package org.steelhawks;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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
import org.steelhawks.subsystems.climb.ClimbIO;
import org.steelhawks.subsystems.climb.ClimbIOTalonFX;
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

    private SwerveDriveSimulation mDriveSimulation;
    private final Trigger interruptPathfinding;
    private final Trigger isShallowEndgame;
    private final Trigger notifyAtEndgame;
    private final Trigger isDeepEndgame;
    private boolean shallowClimbMode = false;
    private boolean deepClimbMode = false;

    private final LED s_LED = LED.getInstance();
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

    public void updatePhysicsSimulation() {
        if (Constants.getMode() != Mode.SIM) return;

        // physics sim to simulate the field
        SimulatedArena.getInstance().simulationPeriodic();

        Pose3d[] algaePoses =
            SimulatedArena.getInstance()
                .getGamePiecesArrayByType("Algae");
        Pose3d[] coralPoses =
            SimulatedArena.getInstance()
                .getGamePiecesArrayByType("Coral");

        Logger.recordOutput("FieldSimulation/AlgaePoses", algaePoses);
        Logger.recordOutput("FieldSimulation/CoralPoses", coralPoses);
        Logger.recordOutput("FieldSimulation/RobotPosition", mDriveSimulation.getSimulatedDriveTrainPose());
    }

    public void resetSimulation(Pose2d startingPose) {
        if (Constants.getMode() != Mode.SIM) return;

        mDriveSimulation.setSimulationWorldPose(startingPose);
        s_Swerve.setPose(startingPose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public RobotContainer() {
        interruptPathfinding =
            new Trigger(() ->
                Math.abs(driver.getLeftY()) > Deadbands.DRIVE_DEADBAND ||
                Math.abs(driver.getLeftX()) > Deadbands.DRIVE_DEADBAND ||
                Math.abs(driver.getRightX()) > Deadbands.DRIVE_DEADBAND);
        isShallowEndgame = new Trigger(() -> shallowClimbMode);
        isDeepEndgame = new Trigger(() -> deepClimbMode);
        notifyAtEndgame = new Trigger(() ->
            Robot.getState() == RobotState.TELEOP && DriverStation.getMatchTime() <= Constants.ENDGAME_PERIOD);

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
                            new ElevatorIO() {});
                    s_Intake =
                        new Intake(
                            new AlgaeIntakeIO() {},
                            new CoralIntakeIO() {});
                    s_Align =
                        new Align(
                            new AlignIOCANrange());
                    s_Climb = 
                        new Climb(
                            new ClimbIO() {});
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
                            new AlgaeIntakeIOTalonFX(),
                            new CoralIntakeIOTalonFX());
                    s_Align =
                        new Align(
                            new AlignIOCANrange());
                    s_Climb = 
                        new Climb(
                            new ClimbIOTalonFX());
    
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
                            new ClimbIO() {});
    
                }
                case SIMBOT -> {
                    mDriveSimulation = new SwerveDriveSimulation(Swerve.MAPLE_SIM_CONFIG,
                        new Pose2d(3, 3, new Rotation2d()));
                    SimulatedArena.getInstance().addDriveTrainSimulation(mDriveSimulation);

                    Logger.recordOutput("Pose/CoralStationTop", FieldConstants.CORAL_STATION_TOP);
                    Logger.recordOutput("Pose/CoralStationBottom", FieldConstants.CORAL_STATION_BOTTOM);

                    s_Swerve =
                        new Swerve(
                            new GyroIOSim(mDriveSimulation.getGyroSimulation()),
                            new ModuleIOSim(mDriveSimulation.getModules()[0]),
                            new ModuleIOSim(mDriveSimulation.getModules()[1]),
                            new ModuleIOSim(mDriveSimulation.getModules()[2]),
                            new ModuleIOSim(mDriveSimulation.getModules()[3]));
                    s_Vision =
                        new Vision(
                            s_Swerve::accept,
                            new VisionIOPhotonSim(VisionConstants.cameraNames()[0], VisionConstants.robotToCamera()[0],
                                mDriveSimulation::getSimulatedDriveTrainPose));
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
                            new ClimbIO() {});
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
                            new ClimbIO() {});
    
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

        if (Constants.TUNING_MODE) {
            new Alert("Tuning mode enabled", AlertType.kInfo).set(true);
        }

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
        
        s_Climb.atOuterLimit()
            .onTrue(
                s_LED.flashCommand(LEDColor.HOT_PINK, 0.1, 1));

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
    }

    private void configureDriver() {
        /* ------------- Swerve Controls ------------- */
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

        driver.leftTrigger().whileTrue(
            s_Align.forwardUntil(new Rotation2d()));

        driver.leftBumper().whileTrue(
            s_Align.alignLeft(new Rotation2d()));

        driver.rightBumper().whileTrue(
            s_Align.alignRight(new Rotation2d()));

        driver.rightTrigger().onTrue(s_Swerve.toggleMultiplier()
            .alongWith(
                Commands.either(
                    s_LED.flashCommand(LEDColor.GREEN, 0.2, 2),
                    s_LED.flashCommand(LEDColor.RED, 0.2, 2),
                    () -> s_Swerve.isSlowMode())));

        driver.b().onTrue(
            s_Swerve.zeroHeading(
                new Pose2d(s_Swerve.getPose().getTranslation(), new Rotation2d())));

//        if (RobotBase.isReal()) {
//            driver.b().onTrue(
//                s_Swerve.zeroHeading(
//                    new Pose2d(s_Swerve.getPose().getTranslation(), new Rotation2d())));
//        }
//
//        if (Constants.getMode() == Mode.SIM) {
//            driver.b().onTrue(
//                s_Swerve.zeroHeading(
//                    new Pose2d(
//                        mDriveSimulation.getSimulatedDriveTrainPose().getTranslation(), new Rotation2d())));
//        }
    }

    private void configureOperator() {
//        operator.start()
//            .and(operator.back())
//            .onTrue(
//                Commands.runOnce(
//                    () -> altMode = !altMode));

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

//        operator.x().whileTrue(
//            s_Elevator.applyVolts(1));

        // L1
        operator.leftBumper()
            .or(new DashboardTrigger("l1"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.L1));

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

        // operator.b().onTrue(
        //     s_Elevator.homeCommand());
        operator.b()
            .or(new DashboardTrigger("elevatorHome"))
            .onTrue(
                s_Elevator.setDesiredState(ElevatorConstants.State.HOME));

        /* ------------- Intake Controls ------------- */

        operator.rightStick().onTrue(
            s_Intake.mAlgaeIntake.toggleManualControl(
                () -> -operator.getRightY()));

        // coral shoot
        // operator.leftTrigger().whileTrue(
        //     s_Intake.shootCoral()
        // );
        
        operator.leftTrigger()
            .or(new DashboardTrigger("scoreCoral"))
            .whileTrue(
                Commands.either(
                    s_Intake.shootCoralSlow(),
                    s_Intake.shootCoral(),
                    () -> s_Elevator.getDesiredState() == ElevatorConstants.State.L4.getRadians() && s_Elevator.isEnabled()));

        operator.povLeft()
            .or(new DashboardTrigger("intakeCoral"))
            .whileTrue(
                s_Intake.reverseCoral());

        // intake algae
        operator.rightBumper().whileTrue(
            s_Intake.intakeAlgae());

        // shoot algae
        operator.rightTrigger().whileTrue(
            s_Intake.shootAlgae());

        operator.povUp().whileTrue(
            s_Intake.pivotManualAlgaeUp());

        operator.povDown().whileTrue(
            s_Intake.pivotManualAlgaeDown());

        // operator.povUp().onTrue(
        //     s_Climb.climbCommandWithCurrent());

        // operator.povDown().onTrue(
        //     s_Climb.homeCommandWithCurrent());

        // operator.povUp().onTrue(
        //     s_Climb.runClimbViaSpeed(0.2));

        // operator.povDown().onTrue(
        //     s_Climb.runClimbViaSpeed(-0.2));
    
        // operator.povLeft().whileTrue(
        //     s_Intake.mAlgaeIntake.setDesiredState(IntakeConstants.AlgaeIntakeState.HOME));

        
        
        operator.povRight().whileTrue(
            s_Intake.mAlgaeIntake.setDesiredState(IntakeConstants.AlgaeIntakeState.INTAKE));
    }
}
