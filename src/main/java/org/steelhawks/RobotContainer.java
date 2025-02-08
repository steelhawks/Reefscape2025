package org.steelhawks;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.commands.SensorAlign;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsHawkRider;
import org.steelhawks.subsystems.LED.LEDColor;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.elevator.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIO;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIOSim;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIO;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIOSim;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIOTalonFX;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.util.AllianceFlip;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class RobotContainer {

    public static final boolean useVision = false;

    private SwerveDriveSimulation mDriveSimulation;
    private final Trigger interruptPathfinding;
    private final Trigger isAltMode;
    private boolean altMode = false;

    private final LED s_LED = LED.getInstance();
    public static Swerve s_Swerve;
    public static Vision s_Vision;
    public static Elevator s_Elevator;
    public static Intake s_Intake;
    public static SensorAlign s_SensorAlign;

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
        isAltMode = new Trigger(() -> altMode);

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
                            new VisionIO() {});
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOTalonFX());
                    s_Intake = 
                        new Intake(
                            new AlgaeIntakeIO() {},
                            new CoralIntakeIOTalonFX(IntakeConstants.ALPHA));
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
                            new VisionIOPhoton(VisionConstants.CAMERA0NAME, VisionConstants.ROBOT_TO_CAMERA0),
                            new VisionIOLimelight(VisionConstants.CAMERA1NAME, () -> s_Swerve.getRotation()),
                            new VisionIOLimelight(VisionConstants.CAMERA2NAME, () -> s_Swerve.getRotation()));
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOTalonFX());
                    s_Intake =
                        new Intake(
                            new AlgaeIntakeIO() {},
                            new CoralIntakeIO() {});
                }
                case SIMBOT -> {
                    mDriveSimulation = new SwerveDriveSimulation(Swerve.MAPLE_SIM_CONFIG, new Pose2d(3, 3,
                        new Rotation2d()));
                    SimulatedArena.getInstance().addDriveTrainSimulation(mDriveSimulation);

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
                            new VisionIOPhotonSim(VisionConstants.CAMERA0NAME, VisionConstants.ROBOT_TO_CAMERA0,
                                mDriveSimulation::getSimulatedDriveTrainPose));
                    s_Elevator =
                        new Elevator(
                            new ElevatorIOSim());
                    s_Intake =
                        new Intake(
                            new AlgaeIntakeIOSim(),
                            new CoralIntakeIOSim());
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

        s_SensorAlign =
            new SensorAlign();

        if (Constants.TUNING_MODE) {
            new Alert("Tuning mode enabled", AlertType.kInfo).set(true);
        }

        configurePathfindingCommands();
        configureDefaultCommands();
        configureTestBindings();
        configureAltBindings();
        configureTriggers();
        configureOperator();
        configureDriver();
    }

    private void configurePathfindingCommands() {
        /* ------------- Pathfinding Poses ------------- */

//        driver.leftTrigger().onTrue(
//            DriveCommands.driveToPosition(FieldConstants.PROCESSOR, interruptPathfinding));
    }

    private void configureDefaultCommands() {}
    private void configureTestBindings() {}
    private void configureAltBindings() {}

    private void configureTriggers() {
        s_Swerve.isPathfinding()
            .whileTrue(
                s_LED.fadeCommand(LEDColor.PURPLE));

        isAltMode
            .whileTrue(
                s_LED.bounceWaveCommand(LEDColor.PURPLE));

        s_Elevator.atLimit()
            .onTrue(
                s_LED.flashCommand(LEDColor.PURPLE, 0.1, 1))
            .whileFalse(
                s_LED.setColorCommand(LEDColor.WHITE));

        s_Intake.algaeAtLimit()
            .onTrue(
                s_LED.flashCommand(LEDColor.BLUE, 0.1, 1));
    }

    private void configureDriver() {
        /* ------------- Swerve Controls ------------- */
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

//        driver.x().onTrue(Commands.runOnce(s_Swerve::stopWithX, s_Swerve));

        // align robot front to processor
        driver.rightBumper().whileTrue(
            DriveCommands.joystickDriveAtAngle(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> new Rotation2d(-Math.PI / 2)));

        driver.rightTrigger().onTrue(s_Swerve.toggleMultiplier()
            .alongWith(
                Commands.either(
                    s_LED.flashCommand(LEDColor.GREEN, 0.2, 2),
                    s_LED.flashCommand(LEDColor.RED, 0.2, 2),
                    () -> s_Swerve.isSlowMode())));

        // align robot front to reef, then move left until aligned with coral branch
        driver.leftBumper().whileTrue(
            DriveCommands.joystickDriveAtAngle(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> new Rotation2d(0))
            .until(s_Swerve.alignAtGoal())
            .andThen(s_SensorAlign.alignLeft()));

        if (RobotBase.isReal()) {
            driver.b().onTrue(
                s_Swerve.zeroHeading(
                    new Pose2d(s_Swerve.getPose().getTranslation(), new Rotation2d())));
        }

        if (Constants.getMode() == Mode.SIM) {
            driver.b().onTrue(
                s_Swerve.zeroHeading(
                    new Pose2d(
                        mDriveSimulation.getSimulatedDriveTrainPose().getTranslation(), new Rotation2d())));
        }

        /* ------------- Elevator SYSID ------------- */
//        driver.povRight().whileTrue(
//            s_Elevator.applykV(RadiansPerSecond.of(2)));
    }

    private void configureOperator() {
        operator.start()
            .and(operator.back())
            .onTrue(
                Commands.runOnce(
                    () -> altMode = !altMode));

        /* ------------- Elevator Controls ------------- */

        operator.leftStick().onTrue(
            s_Elevator.toggleManualControl(
                () -> -operator.getLeftY()));

        operator.x().onTrue(
            s_Elevator.setDesiredState(ElevatorConstants.State.L2));

        operator.y().onTrue(
            s_Elevator.setDesiredState(ElevatorConstants.State.L3));

        operator.a().onTrue(
            s_Elevator.setDesiredState(ElevatorConstants.State.L4));

        operator.b().onTrue(
            s_Elevator.homeCommand());

        /* ------------- Intake Controls ------------- */

        // coral intake
        operator.leftBumper().whileTrue(
            s_Intake.intakeCoral());

        // coral shoot
        operator.leftTrigger().whileTrue(
            s_Intake.shootCoral());

        // intake algae
//        operator.rightBumper().whileTrue(
//            s_Intake.
//        )

        // shoot algae
//        operator.rightTrigger().whileTrue(
//            s_Intake.
//        )
    }
}
