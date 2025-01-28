package org.steelhawks;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.subsystems.LED.LEDColor;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.elevator.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIO;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIOSim;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIOTalonFX;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIO;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIOSim;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIOTalonFX;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.util.AllianceFlip;

import static org.steelhawks.subsystems.elevator.KElevator.State.*;


public class RobotContainer {

    private SwerveDriveSimulation mDriveSimulation;
    private final Trigger interruptPathfinding;

    private final LED s_LED = LED.getInstance();
    public static Swerve s_Swerve;
    public static Vision s_Vision;
    public static Elevator s_Elevator;
    public static Intake s_Intake;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    /**
     * Anything that relies on Driver Station/FMS information should run here.
     */
    public void waitForDs() {
        boolean isRed = AllianceFlip.shouldFlip();
        Color c1 = isRed ? Color.kBlue : Color.kRed;
        Color c2 = isRed ? Color.kRed : Color.kBlue;

        s_LED.setDefaultLighting(
            s_LED.movingDiscontinuousGradient(
                c1, c2));
    }

    /**
     * The implementation for MapleSim's physics simulator.
     * This only runs during the SIM mode of CURRENT_MODE.
     */
    public void updatePhysicsSimulation() {
        if (Constants.CURRENT_MODE != Mode.SIM) return;

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

    /**
     * Resets all game pieces on the MapleSim field.
     */
    public void resetSimulation() {
        if (Constants.CURRENT_MODE != Mode.SIM) return;

        Pose2d startPose = new Pose2d(3, 3, new Rotation2d());
        mDriveSimulation.setSimulationWorldPose(startPose);
        s_Swerve.setPose(startPose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public RobotContainer() {
        interruptPathfinding =
            new Trigger(() ->
                Math.abs(driver.getLeftY()) > Deadbands.DRIVE_DEADBAND ||
                Math.abs(driver.getLeftX()) > Deadbands.DRIVE_DEADBAND ||
                Math.abs(driver.getRightX()) > Deadbands.DRIVE_DEADBAND);

        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                s_Swerve =
                    new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                s_Vision =
                    new Vision(
                        s_Swerve::accept,
                        new VisionIOPhoton(KVision.CAM_01_NAME, KVision.CAM_01_TO_ROBOT));
                s_Elevator =
                    new Elevator(
                        new ElevatorIOTalonFX());
                s_Intake =
                    new Intake(
                        new AlgaeIntakeIOTalonFX(),
                        new CoralIntakeIOTalonFX());
            }
            case SIM -> {
                mDriveSimulation = new SwerveDriveSimulation(Swerve.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d()));
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
                        new VisionIOPhotonSim(KVision.CAM_01_NAME, KVision.CAM_01_TO_ROBOT,
                            mDriveSimulation::getSimulatedDriveTrainPose));
                s_Elevator =
                    new Elevator(
                        new ElevatorIOSim());
                s_Intake =
                    new Intake(
                        new AlgaeIntakeIOSim(),
                        new CoralIntakeIOSim());
            }
            default -> {
                s_Swerve =
                    new Swerve(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
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
        }

        Autos.configureTuningCommands();
        configurePathfindingCommands();
        configureDefaultCommands();
        configureTestBindings();
        configureAltBindings();
        configureTriggers();
        configureOperator();
        configureDriver();
    }

    private void configurePathfindingCommands() {}
    private void configureDefaultCommands() {}
    private void configureTestBindings() {}
    private void configureAltBindings() {}

    private void configureTriggers() {
        s_Swerve.isPathfinding()
            .whileTrue(
                s_LED.fadeCommand(LEDColor.PURPLE));

        s_Elevator.atLimit()
            .onTrue(
                s_LED.flashCommand(LEDColor.PURPLE, 0.1, 1))
            .whileFalse(
                s_LED.setColorCommand(LEDColor.WHITE));

    }

    private void configureDriver() {
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

        driver.x().onTrue(Commands.runOnce(s_Swerve::stopWithX, s_Swerve));

        driver.rightBumper().whileTrue( // align to processor
            DriveCommands.joystickDriveAtAngle(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> new Rotation2d(-Math.PI / 2)));

        driver.leftBumper().whileTrue(
            DriveCommands.joystickDriveAtAngle(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> s_Swerve.calculateTurnAngle(FieldConstants.REEF_POSE)));

        driver.rightTrigger().onTrue(s_Swerve.toggleMultiplier()
            .alongWith(
                Commands.either(
                    s_LED.flashCommand(LEDColor.GREEN, 0.2, 2),
                    s_LED.flashCommand(LEDColor.RED, 0.2, 2),
                    () -> s_Swerve.isSlowMode())));

        if (RobotBase.isReal()) {
            driver.b().onTrue(
                s_Swerve.zeroHeading(
                    new Pose2d(s_Swerve.getPose().getTranslation(), new Rotation2d())));
        }

        if (Constants.CURRENT_MODE == Mode.SIM) {
            driver.b().onTrue(
                s_Swerve.zeroHeading(
                    new Pose2d(
                        mDriveSimulation.getSimulatedDriveTrainPose().getTranslation(), new Rotation2d())));
        }


        driver.povUp().whileTrue(
            s_Elevator.elevatorManual(.1));

        driver.povDown().whileTrue(
            s_Elevator.elevatorManual(-.1));

        driver.povLeft().onTrue(
            s_Elevator.homeCommand());

        driver.povRight().onTrue(
            s_Elevator.setDesiredState(L3));
    }

    private void configureOperator() {}
}
