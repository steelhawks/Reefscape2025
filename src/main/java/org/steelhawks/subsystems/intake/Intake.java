package org.steelhawks.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.LED.LEDColor;
import org.steelhawks.subsystems.intake.algae.AlgaeIntake;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIO;
import org.steelhawks.subsystems.intake.coral.CoralIntake;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIO;

public class Intake {

    private final IntakeConstants constants;

    public final AlgaeIntake mAlgaeIntake;
    public final CoralIntake mCoralIntake;

    public Intake(AlgaeIntakeIO algaeIO, CoralIntakeIO coralIO) {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = IntakeConstants.ALPHA;
            case HAWKRIDER -> constants = IntakeConstants.HAWKRIDER;
            default -> constants = IntakeConstants.OMEGA;
        }

        mAlgaeIntake = new AlgaeIntake(algaeIO);
        mCoralIntake = new CoralIntake(coralIO);
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Trigger hasCoral() {
        return mCoralIntake.hasCoral();
    }

    public Trigger algaeAtGoal() {
        return mAlgaeIntake.atGoal();
    }

    public Trigger algaeAtLimit() {
        return mAlgaeIntake.atLimit();
    }

    public Command setDesiredState(IntakeConstants.AlgaeIntakeState state) {
        return Commands.runOnce(
            () -> mAlgaeIntake.setDesiredState(state), mAlgaeIntake);
    }

    public Command homeAlgae() {
        return mAlgaeIntake.homeCommand();
    }

    public Command pivotManualAlgae(boolean isUp) {
        return mAlgaeIntake.runPivotManual(isUp);
    }

    public Command pivotManualAlgaeUp() {
        return mAlgaeIntake.runPivotManualUp();
    }

    public Command pivotManualAlgaeDown() {
        return mAlgaeIntake.runPivotManualDown();
    }


    public Command intakeAlgae() {
        return mAlgaeIntake.intake();
    }

    public Command shootAlgae() {
        return mAlgaeIntake.outtake();
    }

    public Command shootCoralSlow() {
        return Commands.run(
            () -> mCoralIntake.shootSlowCoral(), mCoralIntake)
//            .alongWith(LED.getInstance().flashCommand(LEDColor.WHITE, 0.2, 2))
            .finallyDo(() -> mCoralIntake.stop());
    }

    public Command shootPulsatingCoral() {
        return Commands.sequence(
                Commands.run(() -> mCoralIntake.shootSlowCoral(), mCoralIntake).withTimeout(0.025),
                Commands.run(() -> mCoralIntake.stop(), mCoralIntake).withTimeout(0.025)
            ).repeatedly()
            .alongWith(LED.getInstance().flashCommand(LEDColor.WHITE, 0.2, 2))
            .finallyDo(() -> mCoralIntake.stop());
    }

    public Command shootCoral() {
        return Commands.run(
            () -> mCoralIntake.shootCoral(), mCoralIntake)
            .alongWith(LED.getInstance().flashCommand(LEDColor.WHITE, 0.2, 2))
            .finallyDo(() -> mCoralIntake.stop());
    }

    public Command reverseCoral() {
        return Commands.run(
            () -> mCoralIntake.reverseCoral(), mCoralIntake)
            .alongWith(LED.getInstance().flashCommand(LEDColor.PINK, 0.2, 2))
            .finallyDo(() -> mCoralIntake.stop());
    }

    public Command intakeCoral() {
        return Commands.run(
            () -> mCoralIntake.intakeCoral())
            .alongWith(LED.getInstance().flashCommand(LEDColor.GREEN, 0.2, 2))
            .finallyDo(() -> mCoralIntake.stop());
    }
}

