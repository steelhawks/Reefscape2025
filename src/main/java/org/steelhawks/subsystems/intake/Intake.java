package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.intake.IntakeConstants.AlgaeIntakeState;
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

    public Trigger algaeAtGoal() {
        return mAlgaeIntake.atGoal();
    }

    public Trigger algaeAtLimit() {
        return mAlgaeIntake.atLimit();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return mAlgaeIntake.sysIdQuasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return mAlgaeIntake.sysIdDynamic(dir);
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

    public Command intakeAlgae() {
        return mAlgaeIntake.intake();
    }

    public Command shootAlgae() {
        return mAlgaeIntake.outtake();
    }

    public Command shootCoral() {
        return Commands.run(
            () -> mCoralIntake.runOuttake(), mCoralIntake)
            .finallyDo(() -> mCoralIntake.stop());
    }

    public Command intakeCoral() {
        return Commands.run(
            () -> mCoralIntake.runIntake(), mCoralIntake)
            .until(mCoralIntake.hasCoral())
            .finallyDo(() -> mCoralIntake.stop());
    }
}

