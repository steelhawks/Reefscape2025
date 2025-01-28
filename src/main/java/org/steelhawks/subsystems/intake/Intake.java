package org.steelhawks.subsystems.intake;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.intake.algae.AlgaeIntake;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIO;
import org.steelhawks.subsystems.intake.coral.CoralIntake;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIO;

import static edu.wpi.first.units.Units.Volts;

public class Intake extends SubsystemBase {

    private final AlgaeIntake mAlgaeIntake;
    private final CoralIntake mCoralIntake;

    private final SysIdRoutine mAlgaeSysId;
    private final SysIdRoutine mCoralSysId;

    public Intake(AlgaeIntakeIO algaeIO, CoralIntakeIO coralIO) {
        mAlgaeIntake = new AlgaeIntake(algaeIO);
        mCoralIntake = new CoralIntake(coralIO);

        mAlgaeSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Intake/Algae/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> mAlgaeIntake.runCharacterization(voltage.in(Volts)), null, this));

        mCoralSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Intake/Algae/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> mCoralIntake.runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        mAlgaeIntake.periodic();
        mCoralIntake.periodic();
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command algaeSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return mAlgaeSysId.quasistatic(dir);
    }

    public Command algaeSysIdDynamic(SysIdRoutine.Direction dir) {
        return mAlgaeSysId.dynamic(dir);
    }

    public Command coralSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return mCoralSysId.quasistatic(dir);
    }

    public Command coralSysIdDynamic(SysIdRoutine.Direction dir) {
        return mCoralSysId.dynamic(dir);
    }
}

