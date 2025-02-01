package org.steelhawks.subsystems.intake;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.intake.algae.AlgaeIntake;
import org.steelhawks.subsystems.intake.algae.AlgaeIntakeIO;
import org.steelhawks.subsystems.intake.coral.CoralIntake;
import org.steelhawks.subsystems.intake.coral.CoralIntakeIO;

import static edu.wpi.first.units.Units.Volts;

public class Intake extends SubsystemBase {

    private final AlgaeIntake mAlgaeIntake;
    private final CoralIntake mCoralIntake;

    private final IntakeConstants constants;

    private final SysIdRoutine mAlgaeSysId;


    public Intake(AlgaeIntakeIO algaeIO, CoralIntakeIO coralIO, IntakeConstants constants) {
        mAlgaeIntake = new AlgaeIntake(algaeIO, constants);
        mCoralIntake = new CoralIntake(coralIO, constants);
        this.constants = constants;

        mAlgaeSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Intake/Algae/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> mAlgaeIntake.runCharacterization(voltage.in(Volts)), null, this));
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

    public Command setDesiredAlgaeIntakeState(IntakeConstants.AlgaeIntakeState state) {
        return Commands.runOnce(
            () -> {
                double goal =
                    MathUtil.clamp(state.rotations, 0, constants.ALGAE_MAX_ROTATIONS);
//                inputs.setpoint = goal;
                mAlgaeIntake.mController.setGoal(goal);
                mAlgaeIntake.enable();
            }, this);
    }

    
}

