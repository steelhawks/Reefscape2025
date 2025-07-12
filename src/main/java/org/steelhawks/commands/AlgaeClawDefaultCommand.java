package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.algaeclaw.AlgaeClaw;
import org.steelhawks.subsystems.algaeclaw.AlgaeClawConstants;

public class AlgaeClawDefaultCommand extends Command {

    private static final AlgaeClaw s_AlgaeClaw = RobotContainer.s_AlgaeClaw;

    public AlgaeClawDefaultCommand() {
        addRequirements(s_AlgaeClaw);
    }

    @Override
    public void execute() {
        if (s_AlgaeClaw.hasAlgae()) {
            s_AlgaeClaw.intakeAlgae(AlgaeClawConstants.RETAIN_ALGAE_SPEED);
        } else if (!s_AlgaeClaw.hasAlgae()) {
            s_AlgaeClaw.stopSpin();
        }
    }
}
