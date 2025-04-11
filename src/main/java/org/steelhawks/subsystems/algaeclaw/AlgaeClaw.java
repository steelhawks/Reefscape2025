package org.steelhawks.subsystems.algaeclaw;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotContainer;
import org.steelhawks.util.ArmDriveFeedforward;
import org.steelhawks.util.TunableNumber;

public class AlgaeClaw extends SubsystemBase {

    private final AlgaeClawIOInputsAutoLogged inputs = new AlgaeClawIOInputsAutoLogged();
    private final AlgaeClawIO io;

    private final ProfiledPIDController mController;
    private final ArmDriveFeedforward mDriveFeedforward;
    private final ArmFeedforward mFeedforward;
    private boolean mEnabled = false;

    private void enable() {
        mEnabled = true;
        mController.reset(getPivotPosition());
    }

    private void disable() {
        mEnabled = false;
        runPivot();
    }

    public boolean isEnabled() {
        return mEnabled;
    }

    public AlgaeClaw(AlgaeClawIO io) {
        this.io = io;

        mController =
            new ProfiledPIDController(
                AlgaeClawConstants.PIVOT_KP,
                AlgaeClawConstants.PIVOT_KI,
                AlgaeClawConstants.PIVOT_KD,
                new TrapezoidProfile.Constraints(
                    AlgaeClawConstants.MAX_VELOCITY,
                    AlgaeClawConstants.MAX_ACCELERATION));
        mFeedforward =
            new ArmFeedforward(
                AlgaeClawConstants.PIVOT_KS,
                AlgaeClawConstants.PIVOT_KG,
                AlgaeClawConstants.PIVOT_KV);
        mDriveFeedforward =
            new ArmDriveFeedforward(AlgaeClawConstants.PIVOT_KG);

        disable();
    }

    public double getPivotPosition() {
        return inputs.pivotPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeClaw", inputs);

        if (mEnabled)
            runPivot();
    }

    private void runPivot() {
        double fb = mController.calculate(getPivotPosition());
        double ff = mFeedforward.calculate(mController.getSetpoint().position, mController.getSetpoint().velocity)
            + mDriveFeedforward.calculate(getPivotPosition(), RobotContainer.s_Swerve::getRobotRelativeXAccelGs);
        double volts = fb + ff;

        if ((getPivotPosition() >= AlgaeClawConstants.MAX_PIVOT_RADIANS && volts >= 0)
            || (getPivotPosition() <= AlgaeClawConstants.MIN_PIVOT_RADIANS && volts <= 0)) {
            io.stopPivot();
            return;
        }

        io.runPivot(volts);
    }

    private Command setDesiredState(AlgaeClawConstants.AlgaeClawState state) {
        return Commands.runOnce(
            () -> {
                double goal = MathUtil.clamp(
                    state.getAngle().getRadians(),
                    AlgaeClawConstants.MIN_PIVOT_RADIANS,
                    AlgaeClawConstants.MAX_PIVOT_RADIANS);
                inputs.goal = goal;
                mController.setGoal(goal);
            }
        );
    }

    public Command home() {
        return setDesiredState(AlgaeClawConstants.AlgaeClawState.HOME);
    }

    public Command avoid() {
        return setDesiredState(AlgaeClawConstants.AlgaeClawState.AVOID);
    }

    public Command intake() {
        return setDesiredState(AlgaeClawConstants.AlgaeClawState.INTAKE);
    }

    public Command catapult() {
        return setDesiredState(AlgaeClawConstants.AlgaeClawState.CATAPULT);
    }

    TunableNumber s = new TunableNumber("AlgaeClaw/kS", 0.0);
    public Command applyKS() {
        return Commands.run(
            () -> io.runPivot(s.getAsDouble())
        ).finallyDo(io::stopPivot);
    }

    public Command spin(double speed) {
        return Commands.run(
            () -> io.runSpin(speed)
        ).finallyDo(io::stopSpin);
    }
}

