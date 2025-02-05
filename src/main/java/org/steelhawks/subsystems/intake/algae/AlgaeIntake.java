package org.steelhawks.subsystems.intake.algae;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.intake.IntakeConstants;

import java.io.Console;

import static edu.wpi.first.units.Units.Volts;

public class AlgaeIntake extends SubsystemBase {

    private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();
    private final IntakeConstants constants;
    private final SysIdRoutine mAlgaeSysId;
    private boolean mEnabled = false;
    private final AlgaeIntakeIO io;

    public final ProfiledPIDController mController;
    private ArmFeedforward mFeedforward;

    private final Alert intakeMotorDisconnected;
    private final Alert pivotMotorDisconnected;
    private final Alert canCoderDisconnected;
    private final Alert limitSwitchDisconnected;
    private final Alert canCoderMagnetBad;

    public void enable() {
        mController.reset(inputs.pivotPositionRad);
        mEnabled = true;
    }

    public void disable() {
        mEnabled = false;
    }

    public AlgaeIntake(AlgaeIntakeIO io) {
        this.io = io;

        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = IntakeConstants.ALPHA;
            case HAWKRIDER -> constants = IntakeConstants.HAWKRIDER;
            default -> constants = IntakeConstants.OMEGA;
        }

        intakeMotorDisconnected =
            new Alert("Intake Motor is Disconnected", AlertType.kError);
        pivotMotorDisconnected =
            new Alert("Pivot Motor is Disconnected", AlertType.kError);
        canCoderDisconnected =
            new Alert("CANcoder is Disconnected", AlertType.kError);
        limitSwitchDisconnected =
            new Alert("Limit Switch is Disconnected", AlertType.kError);
        canCoderMagnetBad =
            new Alert("CANcoder Magnet is Bad", AlertType.kError);

        mAlgaeSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Intake/Algae/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runPivot(voltage.in(Volts)), null, this));

        mController =
            new ProfiledPIDController(
                constants.ALGAE_KP.getAsDouble(),
                constants.ALGAE_KI.getAsDouble(),
                constants.ALGAE_KD.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    constants.ALGAE_MAX_VELOCITY_PER_SEC.getAsDouble(),
                    constants.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED.getAsDouble()));

        mController.setTolerance(constants.ALGAE_TOLERANCE);

        mFeedforward =
            new ArmFeedforward(
                constants.ALGAE_KS.getAsDouble(),
                constants.ALGAE_KG.getAsDouble(),
                constants.ALGAE_KV.getAsDouble());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeIntake", inputs);

        intakeMotorDisconnected.set(!inputs.intakeConnected);
        pivotMotorDisconnected.set(!inputs.pivotConnected);
        canCoderDisconnected.set(!inputs.encoderConnected);
        limitSwitchDisconnected.set(!inputs.limitSwitchConnected);
        canCoderMagnetBad.set(!inputs.magnetGood);

        if (constants.ALGAE_KP.hasChanged(hashCode()) ||
            constants.ALGAE_KI.hasChanged(hashCode()) ||
            constants.ALGAE_KD.hasChanged(hashCode()) ||
            constants.ALGAE_MAX_VELOCITY_PER_SEC.hasChanged(hashCode()) ||
            constants.ALGAE_MAX_ACCELERATION_PER_SEC_SQUARED.hasChanged(hashCode())
        ) {
            disable();
            mController.setPID(
                constants.ALGAE_KP.getAsDouble(),
                constants.ALGAE_KI.getAsDouble(),
                constants.ALGAE_KD.getAsDouble());

            enable();
        }

        if (constants.ALGAE_KS.hasChanged(hashCode()) ||
            constants.ALGAE_KG.hasChanged(hashCode()) ||
            constants.ALGAE_KV.hasChanged(hashCode())
        ) {
            mFeedforward = new ArmFeedforward(
                constants.ALGAE_KS.getAsDouble(),
                constants.ALGAE_KG.getAsDouble(),
                constants.ALGAE_KV.getAsDouble());
        }

        if (!mEnabled) return;

        double pid =  mController.calculate(inputs.pivotPositionRad);
        TrapezoidProfile.State setpoint = mController.getSetpoint();
        double ff = mFeedforward.calculate(setpoint.position, setpoint.velocity);

        io.runPivot(pid + ff);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return mAlgaeSysId.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return mAlgaeSysId.dynamic(dir);
    }

}
