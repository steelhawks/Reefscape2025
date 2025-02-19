package org.steelhawks.subsystems.climb.shallow;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.climb.ClimbConstants;

public class ShallowClimbIOTalonFX implements ShallowClimbIO {

    private final ClimbConstants constants;

    private final TalonFX mClimbMotor;

    private final StatusSignal<Angle> climbPosition;
    private final StatusSignal<AngularVelocity> climbVelocity;
    private final StatusSignal<Voltage> climbVoltage;
    private final StatusSignal<Current> climbCurrent;
    private final StatusSignal<Temperature> climbTemp;

    private boolean atOutsideLimit = false;
    private boolean atInsideLimit = false;

    public ShallowClimbIOTalonFX() {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = ClimbConstants.ALPHA;
            case HAWKRIDER -> constants = ClimbConstants.HAWKRIDER;
            default -> constants = ClimbConstants.OMEGA;
        }

        mClimbMotor = new TalonFX(constants.SHALLOW_MOTOR_ID, Constants.getCANBus());

        var config =
            new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(constants.SHALLOW_GEAR_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));

        mClimbMotor.getConfigurator().apply(config);

        zeroEncoders();

        climbPosition = mClimbMotor.getPosition();
        climbVelocity = mClimbMotor.getVelocity();
        climbVoltage = mClimbMotor.getSupplyVoltage();
        climbCurrent = mClimbMotor.getStatorCurrent();
        climbTemp = mClimbMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            climbPosition,
            climbVelocity,
            climbVoltage,
            climbCurrent,
            climbTemp);

        ParentDevice.optimizeBusUtilizationForAll(mClimbMotor);
    }

    @Override
    public void updateInputs(ShallowClimbIOInputs inputs) {
        double climbPos = climbPosition.getValueAsDouble();
        double climbVelo = climbVelocity.getValueAsDouble();

        inputs.motorConnected =
            BaseStatusSignal.refreshAll(
                climbPosition,
                climbVelocity,
                climbVoltage,
                climbCurrent,
                climbTemp).isOK();
        inputs.climbPositionRad = Units.rotationsToRadians(climbPos);
        inputs.climbVelocityRadPerSec = Units.rotationsToRadians(climbVelo);
        inputs.climbAppliedVolts = climbVoltage.getValueAsDouble();
        inputs.climbCurrentAmps = climbCurrent.getValueAsDouble();
        inputs.climbTempCelsius = climbTemp.getValueAsDouble();

        atOutsideLimit = inputs.atOutsideLimit;
        atInsideLimit = inputs.atInsideLimit;
    }

    @Override
    public void runClimb(double volts) {
        boolean stopClimb = (atOutsideLimit && volts > 0) || (atInsideLimit && volts < 0);
        Logger.recordOutput("Climb/StopClimb", stopClimb);
        if (stopClimb) {
            stop();
            return;
        }

         mClimbMotor.setVoltage(volts);
    }

    @Override
    public void runClimbViaSpeed(double speed) {
        boolean isGoingOut = Math.abs(speed) == speed;
        if ((atOutsideLimit && isGoingOut) || (atInsideLimit && !isGoingOut)) {
            stop();
            return;
        }

         mClimbMotor.set(speed);
    }

    @Override
    public void zeroEncoders() {
        tryUntilOk(5, () -> mClimbMotor.setPosition(0));
    }

    @Override
    public void stop() {
        mClimbMotor.stopMotor();
    }
}
