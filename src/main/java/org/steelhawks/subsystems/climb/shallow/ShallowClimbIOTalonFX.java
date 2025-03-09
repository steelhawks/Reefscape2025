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

    private final TalonFX mClimbMotor;

    private final StatusSignal<Angle> climbPosition;
    private final StatusSignal<AngularVelocity> climbVelocity;
    private final StatusSignal<Voltage> climbVoltage;
    private final StatusSignal<Current> climbCurrent;
    private final StatusSignal<Temperature> climbTemp;

    public ShallowClimbIOTalonFX() {
        mClimbMotor = new TalonFX(ClimbConstants.SHALLOW_MOTOR_ID, Constants.getCANBus());

        var config =
            new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(ClimbConstants.SHALLOW_GEAR_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
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
        inputs.motorConnected =
            BaseStatusSignal.refreshAll(
                climbPosition,
                climbVelocity,
                climbVoltage,
                climbCurrent,
                climbTemp).isOK();
        inputs.climbPositionRad = Units.rotationsToRadians(climbPosition.getValueAsDouble());
        inputs.climbVelocityRadPerSec = Units.rotationsToRadians(climbVelocity.getValueAsDouble());
        inputs.climbAppliedVolts = climbVoltage.getValueAsDouble();
        inputs.climbCurrentAmps = climbCurrent.getValueAsDouble();
        inputs.climbTempCelsius = climbTemp.getValueAsDouble();
    }

    @Override
    public void runClimbViaVolts(double volts) {
         mClimbMotor.setVoltage(volts);
    }

    @Override
    public void runClimbViaSpeed(double speed) {
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
