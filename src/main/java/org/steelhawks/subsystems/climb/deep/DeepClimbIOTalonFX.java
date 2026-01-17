package org.steelhawks.subsystems.climb.deep;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.climb.ClimbConstants;

public class DeepClimbIOTalonFX implements DeepClimbIO {

    private final TalonFX mPivotMotor;
    private final CANcoder mPivotEncoder;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;
    private final StatusSignal<Temperature> motorTemperature;

    private final StatusSignal<Boolean> magnetFault;
    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<Angle> pivotAbsolutePosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;

    public DeepClimbIOTalonFX() {
        mPivotMotor = new TalonFX(ClimbConstants.DEEP_TOP_MOTOR_ID, Constants.getCANBus());
        mPivotEncoder = new CANcoder(ClimbConstants.DEEP_CANCODER_ID, Constants.getCANBus());

        var config =
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
        mPivotMotor.getConfigurator().apply(config);

        motorPosition = mPivotMotor.getPosition();
        motorVelocity = mPivotMotor.getVelocity();
        motorVoltage = mPivotMotor.getMotorVoltage();
        motorCurrent = mPivotMotor.getStatorCurrent();
        motorTemperature = mPivotMotor.getDeviceTemp();


        var encoderConfig =
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withMagnetOffset(ClimbConstants.ANGLE_OFFSET_TO_HORIZONTAL)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        mPivotEncoder.getConfigurator().apply(encoderConfig);

        magnetFault = mPivotEncoder.getFault_BadMagnet();
        pivotPosition = mPivotEncoder.getPosition();
        pivotAbsolutePosition = mPivotEncoder.getAbsolutePosition();
        pivotVelocity = mPivotEncoder.getVelocity();
        pivotVoltage = mPivotEncoder.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            magnetFault,
            pivotPosition,
            pivotAbsolutePosition,
            pivotVelocity,
            pivotVoltage);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            motorPosition,
            motorVelocity,
            motorVoltage,
            motorCurrent,
            motorTemperature);
        ParentDevice.optimizeBusUtilizationForAll(mPivotMotor, mPivotEncoder);
    }

    @Override
    public void updateInputs(DeepClimbIOInputs inputs) {
        inputs.connected =
            BaseStatusSignal.refreshAll(
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorCurrent,
                motorTemperature).isOK();
        inputs.climbPositionRad = Units.rotationsToRadians(motorPosition.getValueAsDouble());
        inputs.climbVelocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
        inputs.climbAppliedVolts = motorVoltage.getValueAsDouble();
        inputs.climbCurrentAmps = motorCurrent.getValueAsDouble();
        inputs.climbTempCelsius = motorTemperature.getValueAsDouble();

        inputs.encoderConnected =
            BaseStatusSignal.refreshAll(
                magnetFault,
                pivotPosition,
                pivotAbsolutePosition,
                pivotVelocity,
                pivotVoltage).isOK();
        inputs.magnetGood = !magnetFault.getValue();
        inputs.encoderPositionRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
        inputs.encoderAbsolutePositionRad = Units.rotationsToRadians(pivotAbsolutePosition.getValueAsDouble());
        inputs.encoderVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
    }

    @Override
    public void runClimb(double volts) {
        mPivotMotor.setVoltage(volts);
    }

    @Override
    public void runClimbViaSpeed(double speed) {
        mPivotMotor.set(speed);
    }

    @Override
    public void stop() {
        mPivotMotor.stopMotor();
    }
}
