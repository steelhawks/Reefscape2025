package org.steelhawks.subsystems.climb.deep;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.climb.ClimbConstants;

public class DeepClimbIOTalonFX implements DeepClimbIO {

    private final ClimbConstants constants;

    private final TalonFX mTopMotor;
    private final TalonFX mBottomMotor;
    private final CANcoder mPivotEncoder;

    private final StatusSignal<Angle> topPosition;
    private final StatusSignal<AngularVelocity> topVelocity;
    private final StatusSignal<Voltage> topVoltage;
    private final StatusSignal<Current> topCurrent;
    private final StatusSignal<Temperature> topTemperature;

    private final StatusSignal<Angle> bottomPosition;
    private final StatusSignal<AngularVelocity> bottomVelocity;
    private final StatusSignal<Voltage> bottomVoltage;
    private final StatusSignal<Current> bottomCurrent;
    private final StatusSignal<Temperature> bottomTemperature;

    private final StatusSignal<Boolean> magnetFault;
    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<Angle> pivotAbsolutePosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;

    public DeepClimbIOTalonFX() {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = ClimbConstants.ALPHA;
            case HAWKRIDER -> constants = ClimbConstants.HAWKRIDER;
            default -> constants = ClimbConstants.OMEGA;
        }

        mTopMotor = new TalonFX(constants.DEEP_TOP_MOTOR_ID, Constants.getCANBus());
        mBottomMotor = new TalonFX(constants.DEEP_BOTTOM_MOTOR_ID, Constants.getCANBus());
        mPivotEncoder = new CANcoder(constants.DEEP_CANCODER_ID, Constants.getCANBus());

        var topConfig =
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));

        var bottomConfig =
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));


        mTopMotor.getConfigurator().apply(topConfig);
        mBottomMotor.getConfigurator().apply(bottomConfig);

        topPosition = mTopMotor.getPosition();
        topVelocity = mTopMotor.getVelocity();
        topVoltage = mTopMotor.getMotorVoltage();
        topCurrent = mTopMotor.getStatorCurrent();
        topTemperature = mTopMotor.getDeviceTemp();

        bottomPosition = mBottomMotor.getPosition();
        bottomVelocity = mBottomMotor.getVelocity();
        bottomVoltage = mBottomMotor.getMotorVoltage();
        bottomCurrent = mBottomMotor.getStatorCurrent();
        bottomTemperature = mBottomMotor.getDeviceTemp();

        magnetFault = mPivotEncoder.getFault_BadMagnet();
        pivotPosition = mPivotEncoder.getPosition();
        pivotAbsolutePosition = mPivotEncoder.getAbsolutePosition();
        pivotVelocity = mPivotEncoder.getVelocity();
        pivotVoltage = mPivotEncoder.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            topPosition,
            topVelocity,
            topVoltage,
            topCurrent,
            topTemperature,

            bottomPosition,
            bottomVelocity,
            bottomVoltage,
            bottomCurrent,
            bottomTemperature,

            magnetFault,
            pivotPosition,
            pivotAbsolutePosition,
            pivotVelocity,
            pivotVoltage);

        ParentDevice.optimizeBusUtilizationForAll(mTopMotor, mBottomMotor, mPivotEncoder);
    }

    @Override
    public void updateInputs(DeepClimbIOInputs inputs) {
        inputs.topConnected =
            BaseStatusSignal.refreshAll(
                topPosition,
                topVelocity,
                topVoltage,
                topCurrent,
                topTemperature).isOK();
        inputs.topClimbPositionRad = Units.rotationsToRadians(topPosition.getValueAsDouble());
        inputs.topClimbVelocityRadPerSec = Units.rotationsToRadians(topVelocity.getValueAsDouble());
        inputs.topClimbAppliedVolts = topVoltage.getValueAsDouble();
        inputs.topClimbCurrentAmps = topCurrent.getValueAsDouble();
        inputs.topClimbTempCelsius = topTemperature.getValueAsDouble();

        inputs.bottomConnected =
            BaseStatusSignal.refreshAll(
                bottomPosition,
                bottomVelocity,
                bottomVoltage,
                bottomCurrent,
                bottomTemperature).isOK();
        inputs.bottomClimbPositionRad = Units.rotationsToRadians(bottomPosition.getValueAsDouble());
        inputs.bottomClimbVelocityRadPerSec = Units.rotationsToRadians(bottomVelocity.getValueAsDouble());
        inputs.bottomClimbAppliedVolts = bottomVoltage.getValueAsDouble();
        inputs.bottomClimbCurrentAmps = bottomCurrent.getValueAsDouble();
        inputs.bottomClimbTempCelsius = bottomTemperature.getValueAsDouble();

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
        mTopMotor.setVoltage(volts);
        mBottomMotor.setVoltage(volts);
    }

    @Override
    public void runClimbViaSpeed(double speed) {
        mTopMotor.set(speed);
        mBottomMotor.set(speed);
    }

    @Override
    public void stop() {
        mTopMotor.stopMotor();
        mBottomMotor.stopMotor();
    }
}
