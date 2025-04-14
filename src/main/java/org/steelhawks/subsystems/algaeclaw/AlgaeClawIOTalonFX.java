package org.steelhawks.subsystems.algaeclaw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.subsystems.elevator.ElevatorConstants;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class AlgaeClawIOTalonFX implements AlgaeClawIO {

    private final TalonFX mPivotMotor;
    private final TalonFX mSpinMotor;
    private final CANcoder mPivotEncoder;
    private final MotionMagicVoltage motionMagicVoltage;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Temperature> pivotTemperature;

    private final StatusSignal<Angle> spinPosition;
    private final StatusSignal<AngularVelocity> spinVelocity;
    private final StatusSignal<Voltage> spinVoltage;
    private final StatusSignal<Current> spinCurrent;
    private final StatusSignal<Temperature> spinTemperature;

    private final StatusSignal<Angle> pivotEncoderPosition;
    private final StatusSignal<AngularVelocity> pivotEncoderVelocity;
    private final StatusSignal<Voltage> pivotEncoderVoltage;

    public AlgaeClawIOTalonFX() {
        mPivotMotor = new TalonFX(AlgaeClawConstants.PIVOT_ID, AlgaeClawConstants.CLAW_BUS);
        mSpinMotor = new TalonFX(AlgaeClawConstants.SPIN_ID, AlgaeClawConstants.CLAW_BUS);
        mPivotEncoder = new CANcoder(AlgaeClawConstants.CANCODER_ID, AlgaeClawConstants.CLAW_BUS);
        motionMagicVoltage = new MotionMagicVoltage(0.0);

        var config = new TalonFXConfiguration()
//            .withSlot0(
//                new Slot0Configs()
//                    .withGravityType(GravityTypeValue.Arm_Cosine)
//                    .withKS(AlgaeClawConstants.PIVOT_KS)
//                    .withKG(AlgaeClawConstants.PIVOT_KG)
//                    .withKV(AlgaeClawConstants.PIVOT_KV)
//                    .withKP(AlgaeClawConstants.PIVOT_KP)
//                    .withKI(AlgaeClawConstants.PIVOT_KI)
//                    .withKD(AlgaeClawConstants.PIVOT_KD))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withRemoteCANcoder(mPivotEncoder));
//            .withMotionMagic(
//                new MotionMagicConfigs()
//                    .withMotionMagicCruiseVelocity(AlgaeClawConstants.MAX_VELOCITY)
//                    .withMotionMagicAcceleration(AlgaeClawConstants.MAX_ACCELERATION)
//                    .withMotionMagicJerk(AlgaeClawConstants.MAX_JERK));
//        tryUntilOk(5, () -> mPivotMotor.getConfigurator().apply(config));
        mPivotMotor.getConfigurator().apply(config);

        var encoderConfig = new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withMagnetOffset(AlgaeClawConstants.CANCODER_OFFSET));
        tryUntilOk(5, () -> mPivotEncoder.getConfigurator().apply(encoderConfig));

        pivotPosition = mPivotMotor.getPosition();
        pivotVelocity = mPivotMotor.getVelocity();
        pivotVoltage = mPivotMotor.getSupplyVoltage();
        pivotCurrent = mPivotMotor.getStatorCurrent();
        pivotTemperature = mPivotMotor.getDeviceTemp();

        spinPosition = mSpinMotor.getPosition();
        spinVelocity = mSpinMotor.getVelocity();
        spinVoltage = mSpinMotor.getSupplyVoltage();
        spinCurrent = mSpinMotor.getStatorCurrent();
        spinTemperature = mSpinMotor.getDeviceTemp();

        pivotEncoderPosition = mPivotEncoder.getAbsolutePosition();
        pivotEncoderVelocity = mPivotEncoder.getVelocity();
        pivotEncoderVoltage = mPivotEncoder.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            pivotPosition,
            pivotVelocity,
            pivotVoltage,
            pivotCurrent,
            pivotTemperature,
            spinPosition,
            spinVelocity,
            spinVoltage,
            spinCurrent,
            spinTemperature,
            pivotEncoderPosition,
            pivotEncoderVelocity,
            pivotEncoderVoltage);
        ParentDevice.optimizeBusUtilizationForAll(
            mPivotMotor,
            mSpinMotor,
            mPivotEncoder);
    }

    @Override
    public void updateInputs(AlgaeClawIOInputs inputs) {
        inputs.pivotConnected =
            BaseStatusSignal.refreshAll(
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotCurrent,
                pivotTemperature).isOK();
        inputs.pivotPosition = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
        inputs.pivotVelocity = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
        inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
        inputs.pivotCurrent = pivotCurrent.getValueAsDouble();
        inputs.pivotTemperature = pivotTemperature.getValueAsDouble();

        inputs.spinConnected =
            BaseStatusSignal.refreshAll(
                spinPosition,
                spinVelocity,
                spinVoltage,
                spinCurrent,
                spinTemperature).isOK();
        inputs.spinPosition = Units.rotationsToRadians(spinPosition.getValueAsDouble());
        inputs.spinVelocity = Units.rotationsToRadians(spinVelocity.getValueAsDouble());
        inputs.spinAppliedVolts = spinVoltage.getValueAsDouble();
        inputs.spinCurrent = spinCurrent.getValueAsDouble();
        inputs.spinTemperature = spinTemperature.getValueAsDouble();

        inputs.encoderConnected =
            BaseStatusSignal.refreshAll(
                pivotEncoderPosition,
                pivotEncoderVelocity,
                pivotEncoderVoltage).isOK();
        inputs.encoderPosition = Units.rotationsToRadians(pivotEncoderPosition.getValueAsDouble());
        inputs.encoderVelocity = Units.rotationsToRadians(pivotEncoderVelocity.getValueAsDouble());
        inputs.encoderAppliedVolts = pivotEncoderVoltage.getValueAsDouble();
    }

    @Override
    public void runSpin(double speed) {
        mSpinMotor.set(speed);
    }

    @Override
    public void stopSpin() {
        mSpinMotor.stopMotor();
    }

    @Override
    public void runPivot(double volts) {
        mPivotMotor.setVoltage(volts);
    }

    @Override
    public void runPivotViaSpeed(double speed) {
        mPivotMotor.set(speed);
    }

    @Override
    public void stopPivot() {
        mPivotMotor.stopMotor();
    }

    @Override
    public void runPosition(double positionRad) {
        mPivotMotor.setControl(
            motionMagicVoltage.withPosition(positionRad));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        new Thread(() -> mPivotMotor.setNeutralMode(
            brake ? NeutralModeValue.Brake : NeutralModeValue.Coast)).start();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {}

    @Override
    public void setFF(double kS, double kG, double kV) {
        AlgaeClawIO.super.setFF(kS, kG, kV);
    }
}
