package org.steelhawks.subsystems.algaeclaw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class AlgaeClawIOTalonFX implements AlgaeClawIO {

    private final TalonFXConfiguration config;
    private final CANcoderConfiguration canCoderConfig;

    private final TalonFX pivotMotor;
    private final TalonFX spinMotor;
    private final CANcoder pivotEncoder;
    private final PositionVoltage positionVoltage;

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

    public AlgaeClawIOTalonFX() {
        config = new TalonFXConfiguration();
        canCoderConfig = new CANcoderConfiguration();
        pivotMotor = new TalonFX(AlgaeClawConstants.PIVOT_ID, AlgaeClawConstants.CLAW_BUS);
        spinMotor = new TalonFX(AlgaeClawConstants.SPIN_ID, AlgaeClawConstants.CLAW_BUS);
        pivotEncoder = new CANcoder(AlgaeClawConstants.CANCODER_ID, AlgaeClawConstants.CLAW_BUS);
        positionVoltage = new PositionVoltage(0.0).withSlot(0);

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(config, 0.25));

        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = AlgaeClawConstants.CANCODER_OFFSET.getRotations();
        tryUntilOk(5, () -> pivotEncoder.getConfigurator().apply(canCoderConfig, 0.25));

        pivotPosition = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();
        pivotVoltage = pivotMotor.getSupplyVoltage();
        pivotCurrent = pivotMotor.getStatorCurrent();
        pivotTemperature = pivotMotor.getDeviceTemp();

        spinPosition = spinMotor.getPosition();
        spinVelocity = spinMotor.getVelocity();
        spinVoltage = spinMotor.getSupplyVoltage();
        spinCurrent = spinMotor.getStatorCurrent();
        spinTemperature = spinMotor.getDeviceTemp();

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
            spinTemperature);
        ParentDevice.optimizeBusUtilizationForAll(
            pivotMotor,
            spinMotor,
            pivotEncoder);
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

        inputs.encoderConnected = pivotEncoder.isConnected();
    }

    @Override
    public void runSpin(double speed) {
        spinMotor.set(speed);
    }

    @Override
    public void stopSpin() {
        spinMotor.stopMotor();
    }

    @Override
    public void runPivot(double volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public void runPivotViaSpeed(double speed) {
        pivotMotor.set(speed);
    }

    @Override
    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    @Override
    public void runPosition(Rotation2d angle, double feedforward) {
        pivotMotor.setControl(
            positionVoltage
                .withPosition(angle.getRotations())
                .withFeedForward(feedforward));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        new Thread(() -> pivotMotor.setNeutralMode(
            brake ? NeutralModeValue.Brake : NeutralModeValue.Coast)).start();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(config, 0.25));
    }
}
