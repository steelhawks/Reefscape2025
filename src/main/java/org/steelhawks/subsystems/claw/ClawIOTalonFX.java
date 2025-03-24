package org.steelhawks.subsystems.claw;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.*;
import org.steelhawks.Constants;
import org.steelhawks.Constants.RobotType;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ClawIOTalonFX implements ClawIO {

    private final TalonFX mIntakeMotor;
    private CANrange mBeamBreak = null;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;

    private StatusSignal<Distance> distance = null;
    private StatusSignal<Boolean> beamBroken = null;

    public ClawIOTalonFX() {
        CANBus canBus = Constants.getCANBus();
        if (Constants.getRobot() == RobotType.OMEGABOT) // claw is on the rio bus on omega
            canBus = new CANBus();
        mIntakeMotor = new TalonFX(ClawConstants.CLAW_INTAKE_MOTOR_ID, canBus);
        if (Constants.getRobot() == RobotType.OMEGABOT) {
            mBeamBreak = new CANrange(ClawConstants.CAN_RANGE_ID_OMEGA, canBus);
            var canRangeConfig =
                new CANrangeConfiguration()
                    .withProximityParams(new ProximityParamsConfigs()
                        .withProximityThreshold(Claw.DIST_TO_HAVE_CORAL));
            mBeamBreak.getConfigurator().apply(canRangeConfig);
            distance = mBeamBreak.getDistance();
            beamBroken = mBeamBreak.getIsDetected();
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                distance,
                beamBroken);
            mBeamBreak.optimizeBusUtilization();
        }


        var motorConfig =
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
        mIntakeMotor.getConfigurator().apply(motorConfig);

        position = mIntakeMotor.getPosition();
        velocity = mIntakeMotor.getVelocity();
        voltage = mIntakeMotor.getSupplyVoltage();
        current = mIntakeMotor.getStatorCurrent();
        temp = mIntakeMotor.getDeviceTemp();
        
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            position,
            velocity,
            voltage,
            current,
            temp);

        ParentDevice.optimizeBusUtilizationForAll(mIntakeMotor);
    }

    @Override
    public void updateInputs(ClawIntakeIOInputs inputs) {
        inputs.connected =
            BaseStatusSignal.refreshAll(
                position,
                velocity,
                voltage,
                current,
                temp).isOK();
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();

        inputs.beamConnected = mBeamBreak != null && mBeamBreak.isConnected();

        if (mBeamBreak != null) {
            inputs.beamDistance = distance.refresh().getValueAsDouble();
            inputs.beamBroken = beamBroken.refresh().getValue();
        }
    }

    @Override
    public void runIntake(double percentageOutput) {
        mIntakeMotor.set(percentageOutput);
    }

    @Override
    public void stop() {
        mIntakeMotor.stopMotor();
    }
}
