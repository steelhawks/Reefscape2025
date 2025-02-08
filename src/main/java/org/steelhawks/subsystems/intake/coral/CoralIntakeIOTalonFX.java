package org.steelhawks.subsystems.intake.coral;

import org.steelhawks.Constants;
import org.steelhawks.subsystems.intake.IntakeConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import static org.steelhawks.util.PhoenixUtil.*;

public class CoralIntakeIOTalonFX implements CoralIntakeIO {
    
    private final IntakeConstants constants;
    private final TalonFX mIntakeMotor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;

    public CoralIntakeIOTalonFX(IntakeConstants constants) {
        this.constants = constants;
        mIntakeMotor = new TalonFX(constants.CORAL_INTAKE_MOTOR_ID, Constants.getCANBus());
        
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        tryUntilOk(5, () -> mIntakeMotor.getConfigurator().apply(motorConfig));

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
    public void updateInputs(CoralIntakeIOInputs inputs) {
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
