package org.steelhawks.commands;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.subsystems.elevator.Elevator;

public class ElevatorCharacterize extends Command {
    private final Elevator elevator;
    private final TalonFX[] motors;
    private final Timer timer = new Timer();

    private double voltage = 0;
    private double kS = 0;
    private double kG = 0;
    private boolean foundKS = false;
    private boolean foundKG = false;

    public ElevatorCharacterize(Elevator elevator, TalonFX... motors) {
        this.elevator = elevator;
        this.motors = motors;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        voltage = 0;
        kS = 0;
        kG = 0;
        foundKS = false;
        foundKG = false;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        for (TalonFX motor : motors) {
            if (!foundKS) {
                // Slowly ramp up voltage to find kS
                voltage += 0.02;  // Small increments
                motor.setVoltage(voltage);

                if (Math.abs(motor.getVelocity().getValueAsDouble()) > 0.01) { // If elevator starts moving
                    kS = voltage;
                    foundKS = true;
                    voltage = 0; // Reset for kG test
                }
            } else if (!foundKG) {
                // Slowly increase voltage to find kG (holding voltage)
                voltage += 0.02;
                motor.setVoltage(voltage);

                if (Math.abs(motor.getVelocity().getValueAsDouble()) < 0.01 && Math.abs(elevator.getPosition() - 0) < 0.05) {
                    kG = voltage;
                    foundKG = true;
                }
            }
        }

        // Display values on SmartDashboard
        SmartDashboard.putNumber("Elevator Voltage", voltage);
        SmartDashboard.putNumber("kS", kS);
        SmartDashboard.putNumber("kG", kG);
    }

    @Override
    public void end(boolean interrupted) {
        motor.setVoltage(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return foundKS && foundKG;
    }
}
