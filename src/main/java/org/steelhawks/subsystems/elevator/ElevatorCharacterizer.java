package org.steelhawks.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.steelhawks.util.TunableNumber;

public class ElevatorCharacterizer {

    @FunctionalInterface
    public interface ElevatorConsumer {
        void accept(double volts);
    }

    private final TunableNumber kS = new TunableNumber("Elevator/kS");
    private final TunableNumber kG = new TunableNumber("Elevator/kG");
    private final TunableNumber kV = new TunableNumber("Elevator/kV");

    private final ElevatorConsumer consumer;
    private final SubsystemBase elevator;

    public ElevatorCharacterizer(ElevatorConsumer c, SubsystemBase subsystem) {
        this.consumer = c;
        this.elevator = subsystem;
    }

    public Command runkS(boolean up) {
        return new StartEndCommand(
            () -> consumer.accept(up ? 1 : -1 * kS.get()),
            () -> consumer.accept(0.0),
                elevator);
    }

    public Command runkG(boolean up) {
        return new StartEndCommand(
            () -> consumer.accept(kG.get() + (up ? 1 : -1 * kS.get())),
            () -> consumer.accept(0.0),
                elevator);
    }

    public Command runkV(boolean up) {
        return new StartEndCommand(
            () -> consumer.accept(kG.get() + (up ? 1 : -1 * kV.get())),
            () -> consumer.accept(0.0),
                elevator);
    }
}
