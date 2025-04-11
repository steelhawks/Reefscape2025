package org.steelhawks.util;

import edu.wpi.first.wpilibj.Alert;

import java.util.function.BooleanSupplier;

public final class AlertUtil extends VirtualSubsystem {

    private static int instances = 0;
    private BooleanSupplier condition;
    private final Alert alert;

    public AlertUtil(String message, Alert.AlertType alertType) {
        super("AlertUtil/" + instances);
        instances++;
        alert = new Alert(message, alertType);
    }

    public Alert withCondition(BooleanSupplier condition) {
        this.condition = condition;
        return alert;
    }


    @Override
    public void periodic() {
        alert.set(condition != null && condition.getAsBoolean());
    }
}
