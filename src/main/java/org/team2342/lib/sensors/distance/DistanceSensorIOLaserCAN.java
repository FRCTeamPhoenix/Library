package org.team2342.lib.sensors.distance;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class DistanceSensorIOLaserCAN implements DistanceSensorIO {
    private final LaserCan laserCan = new LaserCan(0);
    private final Alert sensorAlert = new Alert("LaserCAN is disconnected", AlertType.kError);

    public DistanceSensorIOLaserCAN() {
        try {
            sensorAlert.set(false);
            laserCan.setRangingMode(null);
            laserCan.setTimingBudget(null);
        } catch (Exception e) {
            sensorAlert.set(true);
            System.err.println("LaserCAN configuration failed: " + e.getMessage());
        }
    }

    @Override
    public void updateInputs(DistanceSensorIOInputs inputs) {
        try {
            sensorAlert.set(false);
            inputs.distance = laserCan.getMeasurement().distance_mm;
            inputs.connected = true;
        } catch (Exception e) {
            sensorAlert.set(true);
            inputs.connected = false;
            System.err.println("Failed to read LaserCAN: " + e.getMessage());
        }
    }
}
