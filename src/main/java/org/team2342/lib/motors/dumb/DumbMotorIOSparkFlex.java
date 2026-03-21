package org.team2342.lib.motors.dumb;

import org.team2342.lib.motors.MotorConfig;
import org.team2342.lib.motors.dumb.DumbMotorIO.DumbMotorIOInputs;

import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;

public class DumbMotorIOSparkFlex {
    
    private final SparkFlex motor;
    private final SparkFlexConfig motorConfig = new SparkFlexConfig();
    private final Debouncer connectedDebouncer = new Debouncer(0.5); 

    public DumbMotorIOSparkFlex(int canID, MotorConfig config, MotorType type) {
        motor = new SparkFlex(canID, type);
        motorConfig.inverted(config.motorInverted); 
        //motorConfig.idleMode(config.idleMode);

        motor.configure(motorConfig , ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void updateInputs(DumbMotorIOInputs inputs) {
        inputs.connected = connectedDebouncer.calculate(true);
        inputs.appliedVolts
    }

    
}
