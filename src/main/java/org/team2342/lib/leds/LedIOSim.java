package org.team2342.lib.leds;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Simulation implementation of LedIO 
 * Logs RGB values 
 * 
 */
public class LedIOSim implements LedIO {
    private Color firstHalfColor = Color.kWhite;
    private Color secondHalfColor = Color.kWhite;

    /**
     * Sets the color for the first or second half of the LED strip
     * 
     * @param color The color to set for the half
     */
    @Override
    public void setFirstHalfColor(Color color) {
        firstHalfColor = color;
    }

    @Override
    public void setSecondHalfColor(Color color) {
        secondHalfColor = color;
    }

    /**
     * Called periodically to update the LED input data
     * 
     * @param inputs The LedIOInputs object to update 
     */
    @Override
    public void updateInputs(LedIOInputs inputs) {
        inputs.firstHalfColor = firstHalfColor;
        inputs.secondHalfColor = secondHalfColor;

        Logger.recordOutput("LED/FirstHalf/Red", firstHalfColor.red);
        Logger.recordOutput("LED/FirstHalf/Green", firstHalfColor.green);
        Logger.recordOutput("LED/FirstHalf/Blue", firstHalfColor.blue);

        Logger.recordOutput("LED/SecondHalf/Red", secondHalfColor.red);
        Logger.recordOutput("LED/SecondHalf/Green", secondHalfColor.green);
        Logger.recordOutput("LED/SecondHalf/Blue", secondHalfColor.blue);
    }
}
