// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.CANdleSystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSystem extends SubsystemBase {
  private final CANdle m_candle = new CANdle(22, new CANBus("rio"));
  private final int LedCount = 300;
  private XboxController joystick;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  private AnimationTypes m_currentAnimation;

  public CANdleSystem(XboxController joy) {
    this.joystick = joy;
    changeAnimation(AnimationTypes.SetAll);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    configAll.LED.StripType = StripTypeValue.GRB;
    m_candle.getConfigurator().apply(configAll);
  }

  public void incrementAnimation() {
    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Fire:
        changeAnimation(AnimationTypes.Larson);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
      case SetAll:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
    }
  }

  public void decrementAnimation() {
    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case Fire:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.Larson);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case SetAll:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
    }
  }

  public void setColors() {
    changeAnimation(AnimationTypes.SetAll);
  }

  public void changeAnimation(AnimationTypes toChange) {
    m_currentAnimation = toChange;

    // switch(toChange)
    // {
    //     case ColorFlow:
    //         m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount,
    // Direction.Forward);
    //         break;
    //     case Fire:
    //         m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
    //         break;
    //     case Larson:
    //         m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
    //         break;
    //     case Rainbow:
    //         m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
    //         break;
    //     case RgbFade:
    //         m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
    //         break;
    //     case SingleFade:
    //         m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
    //         break;
    //     case Strobe:
    //         m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
    //         break;
    //     case Twinkle:
    //         m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount,
    // TwinklePercent.Percent6);
    //         break;
    //     case TwinkleOff:
    //         m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount,
    // TwinkleOffPercent.Percent100);
    //         break;
    //     case SetAll:
    //         m_toAnimate = null;
    //         break;
    // }
    // System.out.println("Changed to " + m_currentAnimation.toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SolidColor request = new SolidColor(0, 67);
    // request.withColor(new RGBWColor(0, 255, 0,0));

    m_candle.setControl(new SolidColor(0, 67).withColor(new RGBWColor(0, 217, 0, 0)));

    // if(m_toAnimate == null) {
    //     m_candle.setLEDs((int)(joystick.getLeftTriggerAxis() * 255),
    //                       (int)(joystick.getRightTriggerAxis() * 255),
    //                       (int)(joystick.getLeftX() * 255));
    // } else {
    //     m_candle.animate(m_toAnimate);
    // }
    // m_candle.modulateVBatOutput(joystick.getRightY());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
