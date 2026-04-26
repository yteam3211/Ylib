// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.util;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularVelocityUnit;
import lombok.Getter;
import lombok.Setter;

/** Add your docs here. */
@Getter
@Setter
public class PIDFeedForwardGains {
  public enum GravityType {
    Arm_Cosine,
    ElevatorGravity;
  }

  private double K_P = 0;
  private double K_I = 0;
  private double K_D = 0;
  private double K_G = 0;
  private double K_S = 0;
  private double K_V = 0;
  private double K_A = 0;
  private GravityType gravityType = GravityType.Arm_Cosine;

  public PIDFeedForwardGains(
      double K_p,
      double K_i,
      double K_d,
      double K_g,
      double K_s,
      double K_v,
      double K_a,
      GravityType gravityType) {
    this.K_P = K_p;
    this.K_I = K_i;
    this.K_D = K_d;
    this.K_G = K_g;
    this.K_S = K_s;
    this.K_V = K_v;
    this.K_A = K_a;
    this.gravityType = gravityType;
  }

  public PIDFeedForwardGains(
      double K_p, double K_i, double K_d, double K_g, double K_s, GravityType gravityType) {
    this.K_P = K_p;
    this.K_I = K_i;
    this.K_D = K_d;
    this.K_G = K_g;
    this.K_S = K_s;
    this.gravityType = gravityType;
  }

  public PIDFeedForwardGains(double K_p, double K_i, double K_d, double K_v, double K_a) {
    this.K_P = K_p;
    this.K_I = K_i;
    this.K_D = K_d;
    this.K_V = K_v;
    this.K_A = K_a;
  }

  public PIDFeedForwardGains() {}
  ;

  public PIDFeedForwardGains SetKVFromGearAndMotor(
      DCMotor motor, double gearratio, AngularVelocityUnit unit) {
    K_V = 1 / (RadiansPerSecond.of(motor.KvRadPerSecPerVolt).in(unit) * gearratio);
    return this;
  }

  public static PIDFeedForwardGains FromSlot0Config(Slot0Configs slotConfigs) {

    return new PIDFeedForwardGains(
        slotConfigs.kP,
        slotConfigs.kI,
        slotConfigs.kD,
        slotConfigs.kG,
        slotConfigs.kS,
        slotConfigs.kV,
        slotConfigs.kA,
        slotConfigs.GravityType == GravityTypeValue.Elevator_Static
            ? GravityType.ElevatorGravity
            : GravityType.Arm_Cosine);
  }
}
