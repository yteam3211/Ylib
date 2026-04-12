// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.util;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularVelocityUnit;
import lombok.Getter;
import lombok.Setter;

/** Add your docs here. */
@Getter
@Setter
public class PIDFeedForwordGains {
    public enum GravityType{
        Arm_Cosine,
        ElevatorGravity;
    }
    private double K_p = 0;
    private double K_i = 0;
    private double K_d = 0;
    private double K_g = 0;
    private double K_s = 0;
    private double K_v = 0;
    private double K_a = 0;
    private GravityType gravityType = GravityType.Arm_Cosine;
    public PIDFeedForwordGains(double K_p , double K_i , double K_d , double K_g , double K_s , double K_v , double K_a,GravityType gravityType){
        this.K_p = K_p;
        this.K_i = K_i;
        this.K_d = K_d;
        this.K_g = K_g;
        this.K_s = K_s;
        this.K_v = K_v;
        this.K_a = K_a;
    }
    public PIDFeedForwordGains(double K_p , double K_i , double K_d , double K_g , double K_s ,GravityType gravityType){
        this.K_p = K_p;
        this.K_i = K_i;
        this.K_d = K_d;
        this.K_g = K_g;
        this.K_s = K_s;
    }
    public PIDFeedForwordGains(double K_p , double K_i , double K_d , double K_v , double K_a){
        this.K_p = K_p;
        this.K_i = K_i;
        this.K_d = K_d;
        this.K_v = K_v;
        this.K_a = K_a;
    }
    public PIDFeedForwordGains SetKVFromGearAndMotor(DCMotor motor,double gearratio,AngularVelocityUnit unit){
        K_v = 1/(RadiansPerSecond.of(motor.KvRadPerSecPerVolt).in(unit) * gearratio);
        return this;
    }
    public static PIDFeedForwordGains FromSlot0Config(Slot0Configs slotConfigs){
        return new PIDFeedForwordGains(slotConfigs.kP, slotConfigs.kI, slotConfigs.kD, slotConfigs.kG, 
            slotConfigs.kS,slotConfigs.kV,slotConfigs.kA,slotConfigs.GravityType == GravityTypeValue.Elevator_Static ? GravityType.ElevatorGravity : GravityType.Arm_Cosine);
    }
}