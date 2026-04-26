// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.lib.Ylib.util.PIDFeedForwardGains;
import lombok.Getter;

/** Add your docs here. */
public class SwerveConstants {
  public static double ODOMETRY_FREQUENCY = 50;

  @Getter
  public static class GyroConfig {
    private String Canbus;
    private int id;
  }

  @Getter
  public static class ModuleConfig {
    private String Name;
    private String CanBus;
    private int DriveId;
    private int SteerId;
    private int EncoderID;
    private double LocationX;
    private double LocationY;
    private boolean DriveMotorInverted;
    private boolean SteerMotorInverted;
    private boolean EncoderInverted;
    private double DriveMotorGearRatio;
    private double SteerMotorGearRatio;
    private double CouplingGearRatio;
    private double WheelRadiusMeter;
    private PIDFeedForwardGains SteerMotorGains;
    private PIDFeedForwardGains DriveMotorGains;
    private double SlipCurrent;
    private double SpeedAt12Volts;
    private double EncoderOffset;
    private double SteerInertia;
    private double DriveInertia;
    private double SteerFirctionVoltage;
    private double DriveFirctionVoltage;

    public ModuleConfig(String name) {
      this.Name = name;
    }

    public ModuleConfig WithCanBus(String canBus) {
      this.CanBus = canBus;
      return this;
    }

    public ModuleConfig WithDriveId(int DriveId) {
      this.DriveId = DriveId;
      return this;
    }

    public ModuleConfig WithSteerId(int SteerId) {
      this.SteerId = SteerId;
      return this;
    }

    public ModuleConfig WithEncoderID(int EncoderID) {
      this.EncoderID = EncoderID;
      return this;
    }

    public ModuleConfig WithLocationX(double LocationX) {
      this.LocationX = LocationX;
      return this;
    }

    public ModuleConfig WithLocationY(double LocationY) {
      this.LocationY = LocationY;
      return this;
    }

    public ModuleConfig WithDriveMotorInverted(boolean DriveMotorInverted) {
      this.DriveMotorInverted = DriveMotorInverted;
      return this;
    }

    public ModuleConfig WithSteerMotorInverted(boolean SteerMotorInverted) {
      this.SteerMotorInverted = SteerMotorInverted;
      return this;
    }

    public ModuleConfig WithEncoderInverted(boolean EncoderInverted) {
      this.EncoderInverted = EncoderInverted;
      return this;
    }

    public ModuleConfig WithDriveMotorGearRatio(double DriveMotorGearRatio) {
      this.DriveMotorGearRatio = DriveMotorGearRatio;
      return this;
    }

    public ModuleConfig WithSteerMotorGearRatio(double SteerMotorGearRatio) {
      this.SteerMotorGearRatio = SteerMotorGearRatio;
      return this;
    }

    public ModuleConfig WithCouplingGearRatio(double CouplingGearRatio) {
      this.CouplingGearRatio = CouplingGearRatio;
      return this;
    }

    public ModuleConfig WithWheelRadius(double WheelRadiusMeter) {
      this.WheelRadiusMeter = WheelRadiusMeter;
      return this;
    }

    public ModuleConfig WithSteerMotorGains(PIDFeedForwardGains SteerMotorGains) {
      this.SteerMotorGains = SteerMotorGains;
      return this;
    }

    public ModuleConfig WithDriveMotorGains(PIDFeedForwardGains DriveMotorGains) {
      this.DriveMotorGains = DriveMotorGains;
      return this;
    }

    public ModuleConfig WithSteerMotorGains(Slot0Configs SteerMotorSlot0Config) {
      this.SteerMotorGains = PIDFeedForwardGains.FromSlot0Config(SteerMotorSlot0Config);
      return this;
    }

    public ModuleConfig WithDriveMotorGains(Slot0Configs DriveMotorSlot0Config) {
      this.DriveMotorGains = PIDFeedForwardGains.FromSlot0Config(DriveMotorSlot0Config);
      return this;
    }

    public ModuleConfig WithSlipCurrent(double SlipCurrent) {
      this.SlipCurrent = SlipCurrent;
      return this;
    }

    public ModuleConfig WithSpeedAt12Volts(double SpeedAt12Volts) {
      this.SpeedAt12Volts = SpeedAt12Volts;
      return this;
    }

    public ModuleConfig WithEncoderOffset(double EncoderOffset) {
      this.EncoderOffset = EncoderOffset;
      return this;
    }

    public ModuleConfig WithSteerInertia(double SteerInertia) {
      this.SteerInertia = SteerInertia;
      return this;
    }

    public ModuleConfig WithDriveInertia(double DriveInertia) {
      this.DriveInertia = DriveInertia;
      return this;
    }

    public ModuleConfig WithSteerFirctionVoltage(double SteerFirctionVoltage) {
      this.SteerFirctionVoltage = SteerFirctionVoltage;
      return this;
    }

    public ModuleConfig WithDriveFirctionVoltage(double DriveFirctionVoltage) {
      this.DriveFirctionVoltage = DriveFirctionVoltage;
      return this;
    }

    public static ModuleConfig fromTunerModuleConfig(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constants,
        String name) {
      return new ModuleConfig(name)
          .WithEncoderID(constants.EncoderId)
              .WithEncoderInverted(constants.EncoderInverted)
              .WithEncoderOffset(constants.EncoderOffset)
              .WithDriveId(constants.DriveMotorId)
              .WithDriveMotorInverted(constants.DriveMotorInverted)
              .WithDriveMotorGains(constants.DriveMotorGains)
              .WithDriveMotorGearRatio(constants.DriveMotorGearRatio)
              .WithDriveFirctionVoltage(constants.DriveFrictionVoltage)
              .WithDriveInertia(constants.DriveInertia)
              .WithSteerId(constants.SteerMotorId)
              .WithSteerMotorInverted(constants.SteerMotorInverted)
              .WithSteerMotorGains(constants.SteerMotorGains)
              .WithSteerMotorGearRatio(constants.SteerMotorGearRatio)
              .WithSteerFirctionVoltage(constants.SteerFrictionVoltage)
              .WithSteerInertia(constants.SteerInertia)
              .WithCouplingGearRatio(constants.CouplingGearRatio)
              .WithLocationX(constants.LocationX)
              .WithLocationY(constants.LocationY)
              .WithSpeedAt12Volts(constants.SpeedAt12Volts)
              .WithSlipCurrent(constants.SlipCurrent)
              .WithWheelRadius(constants.WheelRadius);
    }

    public boolean isValid() {
      return CanBus != null
          && DriveId != 0
          && SteerId != 0
          && EncoderID != 0
          && LocationX != 0
          && LocationY != 0
          && SpeedAt12Volts != 0
          && SlipCurrent != 0
          && DriveMotorGains != null
          && SteerMotorGains != null
          && WheelRadiusMeter != 0
          && DriveMotorGearRatio != 0
          && SteerMotorGearRatio != 0;
    }
  }
}
