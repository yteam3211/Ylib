// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.subsystems.swerve.Modules;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.Ylib.subsystems.swerve.SwerveConstants;

/** Add your docs here. */
public interface ModuleIO {
   /** update the inputs of this Moudle*/
   public void UpdateInputs(ModuleInputs inputs);
   /**
    * @return the displacment of the moudle
    */
   public Translation2d getTranslation2d();
   /** run the State thet you give him this is how you drive the moudle and probley the most importent thing to do right */
   public void runState(SwerveModuleState state);
   /** this is the drive Open loop given a Voltage it Run it in*/
   public void setDriveOpenLoop(double Votlage);
   /** this is the Drive closed loop velocity control with voltage 
    * @param VelocityRotPerSec needed to be in rotation per sec
   */
   public void setDriveVelocity(double VelocityRotPerSec);
   /** this is the drive closed loop velocity control with voltage */
   public void setDriveVelocity(AngularVelocity Velocity);
   /** this is the Turn (steer) open loop run at Voltage */
   public void setTurnOpenLoop(double Votlage);
   /** this is the Turn (steer) closed loop control with voltage */
   public void setTurnPosition(Rotation2d rotation);
   /** @return the config of the module */
   public SwerveConstants.ModuleConfig getConfig();
   @AutoLog
   public static class ModuleInputs {
      public boolean driveConnected = false;
      public double drivePositionRot = 0.0;
      public double driveVelocityRotPerSec = 0.0;
      public double driveAppliedVolts = 0.0;
      public double driveCurrentAmps = 0.0;

      public boolean SteerConnected = false;
      public boolean SteerEncoderConnected = false;
      public Rotation2d SteerAbsolutePosition = Rotation2d.kZero;
      public Rotation2d SteerPosition = Rotation2d.kZero;
      public double SteerVelocityRotPerSec = 0.0;
      public double SteerAppliedVolts = 0.0;
      public double SteerCurrentAmps = 0.0;

      public double[] odometryTimestamps = new double[] {};
      public double[] odometryDrivePositionsRot = new double[] {};
      public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
   }
}
