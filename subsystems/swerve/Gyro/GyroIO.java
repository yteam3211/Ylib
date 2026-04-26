// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.subsystems.swerve.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.Ylib.subsystems.swerve.swerveDrive;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GyroIO {
  /** how you update the inputs shold be calld evry {@link swerveDrive#periodic()} */
  public void UpdateInputs(GyroIOInputs inputs);

  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation3d yawPitchRollVelocityRadPerSec = new Rotation3d();
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public Rotation3d yawPitchRollPosition = new Rotation3d();
  }
}
