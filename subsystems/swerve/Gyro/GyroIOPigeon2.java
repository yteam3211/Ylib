// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.subsystems.swerve.Gyro;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.Ylib.util.PhoenixOdometryThread;
import java.util.Queue;

/** Add your docs here. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<AngularVelocity> pitchVelocity;
  private final StatusSignal<AngularVelocity> rollVelocity;

  public GyroIOPigeon2(int id, String canbus, Pigeon2Configuration config) {
    pigeon = new Pigeon2(id, new CANBus(canbus));
    pigeon.getConfigurator().setYaw(0.0);
    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pitchVelocity = pigeon.getAngularVelocityYWorld();
    rollVelocity = pigeon.getAngularVelocityXWorld();
    yawVelocity.setUpdateFrequency(50);
    pitchVelocity.setUpdateFrequency(50);
    rollVelocity.setUpdateFrequency(50);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  public void UpdateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, roll, pitch, yawVelocity).equals(StatusCode.OK);
    inputs.yawPitchRollPosition =
        new Rotation3d(
            roll.getValue().in(Radian), pitch.getValue().in(Radian), yaw.getValue().in(Radian));
    inputs.yawPitchRollVelocityRadPerSec =
        new Rotation3d(
            rollVelocity.getValue().in(RadiansPerSecond),
            pitchVelocity.getValue().in(RadiansPerSecond),
            yawVelocity.getValue().in(RadiansPerSecond));
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
