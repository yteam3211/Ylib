// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.hardware.IOlayer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface YMotor {
  /**
   * Sets the motor output as a percentage of available voltage [-1, 1].
   *
   * @apiNote This method is not recommended for precision control, as output will fluctuate based
   *     on battery voltage. Use {@link #setVoltage(Voltage)} instead.
   */
  public void setDutyCycle(Double dutyCycle);
  /** Sets the motor output voltage [-12V, 12V]. */
  public void setVoltage(Voltage Voltage);
  /** Sets the closed-loop position target using the configured PID gains. */
  public void setPosition(Angle position);
  /** Sets the closed-loop velocity target for the mechanism. */
  public void setVelocity(AngularVelocity velocity);
  /** Stops the motor (neutral output). */
  public void stop();
  /** Overrides the motor's current encoder position. */
  public void setEncoderPosition(Angle position);
  /** Resets the motor's encoder to zero. */
  public void zeroEncoder();
  /** Returns the current position of the mechanism. */
  public Angle getPosition();
  /** Returns the current velocity of the mechanism. */
  public AngularVelocity getvelocity();
  /** Returns the actual voltage applied to the motor. */
  public Voltage getApllidVoltage();
  /** Returns the stator current draw (current through the motor coils). */
  public Current getStatorCurrent();
  /** Returns the supply current (current drawn from the PDP/PDH). */
  public Current getSupplyCurrent();
  /** Returns the internal temperature of the motor controller. */
  public Temperature getTemperature();
  /** Returns the simulation of the motor use for chaning the motor */
  /**
   * Updates telemetry data. Integration with AdvantageKit is recommended: {@link
   * https://docs.advantagekit.org/}
   */
  public void updateNetworkTables();
  /** Should be called every periodic cycle to handle motor faults and connectivity issues. */
  public void AlertUpdate();
  /**
   * Configures a motor to follow this one via the CAN bus.
   *
   * @param id The CAN ID of the follower motor.
   * @param opposed Set to true if the follower should move in the opposite direction.
   */
  public void addFollower(int id, boolean Opposed);
  /** Sets this motor to follow a master motor. */
  public void followMotor(YMotor master,boolean opposed);
}
