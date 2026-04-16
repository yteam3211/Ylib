// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.Hardwere;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.Ylib.util.PIDFeedForwordGains;
import frc.lib.Ylib.util.PIDFeedForwordGains.GravityType;
import lombok.Getter;

/**
 * Interface representing a hardware abstraction layer for a motor controller.
 * Provides a standardized set of commands and telemetry for various motor types.
 */
public interface MotorIO {

    /** * Directly sets the output voltage of the motor.
     * @param voltage The voltage to apply (typically -12 to 12).
     */
    public void RunVoltage(double voltage);

    /** * Commands the motor to move to a specific angular position using internal PID control.
     * @param position The target angle for the mechanism.
     */
    public void GoToPosition(Angle position);

    /** * Commands the motor to maintain a specific angular velocity using internal PID control.
     * @param velocity The target velocity for the mechanism.
     */
    public void GoToVelocity(AngularVelocity velocity);

    /** * Stops the motor by setting output to zero.
     */
    public void stop();

    /** * Overrides the current encoder position to a specific value.
     * @param position The new angle to set as the current reference.
     */
    public void setPosition(Angle position);

    /** * Configures another motor to follow this motor's movements.
     * <b>Note:</b> The follower motor must be the same hardware type as the original motor 
     * (e.g., a TalonFX can only follow another TalonFX).
     * @param motor The motor IO instance that will act as the follower.
     */
    public void setFolower(MotorIO motor);

    /** * Returns the maximum possible velocity of the motor hardware.
     * @return AngularVelocity representing the hardware limit.
     */
    public static AngularVelocity getMaxVelocity() { return RPM.of(0); }

    /** * Gets the current angular velocity of the mechanism.
     * @return The measured velocity.
     */
    public AngularVelocity getVelocity();

    /** * Gets the current angular position of the mechanism.
     * @return The measured position.
     */
    public Angle getPostion();

    /** * Gets the current angular acceleration of the mechanism.
     * @return The calculated or measured acceleration.
     */
    public AngularAcceleration getAcceleration();

    /** * Gets the current voltage being applied to the motor.
     * @return The applied voltage.
     */
    public Voltage getVoltage();

    /** * Gets the current being drawn from the battery by the motor controller.
     * @return The supply current.
     */
    public Current getSupllayCurrent();

    /** * Gets the current flowing through the motor windings.
     * @return The stator current.
     */
    public Current getStatorCurrent();

    /**
     * Configuration class containing constants and settings for initializing a motor.
     */
    @Getter
    public class MotorConfig extends hardwereConfig {
        /** The gear ratio between the motor rotor and the integrated sensor. */
        public double rotorToSensorGearRatio = 0;

        /** The gear ratio between the sensor and the final mechanism output. */
        public double SensorToMechanismGearRatio = 0;

        /** Primary PID and Feed-Forward gains for the first control slot. */
        public PIDFeedForwordGains gainsSlot0 = new PIDFeedForwordGains(0, 0, 0, 0, 0, GravityType.Arm_Cosine);

        /** Secondary PID and Feed-Forward gains for the second control slot. */
        public PIDFeedForwordGains gainsSlot1 = new PIDFeedForwordGains(0, 0, 0, 0, 0, GravityType.Arm_Cosine);

        /** Tertiary PID and Feed-Forward gains for the third control slot. */
        public PIDFeedForwordGains gainsSlot2 = new PIDFeedForwordGains(0, 0, 0, 0, 0, GravityType.Arm_Cosine);

        /** The maximum velocity allowed during trapezoidal motion profiling. */
        public AngularVelocity maxVleocitytrapezoidal = RPM.of(0);

        /** The maximum acceleration allowed during trapezoidal motion profiling. */
        public AngularAcceleration maxAccelerationtrapezoidal = RotationsPerSecondPerSecond.of(0);

        /** Whether soft limits (max/min movement) should be enabled. */
        public boolean haveMaxMovement = false;

        /** The maximum forward travel limit (soft limit). */
        public double maxMovementForword = 0;

        /** The maximum reverse travel limit (soft limit). */
        public double maxMovementReverse = 0;

        /** Whether the motor direction should be inverted. */
        public boolean inverted = false;

        /** If true, the motor will use Brake mode; if false, it will use Coast mode. */
        public boolean isBreak = true;

        /** Whether an external sensor (e.g., CANCoder) is used for feedback. */
        public boolean haveOutSideSensor = false;
        /** the outside sensor the you get the data from */
        public SensorIO OutSideSensor = null;
    }
}