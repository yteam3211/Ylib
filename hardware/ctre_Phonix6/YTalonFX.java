// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.hardware.ctre_Phonix6;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.Ylib.hardware.IOlayer.YMotor;
import frc.lib.Ylib.util.PIDFeedForwardGains;
import frc.lib.Ylib.util.PIDFeedForwardGains.GravityType;
import frc.lib.Ylib.util.PhoenixUtil;
import lombok.Builder;

/** Add your docs here. */
public class YTalonFX implements YMotor {
  private talonfxConfig config;
  private TalonFX m_motor;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private boolean isFollower = false;
  // private List<Pair<YTalonFX,Boolean>> followers = new ArrayList<>();
  private Map<YTalonFX,Boolean> followers = new HashMap<>();

  private Alert connectionAlarm;
  private Alert faultAlert;

  public YTalonFX(talonfxConfig config) {
    this.config = config;
    m_motor = new TalonFX(config.id, new CANBus(config.canBus));
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = config.ContinuousWrap;

    talonFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        config.closedLoopRampRate.in(Second);
    talonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod =
        config.closedLoopRampRate.in(Second);
    talonFXConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        config.closedLoopRampRate.in(Second);

    if (config.statorCurrentLimit.in(Amp) != -1) {
      talonFXConfiguration.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit.in(Amp);
      talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    }
    if (config.SupplyCurrentLimit.in(Amp) != -1) {
      talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = config.SupplyCurrentLimit.in(Amp);
      talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
    if (config.OutSideSensorID != -1 && config.motorToSensorGearRatio != -1) {
      talonFXConfiguration.Feedback.FeedbackRemoteSensorID = config.OutSideSensorID;
      talonFXConfiguration.Feedback.RotorToSensorRatio = config.motorToSensorGearRatio;
      talonFXConfiguration.Feedback.FeedbackSensorSource =
          m_motor.getIsProLicensed().getValue()
              ? FeedbackSensorSourceValue.FusedCANcoder
              : FeedbackSensorSourceValue.SyncCANcoder;
    }
    talonFXConfiguration.Feedback.SensorToMechanismRatio = config.sensorToMechnisemGearRatio;

    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        config.cruiseVelocity.in(RotationsPerSecond);
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration =
        config.MMacceleration.in(RotationsPerSecondPerSecond);
    talonFXConfiguration.MotionMagic.MotionMagicJerk = config.MMJerk;

    talonFXConfiguration.MotorOutput.Inverted =
        config.inverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    talonFXConfiguration.MotorOutput.NeutralMode =
        config.neutralBreak ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    talonFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        config.openLoopRampRate.in(Second);
    talonFXConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        config.openLoopRampRate.in(Second);
    talonFXConfiguration.OpenLoopRamps.TorqueOpenLoopRampPeriod =
        config.openLoopRampRate.in(Second) * 10;

    talonFXConfiguration.Slot0.kP = config.PIDFeedForwardGains.getK_P();
    talonFXConfiguration.Slot0.kI = config.PIDFeedForwardGains.getK_I();
    talonFXConfiguration.Slot0.kD = config.PIDFeedForwardGains.getK_D();
    talonFXConfiguration.Slot0.kV = config.PIDFeedForwardGains.getK_V();
    talonFXConfiguration.Slot0.kA = config.PIDFeedForwardGains.getK_A();
    talonFXConfiguration.Slot0.kS = config.PIDFeedForwardGains.getK_S();
    talonFXConfiguration.Slot0.kG = config.PIDFeedForwardGains.getK_G();
    talonFXConfiguration.Slot0.GravityType =
        config.PIDFeedForwardGains.getGravityType() == GravityType.Arm_Cosine
            ? GravityTypeValue.Arm_Cosine
            : GravityTypeValue.Elevator_Static;

    talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        config.ForwordSoftLimit.in(Rotation);
    talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        config.ReverseSoftLimit.in(Rotation);

    talonFXConfiguration.Voltage.PeakForwardVoltage = config.MaxVoltage;
    talonFXConfiguration.Voltage.PeakReverseVoltage = -config.MaxVoltage;

    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(talonFXConfiguration));
    connectionAlarm = new Alert("connection problems with " + config.name, AlertType.kWarning);
    faultAlert = new Alert("fault with " + config.name, AlertType.kError);
  }

  @Override
  public void setDutyCycle(Double dutyCycle) {
    m_motor.set(dutyCycle);
    followers.forEach((yTalonFX,opposed)->{
      yTalonFX.followMotor(this, opposed);
    });
  }

  @Override
  public void setVoltage(Voltage Voltage) {
    m_motor.setVoltage(Voltage.in(Volts));
    followers.forEach((yTalonFX,opposed)->{
      yTalonFX.followMotor(this, opposed);
    });
  }

  @Override
  public void setPosition(Angle position) {
    m_motor.setControl(motionMagicVoltage.withPosition(position).withSlot(0));
    followers.forEach((yTalonFX,opposed)->{
      yTalonFX.followMotor(this, opposed);
    });
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    m_motor.setControl(velocityVoltage.withVelocity(velocity).withSlot(0));
    followers.forEach((yTalonFX,opposed)->{
      yTalonFX.followMotor(this, opposed);
    });
  }

  @Override
  public void stop() {
    m_motor.stopMotor();
    followers.forEach((yTalonFX,opposed)->{
      yTalonFX.followMotor(this, opposed);
    });
  }

  @Override
  public void setEncoderPosition(Angle position) {
    m_motor.setPosition(position);
  }

  @Override
  public void zeroEncoder() {
    m_motor.setPosition(0);
  }

  @Override
  public Angle getPosition() {
    return m_motor.getPosition().getValue();
  }

  @Override
  public AngularVelocity getvelocity() {
    return m_motor.getVelocity().getValue();
  }

  @Override
  public Voltage getApllidVoltage() {
    return m_motor.getMotorVoltage().getValue();
  }

  @Override
  public Current getStatorCurrent() {
    return m_motor.getStatorCurrent().getValue();
  }

  @Override
  public Current getSupplyCurrent() {
    return m_motor.getSupplyCurrent().getValue();
  }

  @Override
  public Temperature getTemperature() {
    return m_motor.getDeviceTemp().getValue();
  }

  @Override
  public void updateNetworkTables() {
    if (isFollower) {
      return;
    }
    String key = m_motor.getDeviceID() + "on" + m_motor.getNetwork().getName();
    Logger.recordOutput(
        key + "/position", getPosition());
    Logger.recordOutput(
        key + "/velocity", getvelocity());
    Logger.recordOutput(
        key + "/voltage",
        getApllidVoltage());
    Logger.recordOutput(
        key + "/temperature",
        getTemperature());
    Logger.recordOutput(
        key + "/statorCurrent",
        getStatorCurrent());
    Logger.recordOutput(
        key + "/supplyCurrent",
        getSupplyCurrent());
    Logger.recordOutput(
        key + "/controlMode",
        m_motor.getControlMode().getValue());
  }
  

  @Override
  public void AlertUpdate() {
    connectionAlarm.set(!m_motor.isConnected());
    faultAlert.set(m_motor.getFault_Hardware().getValue() || 
    m_motor.getFault_RotorFault1().getValue() || m_motor.getFault_RotorFault2().getValue() || 
    m_motor.getFault_UnstableSupplyV().getValue() || m_motor.getFault_Undervoltage().getValue());
  }

  @Override
  public void addFollower(int id, boolean Opposed) {
    followers.put(new YTalonFX(config.toBuilder().id(id).build()), Opposed);
    this.isFollower = false;
  }

  @Override
  public void followMotor(YMotor master,boolean opposed) {
    if (master instanceof YTalonFX) {
      this.isFollower = true;
      m_motor.setControl(new Follower(((YTalonFX)master).m_motor.getDeviceID(), opposed? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }
    throw new RuntimeException("not Ytalonfx so cant make a follower");
  }

  @Builder(toBuilder = true)
  public static class talonfxConfig {
    /** The CAN ID of the motor. */
    public int id;
    /** The name of the CAN bus (e.g., "rio" or "canivore"). */
    public String canBus;
    /** the name of the motor (e.g., "intakePitch" or "ElevatorMaster") */
    public String name;
    /** Whether the motor is inverted (true for CCW, false for CW). */
    @Builder.Default public boolean inverted = false;
    /** Whether the motor is in Brake mode (true) or Coast mode (false) when idle. */
    @Builder.Default public boolean neutralBreak = true;
    /** PID and Feedforward gains for the motor controller. */
    @Builder.Default public PIDFeedForwardGains PIDFeedForwardGains = new PIDFeedForwardGains();
    /**
     * * The ratio from the sensor to the final mechanism movement. If using an internal sensor,
     * this is usually the gearbox ratio.
     */
    public double sensorToMechnisemGearRatio;
    /** The CAN ID of an external sensor (e.g., CANcoder). Set to -1 if none. */
    @Builder.Default public int OutSideSensorID = -1;
    /** The gear ratio between the motor and the external sensor. */
    @Builder.Default public double motorToSensorGearRatio = -1;
    /** The maximum velocity allowed for Motion Magic or profiled control. */
    @Builder.Default public AngularVelocity cruiseVelocity = RotationsPerSecond.of(-1);

    @Builder.Default public AngularAcceleration MMacceleration = RotationsPerSecondPerSecond.of(-1);
    @Builder.Default public double MMJerk = -1;
    /** Software forward limit; should be set slightly before the mechanical hard stop. */
    @Builder.Default public Angle ForwordSoftLimit = Rotation.of(Double.POSITIVE_INFINITY);
    /** Software reverse limit; should be set slightly before the mechanical hard stop. */
    @Builder.Default public Angle ReverseSoftLimit = Rotation.of(Double.NEGATIVE_INFINITY);
    /** The maximum voltage the motor is permitted to output. */
    @Builder.Default public double MaxVoltage = 12;
    /** The time (in seconds) to ramp from 0 to full throttle in Open Loop control. */
    @Builder.Default public Time openLoopRampRate = Second.of(-1);
    /** The time (in seconds) to ramp from 0 to full throttle in Closed Loop control. */
    @Builder.Default public Time closedLoopRampRate = Second.of(-1);

    @Builder.Default public boolean ContinuousWrap = false;
    @Builder.Default public Current statorCurrentLimit = Amp.of(-1);
    @Builder.Default public Current SupplyCurrentLimit = Amp.of(-1);
    /**
     * this is the off set of the motor before gear ration and evrything (use this on only gear that
     * the mechanisem dose no more then one rotation of the motor e.g. an arm that is geard 2:1 and
     * move only 90 degree or anything that is geard 1 to 1 and need position) if your motor do more
     * then 1
     */
    @Builder.Default public Angle MotorOffset = Rotation.of(0);
    // @Builder.Default public
    public talonfxConfig cloneConfig() {
      talonfxConfig clone = this.toBuilder().build();
      return clone;
    }
  }
}
