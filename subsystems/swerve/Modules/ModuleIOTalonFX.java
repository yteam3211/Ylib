// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.subsystems.swerve.Modules;

import java.util.Queue;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.Ylib.subsystems.swerve.SwerveConstants;
import frc.lib.Ylib.subsystems.swerve.SwerveConstants.ModuleConfig;
import frc.lib.Ylib.util.PhoenixOdometryThread;
import frc.lib.Ylib.util.PhoenixUtil;
import lombok.val;

/** Add your docs here. */
public class ModuleIOTalonFX implements ModuleIO{
    /** use for savong the config and taking data when needed */
    private final SwerveConstants.ModuleConfig config;
    private final TalonFX SteerMotor;
    private final TalonFX DriveMotor;
    private final CANcoder CanCoder;
    private DoubleConsumer positionRequset;
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final Debouncer driveConncetedDebouncer = new Debouncer(0.5,DebounceType.kFalling);
    private final Debouncer steerConncetedDebouncer = new Debouncer(0.5,DebounceType.kFalling);
    private final Debouncer encoderConncetedDebouncer = new Debouncer(0.5,DebounceType.kFalling);
    private Queue<Double> timestampQueues;
    private Queue<Double> DrivePositionQueue;
    private Queue<Double> SteerPositionQueue;
    public ModuleIOTalonFX(SwerveConstants.ModuleConfig config){
        if (config.isValid()) {
            throw new IllegalArgumentException("the module " + config.getName() + "Config is not Valid and will not work check if all the constantst are right");
        }
        this.config = config;
        CANBus canBus = new CANBus(config.getCanBus());
        /** config of the Drive motor
         * start with break mode for the drive motor 
         * then inverted
         * then gains
         * then SlipCurrent if you want to be safe and dont have the time to find itput it at 50
         */
        DriveMotor = new TalonFX(config.getDriveId(),canBus);
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfiguration.MotorOutput.Inverted = config.isDriveMotorInverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        driveConfiguration.Slot0.kP = config.getDriveMotorGains().getK_p();
        driveConfiguration.Slot0.kI = config.getDriveMotorGains().getK_i();
        driveConfiguration.Slot0.kD = config.getDriveMotorGains().getK_d();
        driveConfiguration.Slot0.kG = config.getDriveMotorGains().getK_g();
        driveConfiguration.Slot0.kS = config.getDriveMotorGains().getK_s();
        driveConfiguration.Slot0.kV = config.getDriveMotorGains().getK_v();
        driveConfiguration.Slot0.kA = config.getDriveMotorGains().getK_a();
        driveConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = config.getSlipCurrent();
        driveConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -config.getSlipCurrent();
        driveConfiguration.CurrentLimits.StatorCurrentLimit = config.getSlipCurrent();
        driveConfiguration.CurrentLimits.StatorCurrentLimitEnable = true; 
        driveConfiguration.Feedback.SensorToMechanismRatio = config.getDriveMotorGearRatio();
        PhoenixUtil.tryUntilOk(5, ()-> DriveMotor.getConfigurator().apply(driveConfiguration));
        PhoenixUtil.tryUntilOk(5, ()-> DriveMotor.setPosition(0.0,0.25));

        /** config of the Steer motor */
        SteerMotor = new TalonFX(config.getSteerId(), canBus);
        TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();
        steerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfiguration.MotorOutput.Inverted = config.isSteerMotorInverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        steerConfiguration.Slot0.kP = config.getSteerMotorGains().getK_p();
        steerConfiguration.Slot0.kI = config.getSteerMotorGains().getK_i();
        steerConfiguration.Slot0.kD = config.getSteerMotorGains().getK_d();
        steerConfiguration.Slot0.kG = config.getSteerMotorGains().getK_g();
        steerConfiguration.Slot0.kS = config.getSteerMotorGains().getK_s();
        steerConfiguration.Slot0.kV = config.getSteerMotorGains().getK_v();
        steerConfiguration.Slot0.kA = config.getSteerMotorGains().getK_a();
        steerConfiguration.Feedback.FeedbackRemoteSensorID = config.getEncoderID();
        steerConfiguration.Feedback.FeedbackSensorSource = SteerMotor.getIsProLicensed().getValue() ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfiguration.Feedback.RotorToSensorRatio = config.getSteerMotorGearRatio();
        steerConfiguration.MotionMagic.MotionMagicCruiseVelocity  = 100.0 / config.getSteerMotorGearRatio();
        steerConfiguration.MotionMagic.MotionMagicAcceleration = steerConfiguration.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        positionRequset = (Rotation)-> SteerMotor.setControl(new MotionMagicVoltage(Rotation));
        if (SteerMotor.getIsProLicensed().getValue()) {
            steerConfiguration.MotionMagic.MotionMagicExpo_kV = 0.12 * config.getSteerMotorGearRatio();
            steerConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;
            positionRequset = (Rotation)-> SteerMotor.setControl(new MotionMagicExpoVoltage(Rotation));
        }
        steerConfiguration.ClosedLoopGeneral.ContinuousWrap = true; 
        CanCoder = new CANcoder(config.getEncoderID(), canBus);
        CANcoderConfiguration caNcoderConfiguration = new CANcoderConfiguration();
        caNcoderConfiguration.MagnetSensor.SensorDirection = config.isEncoderInverted() ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        caNcoderConfiguration.MagnetSensor.MagnetOffset = config.getEncoderOffset();
        CanCoder.getConfigurator().apply(caNcoderConfiguration);
        timestampQueues = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        DrivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(DriveMotor.getPosition());
        SteerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(SteerMotor.getPosition());
    }


    @Override
    public Translation2d getTranslation2d() {
        return new Translation2d(config.getLocationX(), config.getLocationY());
    }

    @Override
    public void runState(SwerveModuleState state) {
        state.optimize(Rotation2d.fromRotations(SteerMotor.getPosition().getValueAsDouble()));
        setTurnPosition(state.angle);
        setDriveVelocity(state.speedMetersPerSecond / config.getWheelRadiusMeter());
    }

    @Override
    public void setDriveOpenLoop(double Votlage) {
        DriveMotor.setVoltage(Votlage);
    }

    @Override
    public void setDriveVelocity(double VelocityRotPerSec) {
        DriveMotor.setControl(velocityVoltage.withVelocity(VelocityRotPerSec));
    }

    @Override
    public void setDriveVelocity(AngularVelocity Velocity) {
        DriveMotor.setControl(velocityVoltage.withVelocity(Velocity));
    }

    @Override
    public void setTurnOpenLoop(double Votlage) {
        SteerMotor.setVoltage(Votlage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        positionRequset.accept(rotation.getRotations());
    }

    @Override
    public void UpdateInputs(ModuleInputs inputs) {
        /** all the drive data */
        inputs.driveConnected = driveConncetedDebouncer.calculate(DriveMotor.isConnected());
        inputs.driveCurrentAmps = DriveMotor.getSupplyCurrent().getValueAsDouble();
        inputs.driveAppliedVolts = DriveMotor.getMotorVoltage().getValueAsDouble();
        inputs.drivePositionRot = DriveMotor.getPosition().getValueAsDouble();
        inputs.driveVelocityRotPerSec = DriveMotor.getVelocity().getValueAsDouble();
        /** all the Steer data */
        inputs.SteerConnected = steerConncetedDebouncer.calculate(SteerMotor.isConnected());
        inputs.SteerCurrentAmps = SteerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.SteerAppliedVolts = SteerMotor.getMotorVoltage().getValueAsDouble();
        inputs.SteerPosition = Rotation2d.fromRotations(SteerMotor.getPosition().getValueAsDouble());
        inputs.SteerEncoderConnected = encoderConncetedDebouncer.calculate(CanCoder.isConnected());
        inputs.SteerAbsolutePosition = Rotation2d.fromRotations(CanCoder.getAbsolutePosition().getValueAsDouble());
        inputs.SteerVelocityRotPerSec = SteerMotor.getVelocity().getValueAsDouble();
        /** odometry data */
        inputs.odometryTimestamps = timestampQueues.stream().mapToDouble((Double val)-> val).toArray();
        inputs.odometryDrivePositionsRot = DrivePositionQueue.stream().mapToDouble((Double val)-> val).toArray();
        inputs.odometrySteerPositions = SteerPositionQueue.stream().map((Double val)-> Rotation2d.fromRotations(val)).toArray(Rotation2d[]::new);
        timestampQueues.clear();
        DrivePositionQueue.clear();
        SteerPositionQueue.clear();
    }


    @Override
    public SwerveConstants.ModuleConfig getConfig() {
        return config;
    }
}
