// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.subsystems.swerve.Modules;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Queue;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
import frc.lib.Ylib.util.StateSpaceController;

/** 
 * this is a more advence way to do swerve.
 * instade of using pid and feedForwrod it use LQR and stateSpaceController
 * to use this you need to know evrything about the subsystem like MOI size of whell size and evrything
 * you need to find the kv and ka of the swerve via sysid and get it via radians and not something else like rotation or degrees
 * you need to do both the velocity and the postion
*/
public class ModuleIOTalonFXAdvence implements ModuleIO{
    /** Hardware and Controller References */
    private final SwerveConstants.ModuleConfig config;
    private final TalonFX SteerMotor;
    private final TalonFX DriveMotor;
    private final CANcoder CanCoder;
    private final StateSpaceController.Position positionController;
    private final StateSpaceController.Velocity velocityController;

    // Status debouncers for connection monitoring
    private final Debouncer driveConncetedDebouncer = new Debouncer(0.5,DebounceType.kFalling);
    private final Debouncer steerConncetedDebouncer = new Debouncer(0.5,DebounceType.kFalling);
    
    // Queues for high-frequency odometry data
    private final Debouncer encoderConncetedDebouncer = new Debouncer(0.5,DebounceType.kFalling);
    private Queue<Double> timestampQueues;
    private Queue<Double> DrivePositionQueue;
    private Queue<Double> SteerPositionQueue;

    // Boolean for Open/Closed Loop
    private boolean VelocityclosedLoop = false;
    private boolean VelocityOpenLoop = true;
    private boolean PositionclosedLoop = false;
    private boolean PositionOpenLoop = true;
    public ModuleIOTalonFXAdvence(SwerveConstants.ModuleConfig config,StateSpaceController.Position positionController, StateSpaceController.Velocity velocityController) {
        if (config.isValid()) {
            throw new IllegalArgumentException("Invalid configuration for module: " + config.getName());
        }
        this.config = config;
        CANBus canBus = new CANBus(config.getCanBus());

        // --- Drive Motor Setup ---
        DriveMotor = new TalonFX(config.getDriveId(), canBus);
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        
        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfiguration.MotorOutput.Inverted = config.isDriveMotorInverted() ? 
            InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        
        // Use SlipCurrent to prevent traction loss/motor burnout
        driveConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = config.getSlipCurrent();
        driveConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -config.getSlipCurrent();
        driveConfiguration.CurrentLimits.StatorCurrentLimit = config.getSlipCurrent();
        driveConfiguration.CurrentLimits.StatorCurrentLimitEnable = true; 
        
        driveConfiguration.Feedback.SensorToMechanismRatio = config.getDriveMotorGearRatio();
        
        PhoenixUtil.tryUntilOk(5, () -> DriveMotor.getConfigurator().apply(driveConfiguration));
        PhoenixUtil.tryUntilOk(5, () -> DriveMotor.setPosition(0.0, 0.25));

        // --- Steer Motor Setup ---
        SteerMotor = new TalonFX(config.getSteerId(), canBus);
        TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();
        
        steerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfiguration.MotorOutput.Inverted = config.isSteerMotorInverted() ? 
            InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        
        // Remote CANcoder integration
        steerConfiguration.Feedback.FeedbackRemoteSensorID = config.getEncoderID();
        steerConfiguration.Feedback.FeedbackSensorSource = SteerMotor.getIsProLicensed().getValue() ? 
            FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RemoteCANcoder;
        
        steerConfiguration.Feedback.RotorToSensorRatio = config.getSteerMotorGearRatio();
        steerConfiguration.ClosedLoopGeneral.ContinuousWrap = true; 

        // --- CANcoder Setup ---
        CanCoder = new CANcoder(config.getEncoderID(), canBus);
        CANcoderConfiguration caNcoderConfiguration = new CANcoderConfiguration();
        caNcoderConfiguration.MagnetSensor.SensorDirection = config.isEncoderInverted() ? 
            SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        caNcoderConfiguration.MagnetSensor.MagnetOffset = config.getEncoderOffset();
        CanCoder.getConfigurator().apply(caNcoderConfiguration);

        // --- Odometry & Controllers ---
        PhoenixOdometryThread phoenixOdometryThread = new PhoenixOdometryThread("Module/" + config.getName());
        timestampQueues = phoenixOdometryThread.makeTimestampQueue();
        DrivePositionQueue = phoenixOdometryThread.registerSignal(DriveMotor.getPosition());
        SteerPositionQueue = phoenixOdometryThread.registerSignal(SteerMotor.getPosition());

        this.positionController = positionController;
        this.velocityController = velocityController;
        positionController.reset(SteerMotor.getPosition().getValueAsDouble(), SteerMotor.getVelocity().getValueAsDouble());
        velocityController.reset(DriveMotor.getVelocity().getValueAsDouble());
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
        VelocityOpenLoop = true;
        VelocityclosedLoop = false;
        DriveMotor.setVoltage(Votlage);
    }

    @Override
    public void setDriveVelocity(double VelocityRotPerSec) {
        VelocityOpenLoop = false;
        VelocityclosedLoop = true;
        velocityController.setReference(VelocityRotPerSec);
    }

    @Override
    public void setDriveVelocity(AngularVelocity Velocity) {
        setDriveVelocity(Velocity.in(RotationsPerSecond));
    }

    @Override
    public void setTurnOpenLoop(double Votlage) {
        PositionOpenLoop = true;
        PositionclosedLoop = false;
        SteerMotor.setVoltage(Votlage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        PositionOpenLoop = false;
        PositionclosedLoop = true;
        positionController.setReference(rotation.getRotations(), 0);
    }

    @Override
    public void UpdateInputs(ModuleInputs inputs) {
        runControllers();
        // Drive motor status
        inputs.driveConnected = driveConncetedDebouncer.calculate(DriveMotor.isConnected());
        inputs.driveCurrentAmps = DriveMotor.getSupplyCurrent().getValueAsDouble();
        inputs.driveAppliedVolts = DriveMotor.getMotorVoltage().getValueAsDouble();
        inputs.drivePositionRot = DriveMotor.getPosition().getValueAsDouble();
        inputs.driveVelocityRotPerSec = DriveMotor.getVelocity().getValueAsDouble();

        // Steer motor and encoder status
        inputs.SteerConnected = steerConncetedDebouncer.calculate(SteerMotor.isConnected());
        inputs.SteerCurrentAmps = SteerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.SteerAppliedVolts = SteerMotor.getMotorVoltage().getValueAsDouble();
        inputs.SteerPosition = Rotation2d.fromRotations(SteerMotor.getPosition().getValueAsDouble());
        inputs.SteerEncoderConnected = encoderConncetedDebouncer.calculate(CanCoder.isConnected());
        inputs.SteerAbsolutePosition = Rotation2d.fromRotations(CanCoder.getAbsolutePosition().getValueAsDouble());
        inputs.SteerVelocityRotPerSec = SteerMotor.getVelocity().getValueAsDouble();

        // Process high-frequency odometry data
        inputs.odometryTimestamps = timestampQueues.stream().mapToDouble(val -> val).toArray();
        inputs.odometryDrivePositionsRot = DrivePositionQueue.stream().mapToDouble(val -> val).toArray();
        inputs.odometrySteerPositions = SteerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        
        // Clear queues for the next loop
        timestampQueues.clear();
        DrivePositionQueue.clear();
        SteerPositionQueue.clear();
    }


    private void runControllers() {
        if (PositionclosedLoop) {
            positionController.correct(SteerMotor.getPosition().getValueAsDouble(), SteerMotor.getVelocity().getValueAsDouble());
            positionController.predict(0.02);
            SteerMotor.setVoltage(positionController.getVoltage());
        }
        if (VelocityclosedLoop) {
            velocityController.correct(DriveMotor.getVelocity().getValueAsDouble());
            velocityController.predict(0.02);
            DriveMotor.setVoltage(velocityController.getVoltage());
        }
    }


    @Override
    public SwerveConstants.ModuleConfig getConfig() {
        return config;
    }
}
