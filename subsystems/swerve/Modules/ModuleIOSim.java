// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.subsystems.swerve.Modules;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.Ylib.subsystems.swerve.SwerveConstants.ModuleConfig;

/** Add your docs here. */
public class ModuleIOSim implements ModuleIO{
    private ModuleConfig config;
    private ProfiledPIDController positionController;
    private PIDController  VelocityController;
    private SimpleMotorFeedforward VelocityFeedforward;
    private DCMotorSim SteerSim;
    private DCMotorSim DriveSim;
    private double VelocityTarget;
    private Rotation2d PostionTarget = new Rotation2d();
    private boolean VelocityClosedLoop;
    private boolean postionClosedLoop;
    public ModuleIOSim(ModuleConfig config,DCMotor DriveMotor,DCMotor SteerMotor){
        this.config = config;
        positionController = new ProfiledPIDController(config.getSteerMotorGains().getK_p(), config.getSteerMotorGains().getK_i(),  config.getSteerMotorGains().getK_d(), 
        new TrapezoidProfile.Constraints(SteerMotor.withReduction(config.getSteerMotorGearRatio()).freeSpeedRadPerSec, SteerMotor.withReduction(config.getSteerMotorGearRatio()).freeSpeedRadPerSec / 0.100));
        positionController.enableContinuousInput(-0.5, 0.5);
        VelocityController = new PIDController(config.getDriveMotorGains().getK_p(), config.getDriveMotorGains().getK_i(), config.getDriveMotorGains().getK_d());
        VelocityFeedforward = new SimpleMotorFeedforward(config.getDriveMotorGains().getK_s(), config.getDriveMotorGains().getK_v(),config.getDriveMotorGains().getK_a());
        SteerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(SteerMotor, config.getSteerInertia(), config.getSteerMotorGearRatio()), SteerMotor,0.01,0.08);
        DriveSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DriveMotor, config.getDriveInertia(), config.getDriveMotorGearRatio()), DriveMotor, 0.03,0.13);
    }

    @Override
    public void UpdateInputs(ModuleInputs inputs) {
        UpdateSim();
        /** all the drive data */
        inputs.driveConnected = true;
        inputs.driveCurrentAmps = DriveSim.getTorqueNewtonMeters();
        inputs.driveAppliedVolts = DriveSim.getInputVoltage();
        inputs.drivePositionRot = DriveSim.getAngularPositionRotations();
        inputs.driveVelocityRotPerSec = DriveSim.getAngularVelocityRPM() / 60;
        /** all the Steer data */
        inputs.SteerConnected = true;
        inputs.SteerCurrentAmps = SteerSim.getTorqueNewtonMeters();
        inputs.SteerAppliedVolts = SteerSim.getInputVoltage();
        inputs.SteerPosition = Rotation2d.fromRadians(SteerSim.getAngularPositionRad());
        inputs.SteerEncoderConnected = true;
        inputs.SteerAbsolutePosition = Rotation2d.fromRotations(DriveSim.getAngularPositionRotations());
        inputs.SteerVelocityRotPerSec = SteerSim.getAngularVelocityRPM() / 60;
    }

    private void UpdateSim() {
        if (VelocityClosedLoop) {
            DriveSim.setInputVoltage(VelocityFeedforward.calculate(VelocityTarget) + VelocityController.calculate(DriveSim.getAngularVelocityRPM()/60, VelocityTarget));
        }
        DriveSim.update(0.02);
        if (postionClosedLoop) {
            SteerSim.setInputVoltage(positionController.calculate(SteerSim.getAngularPositionRotations(),PostionTarget.getRotations()));
        }
        SteerSim.update(0.02);
    }

 @Override
    public Translation2d getTranslation2d() {
        return new Translation2d(config.getLocationX(),config.getLocationY());
    }

    @Override
    public void runState(SwerveModuleState state) {
        state.optimize(Rotation2d.fromRadians(SteerSim.getAngularPositionRad()));
        setDriveVelocity(state.speedMetersPerSecond);
        setTurnPosition(state.angle);
    }

    @Override
    public void setDriveOpenLoop(double Votlage) {
        VelocityClosedLoop = false;
        DriveSim.setInputVoltage(Votlage);
    }

    @Override
    public void setDriveVelocity(double VelocityRotPerSec) {
        VelocityTarget = VelocityRotPerSec;
        VelocityClosedLoop = true;
    }

    @Override
    public void setDriveVelocity(AngularVelocity Velocity) {
        setDriveVelocity(Velocity.in(RotationsPerSecond));
    }

    @Override
    public void setTurnOpenLoop(double Votlage) {
        postionClosedLoop = false;
        DriveSim.setInputVoltage(Votlage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        postionClosedLoop = true;
        this.PostionTarget = rotation;
    }

    @Override
    public ModuleConfig getConfig() {
        return config;
    }
}