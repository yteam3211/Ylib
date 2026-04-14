// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.subsystems.swerve;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Ylib.subsystems.swerve.Gyro.GyroIO;
import frc.lib.Ylib.subsystems.swerve.Modules.ModuleIO;
import frc.lib.Ylib.subsystems.swerve.Modules.ModuleInputsAutoLogged;
import frc.lib.Ylib.util.PhoenixOdometryThread;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
/**
 * Subsystem class for a Swerve Drive powertrain.
 * Uses AdvantageKit for IO abstraction and PathPlanner for kinematic setpoint generation.
 */
public class swerveDrive extends SubsystemBase {
  private final ModuleIO[] modules;
  private final GyroIO gyro;
  private ModuleInputsAutoLogged[] inputs;
  private final SwerveDriveKinematics kinematics;
  private double MaxSpeedMeterPerSec;
  private SwerveModulePosition[] modulePosition;
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint lastSetpoint;
  private Rotation2d gyroAngle = new Rotation2d();
  private RobotConfig PP_Config;
  //------- odometery -------
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SysIdRoutine sysidDrive;
  private final SysIdRoutine sysidSteer;
  public static final Lock odometryLock = new ReentrantLock();
  private FollowPath.Builder pathBuilder;


  private static swerveDrive instance;
  /** * Creates a new swerveDrive subsystem.
   * @param robotMass The total mass of the robot (including battery and bumpers).
   * @param MOI The Moment of Inertia of the robot.
   * @param DriveMotor The drive motor that you use for your swerve
   * @param flModuleIO Front Left module IO layer.
   * @param frModuleIO Front Right module IO layer.
   * @param blModuleIO Back Left module IO layer.
   * @param brModuleIO Back Right module IO layer.
   */
  public swerveDrive(
    Mass robotMass,
    MomentOfInertia MOI,
    DCMotor DriveMotor,
    GyroIO gyro,
    ModuleIO flModuleIO,
    ModuleIO frModuleIO,
    ModuleIO blModuleIO,
    ModuleIO brModuleIO
  ) {
    this.modules = new ModuleIO[]{flModuleIO, frModuleIO, blModuleIO, brModuleIO};
    PhoenixOdometryThread.getInstance().start();
    this.gyro = gyro;
    this.inputs = new ModuleInputsAutoLogged[modules.length];
    this.MaxSpeedMeterPerSec = flModuleIO.getConfig().getSpeedAt12Volts();
    this.modulePosition = new SwerveModulePosition[modules.length];
    
    // sysid routine
    sysidDrive = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(5), Second.of(5), 
      (state)-> Logger.recordOutput("Swerve/sysid/Drive", state.toString())), 
      new SysIdRoutine.Mechanism((voltage)-> runVoltageDrive(voltage.in(Volts)), null, this));
    sysidSteer = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(5), Second.of(13), 
      (state)-> Logger.recordOutput("Swerve/sysid/Steer", state.toString())), 
      new SysIdRoutine.Mechanism((voltage)-> runVoltageSteer(voltage.in(Volts)), null, this));
    // Initialize Kinematics using the physical locations of the modules
    this.kinematics = new SwerveDriveKinematics(getTranslations());
    
    // Initialize Pose Estimator for odometry tracking
    this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePosition, new Pose2d());

    // Configure PathPlanner RobotConfig for trajectory following and setpoint generation
    this.PP_Config = new RobotConfig(
      robotMass, MOI, 
      new ModuleConfig(flModuleIO.getConfig().getWheelRadiusMeter(), flModuleIO.getConfig().getSpeedAt12Volts(), 1.0, DriveMotor, flModuleIO.getConfig().getSlipCurrent(), 1),
      getTranslations());

    // Set up the setpoint generator to ensure smooth acceleration and module constraints
    this.setpointGenerator = new SwerveSetpointGenerator(PP_Config, flModuleIO.getConfig().getSteerMotorGearRatio());
    
    // Initialize the last setpoint to a stationary state
    this.lastSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(), 
        kinematics.toSwerveModuleStates(new ChassisSpeeds()), 
        DriveFeedforwards.zeros(modules.length)
    );
    // AutoBuilder.configure(this::getPose, this::setPose, this::getChassisSpeedsRobotrelitive, this::runVelocity, 
    //   new PPHolonomicDriveController(new PIDConstants(0, 0, 0), new PIDConstants(0,0,0)), PP_Config, ()->DriverStation.getAlliance().isEmpty() ? false : DriverStation.getAlliance().get() == Alliance.Blue, this);
    //   PathPlannerLogging.setLogTargetPoseCallback((pose)-> Logger.recordOutput("Swerve/AutoPath/TargetPose", pose));
    //   PathPlannerLogging.setLogActivePathCallback((Path)-> Logger.recordOutput("Swerve/AutoPath/Path", Path.toArray(Pose2d[]::new)));
    // 2. Max Linear Acceleration (F = ma -> a = F/m)
    double totalStallTorque = DriveMotor.stallTorqueNewtonMeters * flModuleIO.getConfig().getDriveMotorGearRatio();
    double maxForce = totalStallTorque / flModuleIO.getConfig().getWheelRadiusMeter();
    double maxAccelerationMetersPerSec2 = maxForce / robotMass.in(Kilogram);

    // 3. Max Rotational Velocity (Omega = v / r)
    // The "drive radius" is the distance from the center of the robot to the furthest module
    double driveRadius = Math.hypot(flModuleIO.getTranslation2d().getX(), flModuleIO.getTranslation2d().getY());
    double maxVelocityRadPerSec = MaxSpeedMeterPerSec / driveRadius;
    double maxVelocityDegPerSec = Math.toDegrees(maxVelocityRadPerSec);

    // 4. Max Rotational Acceleration (Alpha = Torque_rotational / MomentOfInertia)
    // Simple approximation of Moment of Inertia (J) for a square robot
    double momentOfInertia = (1.0/12.0) * robotMass.in(Kilogram) * (Math.pow(flModuleIO.getTranslation2d().getX(), 2) + Math.pow(flModuleIO.getTranslation2d().getY(), 2));
    double maxRotationalAccelerationRadPerSec2 = (maxForce * driveRadius) / momentOfInertia;
    double maxAccelerationDegPerSec2 = Math.toDegrees(maxRotationalAccelerationRadPerSec2);

    // --- Implementation ---

    Path.setDefaultGlobalConstraints(
      new Path.DefaultGlobalConstraints(
        MaxSpeedMeterPerSec, 
        maxAccelerationMetersPerSec2, 
        maxVelocityDegPerSec, 
        maxAccelerationDegPerSec2, 
        0.05, // endTranslationToleranceMeters (5cm)
        2.0,  // endRotationToleranceDeg (2 degrees)
        0.1   // intermediateHandoffRadiusMeters
    )
  );
  pathBuilder = new FollowPath.Builder(this, this::getPose, this::getChassisSpeedsRobotrelitive, this::runVelocity, 
    new PIDController(momentOfInertia, maxRotationalAccelerationRadPerSec2, maxAccelerationDegPerSec2), 
    new PIDController(momentOfInertia, maxRotationalAccelerationRadPerSec2, maxAccelerationDegPerSec2), 
    new PIDController(momentOfInertia, maxRotationalAccelerationRadPerSec2, maxAccelerationDegPerSec2))
    .withDefaultShouldFlip()
    .withPoseReset(this::setPose);
  }
  @Override
  public void periodic() {
    if (getCurrentCommand() != null) {
      Logger.recordOutput("Swerve/Command", getCurrentCommand().getName());
    }
    odometryLock.lock();
  }
  /** 
   * use this to run the ChasisSpeeds that you want to use
   * this use the {@link SwerveSetpointGenerator} for the speeds that need to be applied
  */
  public void runVelocity(ChassisSpeeds speeds){
    lastSetpoint = setpointGenerator.generateSetpoint(lastSetpoint, speeds, 0.2);
    SwerveModuleState[] setPointStates = lastSetpoint.moduleStates();
    Logger.recordOutput("Swerve/Odometry/UnOptimizedStates", setPointStates);
    for (int i = 0; i < modules.length; i++) {
      modules[i].runState(setPointStates[i]);
    }
    // the states are being optimized in the runState
    Logger.recordOutput("Swerve/Odometry/OptimizedStates", setPointStates);
  }
  /**
   * get the translation of the moudles
   * @return the displacment of all the moudles
   */
  public Translation2d[] getTranslations(){
    Translation2d[] translation2ds = new Translation2d[modules.length];
    for (int i = 0; i < translation2ds.length; i++) {
      translation2ds[i] = modules[i].getTranslation2d();
    }
    return translation2ds;
  }
  public ChassisSpeeds getChassisSpeedsRobotrelitive(){
    return kinematics.toChassisSpeeds();
  }
  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }
  public void setPose(Pose2d pose){
    poseEstimator.resetPosition(gyroAngle, modulePosition, pose);
  }
  public void runVoltageDrive(double Voltage){
    for (ModuleIO module : modules) {
      module.setDriveOpenLoop(Voltage);
      module.setTurnPosition(gyroAngle.unaryMinus());
    }
  }
  
  public void runVoltageSteer(double Voltage){
    for (ModuleIO module : modules) {
      module.setTurnOpenLoop(Voltage);
      module.setDriveOpenLoop(0);
    }
  }
  public Command SysidDrive(){
    return stopCommand().andThen(
      sysidDrive.dynamic(SysIdRoutine.Direction.kForward).andThen(stopCommand()
        .andThen(sysidDrive.dynamic(SysIdRoutine.Direction.kReverse).andThen(stopCommand()
          .andThen(sysidDrive.quasistatic(SysIdRoutine.Direction.kForward).andThen(stopCommand()
            .andThen(sysidDrive.quasistatic(SysIdRoutine.Direction.kReverse)))))))); 
  }

  public Command SysidSteer(){
    return stopCommand().andThen(
      sysidSteer.dynamic(SysIdRoutine.Direction.kForward).andThen(stopCommand()
        .andThen(sysidSteer.dynamic(SysIdRoutine.Direction.kReverse).andThen(stopCommand()
          .andThen(sysidSteer.quasistatic(SysIdRoutine.Direction.kForward).andThen(stopCommand()
            .andThen(sysidSteer.quasistatic(SysIdRoutine.Direction.kReverse)))))))); 
  }  
  public void Stop(){
    runVelocity(new ChassisSpeeds());
  }
  private Command stopCommand(){
    return this.runOnce(()-> Stop());
  }
}
