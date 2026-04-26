// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Ylib.subsystems.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Ylib.subsystems.swerve.Gyro.GyroIO;
import frc.lib.Ylib.subsystems.swerve.Gyro.GyroIOInputsAutoLogged;
import frc.lib.Ylib.subsystems.swerve.Modules.ModuleIO;
import frc.lib.Ylib.subsystems.swerve.Modules.ModuleInputsAutoLogged;
import frc.lib.Ylib.util.PhoenixOdometryThread;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for a Swerve Drive. *
 *
 * <p>Features:
 *
 * <ul>
 *   <li>IO Abstraction via AdvantageKit for hardware-agnostic simulation/replaying.
 *   <li>PathPlanner integration for autonomous path following.
 *   <li>High-frequency odometry updates via a dedicated thread.
 *   <li>SysId routines for characterization of drive and steer motors.
 * </ul>
 */
public class swerveDrive extends SubsystemBase {
  private final ModuleIO[] modules;
  private final GyroIO gyro;
  private GyroIOInputsAutoLogged gyroInputs;
  private ModuleInputsAutoLogged[] inputs;
  private final SwerveDriveKinematics kinematics;
  private double MaxSpeedMeterPerSec;
  private SwerveModulePosition[] LastmodulePosition;
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint lastSetpoint;
  private Rotation2d gyroAngle = new Rotation2d();
  private RobotConfig PP_Config;
  // ------- odometery -------
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SysIdRoutine sysidDrive;
  private final SysIdRoutine sysidSteer;
  public static final Lock odometryLock = new ReentrantLock();

  private swerveDrive instance;
  /**
   * Creates a new swerveDrive subsystem.
   *
   * @param robotMass Total mass of the robot (kg).
   * @param MOI Moment of Inertia of the robot (kg*m^2).
   * @param DriveMotor The DCMotor model used for drive motors.
   * @param gyro The gyro IO implementation.
   * @param flModuleIO Front Left module IO.
   * @param frModuleIO Front Right module IO.
   * @param blModuleIO Back Left module IO.
   * @param brModuleIO Back Right module IO.
   */
  public swerveDrive(
      Mass robotMass,
      MomentOfInertia MOI,
      DCMotor DriveMotor,
      GyroIO gyro,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    // Start the high-frequency thread for capturing odometry signals (e.g., from Phoenix Pro)
    PhoenixOdometryThread.getInstance().start();

    this.modules = new ModuleIO[] {flModuleIO, frModuleIO, blModuleIO, brModuleIO};
    this.inputs = new ModuleInputsAutoLogged[modules.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new ModuleInputsAutoLogged();
    }

    this.gyro = gyro;
    gyroInputs = new GyroIOInputsAutoLogged();

    this.MaxSpeedMeterPerSec = flModuleIO.getConfig().getSpeedAt12Volts();
    this.LastmodulePosition = new SwerveModulePosition[modules.length];
    for (int i = 0; i < LastmodulePosition.length; i++) {}

    // sysid routine
    sysidDrive =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(5),
                Second.of(5),
                (state) -> Logger.recordOutput("Swerve/sysid/Drive", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runVoltageDrive(voltage.in(Volts)), null, this));
    sysidSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(5),
                Second.of(13),
                (state) -> Logger.recordOutput("Swerve/sysid/Steer", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runVoltageSteer(voltage.in(Volts)), null, this));
    // Initialize Kinematics using the physical locations of the modules
    this.kinematics = new SwerveDriveKinematics(getTranslations());

    // Initialize Pose Estimator for odometry tracking
    this.poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, gyroAngle, LastmodulePosition, new Pose2d());

    // Configure PathPlanner RobotConfig for trajectory following and setpoint generation
    this.PP_Config =
        new RobotConfig(
            robotMass,
            MOI,
            new ModuleConfig(
                flModuleIO.getConfig().getWheelRadiusMeter(),
                flModuleIO.getConfig().getSpeedAt12Volts(),
                1.0,
                DriveMotor,
                flModuleIO.getConfig().getSlipCurrent(),
                1),
            getTranslations());

    // Set up the setpoint generator to ensure smooth acceleration and module constraints
    this.setpointGenerator =
        new SwerveSetpointGenerator(PP_Config, flModuleIO.getConfig().getSteerMotorGearRatio());

    // Initialize the last setpoint to a stationary state
    this.lastSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            kinematics.toSwerveModuleStates(new ChassisSpeeds()),
            DriveFeedforwards.zeros(modules.length));
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeedsRobotrelitive,
        this::runVelocity,
        new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0)),
        PP_Config,
        () ->
            DriverStation.getAlliance().isEmpty()
                ? false
                : DriverStation.getAlliance().get() == Alliance.Blue,
        this);
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> Logger.recordOutput("Swerve/AutoPath/TargetPose", pose));
    PathPlannerLogging.setLogActivePathCallback(
        (Path) -> Logger.recordOutput("Swerve/AutoPath/Path", Path.toArray(Pose2d[]::new)));
  }

  @Override
  public void periodic() {
    if (getCurrentCommand() != null) {
      Logger.recordOutput("Swerve/Command", getCurrentCommand().getName());
    }
    odometryLock.lock();
    for (int index = 0; index < modules.length; index++) {
      modules[index].UpdateInputs(inputs[index]);
      Logger.processInputs("Swerve/module " + modules[index].getConfig().getName(), inputs[index]);
    }
    gyro.UpdateInputs(gyroInputs);
    if (DriverStation.isDisabled()) {
      Stop();
    }
    double[] sampleTimestamps = inputs[0].odometryTimestamps; // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
      SwerveModulePosition[] moduleDelta = new SwerveModulePosition[modules.length];
      for (int j = 0; j < modules.length; j++) {
        modulePositions[j] =
            new SwerveModulePosition(
                inputs[j].odometryDrivePositionsRot[i]
                    * modules[j].getConfig().getWheelRadiusMeter()
                    * 2
                    * Math.PI,
                inputs[j].odometrySteerPositions[i]);
        moduleDelta[j] =
            new SwerveModulePosition(
                modulePositions[j].distanceMeters - LastmodulePosition[j].distanceMeters,
                modulePositions[j].angle);
        LastmodulePosition[j] = modulePositions[j];
      }
      if (gyroInputs.connected) {
        gyroAngle = gyroInputs.yawPitchRollPosition.toRotation2d();
      } else {
        Twist2d twist2d = kinematics.toTwist2d(moduleDelta);
        gyroAngle = gyroAngle.plus(new Rotation2d(twist2d.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], gyroAngle, modulePositions);
    }
  }
  /**
   * use this to run the ChasisSpeeds that you want to use this use the {@link
   * SwerveSetpointGenerator} for the speeds that need to be applied
   */
  public void runVelocity(ChassisSpeeds speeds) {
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
   *
   * @return the displacment of all the moudles
   */
  public Translation2d[] getTranslations() {
    Translation2d[] translation2ds = new Translation2d[modules.length];
    for (int i = 0; i < translation2ds.length; i++) {
      translation2ds[i] = modules[i].getTranslation2d();
    }
    return translation2ds;
  }

  public ChassisSpeeds getChassisSpeedsRobotrelitive() {
    return kinematics.toChassisSpeeds();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(gyroAngle, LastmodulePosition, pose);
  }

  public void runVoltageDrive(double Voltage) {
    for (ModuleIO module : modules) {
      module.setDriveOpenLoop(Voltage);
      module.setTurnPosition(gyroAngle.unaryMinus());
    }
  }

  public void runVoltageSteer(double Voltage) {
    for (ModuleIO module : modules) {
      module.setTurnOpenLoop(Voltage);
      module.setDriveOpenLoop(0);
    }
  }

  public Command SysidDrive() {
    return stopCommand()
        .andThen(
            sysidDrive
                .dynamic(SysIdRoutine.Direction.kForward)
                .andThen(
                    stopCommand()
                        .andThen(
                            sysidDrive
                                .dynamic(SysIdRoutine.Direction.kReverse)
                                .andThen(
                                    stopCommand()
                                        .andThen(
                                            sysidDrive
                                                .quasistatic(SysIdRoutine.Direction.kForward)
                                                .andThen(
                                                    stopCommand()
                                                        .andThen(
                                                            sysidDrive.quasistatic(
                                                                SysIdRoutine.Direction
                                                                    .kReverse))))))));
  }

  public Command SysidSteer() {
    return stopCommand()
        .andThen(
            sysidSteer
                .dynamic(SysIdRoutine.Direction.kForward)
                .andThen(
                    stopCommand()
                        .andThen(
                            sysidSteer
                                .dynamic(SysIdRoutine.Direction.kReverse)
                                .andThen(
                                    stopCommand()
                                        .andThen(
                                            sysidSteer
                                                .quasistatic(SysIdRoutine.Direction.kForward)
                                                .andThen(
                                                    stopCommand()
                                                        .andThen(
                                                            sysidSteer.quasistatic(
                                                                SysIdRoutine.Direction
                                                                    .kReverse))))))));
  }

  public void Stop() {
    runVelocity(new ChassisSpeeds());
  }

  private Command stopCommand() {
    return this.runOnce(() -> Stop());
  }

  public swerveDrive getInstance() {
    if (instance == null) {
      throw new RuntimeException("Instance not initialized use ");
    }
    return instance;
  }

  public void InitializeSwerve(swerveDrive swerve) {
    this.instance = swerve;
  }
}
