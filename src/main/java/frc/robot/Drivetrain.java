// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;


/** Represents a mecanum drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 7.5; // 3 meters per second

  private final SparkMax m_frontLeftMotor = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax m_frontRightMotor = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax m_backLeftMotor = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax m_backRightMotor = new SparkMax(3, MotorType.kBrushed);

  private final RelativeEncoder m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
  private final RelativeEncoder m_frontRightEncoder = m_frontRightMotor.getEncoder();
  private final RelativeEncoder m_backLeftEncoder = m_backLeftMotor.getEncoder();
  private final RelativeEncoder m_backRightEncoder = m_backRightMotor.getEncoder();

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
  
  private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

  //private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(m_kinematics, navx.getRotation2d(), getCurrentDistances());

  // Gains are for example purposes only - must be determined for your own robot!

  /** Constructs a MecanumDrive and resets the gyro. */
  public Drivetrain() {
    navx.reset();
    SparkMaxConfig flconfig = new SparkMaxConfig();
    flconfig.encoder.countsPerRevolution(Constants.ENCODER_COUNTS_PER_REVOLUTION)
        .inverted(Constants.ENCODER_INVERTED);

        flconfig.closedLoop.
        p(Constants.kP)
        .i(Constants.kI)
        .d(Constants.kD);

        flconfig.closedLoop.feedForward.
        kS(Constants.kS)
        .kV(Constants.kV)
        .kA(Constants.kA);

        flconfig.closedLoop.maxMotion.
        cruiseVelocity(Constants.cruiseVelocity)
        .maxAcceleration(Constants.acceleration)
        .allowedProfileError(Constants.allowedProfileError);

        flconfig.smartCurrentLimit(9);

        flconfig.inverted(true);

        m_frontLeftMotor.configure(flconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig frconfig = new SparkMaxConfig();
        frconfig.encoder.countsPerRevolution(Constants.ENCODER_COUNTS_PER_REVOLUTION)
        .inverted(Constants.ENCODER_INVERTED);

        frconfig.closedLoop.
        p(Constants.kP)
        .i(Constants.kI)
        .d(Constants.kD);

        frconfig.closedLoop.feedForward.
        kS(Constants.kS)
        .kV(Constants.kV)
        .kA(Constants.kA);

        frconfig.closedLoop.maxMotion.
        cruiseVelocity(Constants.cruiseVelocity)
        .maxAcceleration(Constants.acceleration)
        .allowedProfileError(Constants.allowedProfileError);

        frconfig.smartCurrentLimit(9);

        frconfig.inverted(false);

        m_frontRightMotor.configure(frconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig blconfig = new SparkMaxConfig();
        blconfig.encoder.countsPerRevolution(Constants.ENCODER_COUNTS_PER_REVOLUTION)
        .inverted(Constants.ENCODER_INVERTED);

        blconfig.closedLoop.
        p(Constants.kP)
        .i(Constants.kI)
        .d(Constants.kD);

        blconfig.closedLoop.feedForward.
        kS(Constants.kS)
        .kV(Constants.kV)
        .kA(Constants.kA);

        blconfig.closedLoop.maxMotion.
        cruiseVelocity(Constants.cruiseVelocity)
        .maxAcceleration(Constants.acceleration)
        .allowedProfileError(Constants.allowedProfileError);

        blconfig.smartCurrentLimit(9);

        blconfig.inverted(true);
        m_backLeftMotor.configure(blconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig brconfig = new SparkMaxConfig();
        brconfig.encoder.countsPerRevolution(Constants.ENCODER_COUNTS_PER_REVOLUTION)
        .inverted(Constants.ENCODER_INVERTED);

        brconfig.closedLoop.
        p(Constants.kP)
        .i(Constants.kI)
        .d(Constants.kD);

        brconfig.closedLoop.feedForward.
        kS(Constants.kS)
        .kV(Constants.kV)
        .kA(Constants.kA);

        brconfig.closedLoop.maxMotion.
        cruiseVelocity(Constants.cruiseVelocity)
        .maxAcceleration(Constants.acceleration)
        .allowedProfileError(Constants.allowedProfileError);

        brconfig.smartCurrentLimit(9);

        brconfig.inverted(false);
        m_backRightMotor.configure(brconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
  }

  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_backLeftEncoder.getVelocity(),
        m_backRightEncoder.getVelocity());
  }

  /**
   * Returns the current distances measured by the drivetrain.
   *
   * @return The current distances measured by the drivetrain.
   */
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getPosition(),
        m_frontRightEncoder.getPosition(),
        m_backLeftEncoder.getPosition(),
        m_backRightEncoder.getPosition());
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    m_frontLeftMotor.setVoltage(speeds.frontLeftMetersPerSecond);
    m_frontRightMotor.setVoltage(speeds.frontRightMetersPerSecond);
    m_backLeftMotor.setVoltage(speeds.rearLeftMetersPerSecond);
    m_backRightMotor.setVoltage(speeds.rearRightMetersPerSecond);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, navx.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(navx.getRotation2d(), getCurrentDistances());
  }
}
