// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  private static final double kGearRatio = 10;

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.Drivetrain.LEFT_MASTER_ID);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.Drivetrain.RIGHT_MASTER_ID);

  private final WPI_TalonSRX leftSlave = new WPI_TalonSRX(Constants.Drivetrain.LEFT_SLAVE_ID);
  private final WPI_TalonSRX rightSlave = new WPI_TalonSRX(Constants.Drivetrain.RIGHT_SLAVE_ID);

  private final Encoder leftEncoder = new Encoder(Constants.Drivetrain.LEFT_ENCODER_A_ID, Constants.Drivetrain.LEFT_ENCODER_B_ID);
  private final Encoder rightEncoder = new Encoder(Constants.Drivetrain.RIGHT_ENCODER_A_ID, Constants.Drivetrain.RIGHT_ENCODER_B_ID);

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  private DifferentialDriveOdometry odometry;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);

  private PIDController leftPIDController = new PIDController(Constants.Drivetrain.KP,Constants.Drivetrain.KI,Constants.Drivetrain.KD);
  private PIDController rightPIDController = new PIDController(Constants.Drivetrain.KP,Constants.Drivetrain.KI,Constants.Drivetrain.KD);

  private Pose2d pose = new Pose2d();

  public DrivetrainSubsystem() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
    odometry = new DifferentialDriveOdometry(getHeading(), getPose());

    gyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftEncoder.getDistance() / kGearRatio * 2 * Math.PI * Constants.Drivetrain.WHEEL_RADIUS / 60,
        rightEncoder.getDistance()/ kGearRatio * 2 * Math.PI * Constants.Drivetrain.WHEEL_RADIUS / 60
    );
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / 20);
    rightMaster.set(rightVolts / 20);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }
}

