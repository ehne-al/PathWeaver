// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.Drivetrain.LEFT_MASTER_ID);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.Drivetrain.RIGHT_MASTER_ID);

  private final WPI_TalonSRX leftSlave = new WPI_TalonSRX(Constants.Drivetrain.LEFT_SLAVE_ID);
  private final WPI_TalonSRX rightSlave = new WPI_TalonSRX(Constants.Drivetrain.RIGHT_SLAVE_ID);

  //private final Encoder leftEncoder = new Encoder(Constants.Drivetrain.LEFT_ENCODER_A_ID, Constants.Drivetrain.LEFT_ENCODER_B_ID);
  //private final Encoder rightEncoder = new Encoder(Constants.Drivetrain.RIGHT_ENCODER_A_ID, Constants.Drivetrain.RIGHT_ENCODER_B_ID);

  public ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  private DifferentialDriveOdometry odometry;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0003, 0.0002, 0.0006);

  private PIDController leftPIDController = new PIDController(Constants.Drivetrain.KP,Constants.Drivetrain.KI,Constants.Drivetrain.KD);
  private PIDController rightPIDController = new PIDController(Constants.Drivetrain.KP,Constants.Drivetrain.KI,Constants.Drivetrain.KD);

  private Pose2d pose = new Pose2d();

  public DrivetrainSubsystem() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 1, 1);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 1);

    odometry = new DifferentialDriveOdometry(getHeading(), getPose());

    gyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getSelectedSensorVelocity(1),
        rightMaster.getSelectedSensorVelocity(0));
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

  public double getLeftEncoderPosition(){
    return leftMaster.getSelectedSensorPosition(1);
  } 

  public double getRightEncoderPosition(){
    return rightMaster.getSelectedSensorPosition(0);
  } 

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getLeftEncoderPosition(), getRightEncoderPosition());
  }
}

