package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
    
  private String trajectoryJSON = "/output/test1.wpilib.json";
  private Trajectory trajectory = new Trajectory();
  private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private Drive2 drive2 = new Drive2();

  public Command getAutonomousCommand2() {
      
    TrajectoryConfig config = new TrajectoryConfig(
        0.5, 0.5);
    config.setKinematics(drive2.getKinematics());

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {

      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }

    RamseteCommand command = new RamseteCommand(
        trajectory,
        drivetrain::getPose,
        new RamseteController(Constants.Drivetrain.RAMSETTE_B, Constants.Drivetrain.RAMSETTE_ZETA),
        drivetrain.getFeedforward(),
        drivetrain.getKinematics(),
        drivetrain::getSpeeds,
        drivetrain.getLeftPIDController(),
        drivetrain.getRightPIDController(),
        drivetrain::setOutputVolts,
        drivetrain
    );

    return command.andThen(() -> drivetrain.setOutputVolts(0, 0));
  }

  public void reset() {
    drivetrain.reset();
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        0.5, 0.5);
    config.setKinematics(drive2.getKinematics());

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {

      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }
      RamseteCommand command = new RamseteCommand(
        trajectory,
        drive2::getPose,
        new RamseteController(Constants.Drivetrain.RAMSETTE_B, Constants.Drivetrain.RAMSETTE_ZETA),
        drive2.getFeedForward(),
        drive2.getKinematics(),
        drive2::getWheelSpeeds,
        drive2.getLeftPID(),
        drive2.getRightPID(),
        drive2::tankDriveVolts,
        drive2
    );

    // Reset odometry to the starting pose of the trajectory.
    drive2.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return command.andThen(() -> drive2.tankDriveVolts(0, 0));
  }
}
