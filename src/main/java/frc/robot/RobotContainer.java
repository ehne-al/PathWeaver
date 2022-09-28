package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
    
  private String trajectoryJSON = "paths\\output\\test1.wpilib.json";
  private Trajectory trajectory = new Trajectory();
  private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  public Command getAutonomousCommand() {
      
    TrajectoryConfig config = new TrajectoryConfig(
        Units.feetToMeters(2.0), Units.feetToMeters(2.0));
    config.setKinematics(drivetrain.getKinematics());

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
}