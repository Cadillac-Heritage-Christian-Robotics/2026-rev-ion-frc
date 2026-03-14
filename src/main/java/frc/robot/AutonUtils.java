package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TargetPositions;

public final class AutonUtils {

    /**
     * From your current position, calculate if you are closer to the "North" or "South" Poles to reload.
     * @return 
     */
    public static Pose2d getClosestPole() {
        Pose2d current = LimelightHelpers.getBotPose2d_wpiBlue("limelight");

        double distNorth = current.getTranslation().getDistance(TargetPositions.NORTH_POLE.getTranslation());
        double distSouth = current.getTranslation().getDistance(TargetPositions.SOUTH_POLE.getTranslation());

        return distNorth < distSouth ? TargetPositions.NORTH_POLE : TargetPositions.SOUTH_POLE;
    }

  // TODO - we need better logic for this to see which 'half' of the space we are in, not the specific box
  public static Pose2d getOppositePole(Pose2d currentTarget) {
      return currentTarget.equals(TargetPositions.NORTH_POLE) 
          ? TargetPositions.SOUTH_POLE 
          : TargetPositions.NORTH_POLE;
  }

    public static Pose2d getShootPosition() {
        var alliance = DriverStation.getAlliance();
        // TODO - if not set then PANIC
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return TargetPositions.RED_SHOOT;
        }

        return TargetPositions.BLUE_SHOOT;
    }

  // public Command getAutonomousCommand() {
  //   return new SequentialCommandGroup(
  //       new ShootCommand(drivetrain, m_shooter),     // shoot preloaded
  //       new DriveToPoseCommand(drivetrain, AutonUtils.getClosestPole()),   // go to nearest pole
  //       new DriveToPoseCommand(drivetrain, AutonUtils.getOppositePole()),  // plow through to other side
  //       new DriveToPoseCommand(drivetrain, AutonUtils.getShootPosition()), // drive to shoot spot
  //       new ShootCommand(drivetrain, m_shooter)      // shoot
  //   ).repeatedly();
  // }
}