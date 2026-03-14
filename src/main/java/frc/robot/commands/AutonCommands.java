package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutonUtils;

public class AutonCommands {
    public Command getAutonomousCommand() {
        Pose2d closestPole = AutonUtils.getClosestPole();
        Pose2d oppositePole = AutonUtils.getOppositePole(closestPole);
        Pose2d shootPosition = AutonUtils.getShootPosition();

        return new SequentialCommandGroup(
            new DriveToPoseCommand(closestPole),
            new DriveToPoseCommand(oppositePole),
            new DriveToPoseCommand(shootPosition)
        ).repeatedly();
    }
}
