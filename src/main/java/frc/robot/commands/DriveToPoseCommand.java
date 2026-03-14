package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPoseCommand extends Command {

    private final Pose2d m_targetPose;
    private Command m_pathCommand;

    // Speed constraints for pathfinding
    // TODO - tune these for your robot!
    public static final PathConstraints CONSTRAINTS = new PathConstraints(
        0.5,   // max velocity m/s TODO - tune this!
        0.5,   // max acceleration m/s² TODO - tune this
        Math.PI,     // max angular velocity rad/s
        Math.PI      // max angular acceleration rad/s²
    );

    public DriveToPoseCommand(Pose2d targetPose) {
        m_targetPose = targetPose;
        // NOTE: no addRequirements here — PathPlanner handles that internally!
    }

    @Override
    public void initialize() {
        // Build the path at runtime so it uses current robot position as start
        m_pathCommand = AutoBuilder.pathfindToPose(m_targetPose, CONSTRAINTS);
        m_pathCommand.initialize();
    }

    @Override
    public void execute() {
        m_pathCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return m_pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_pathCommand.end(interrupted);
    }
}
