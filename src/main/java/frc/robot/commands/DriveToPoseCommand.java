package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveToPoseCommand extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Pose2d m_targetPose;

    // PIDs for each axis
    // TODO - these will do nothing until you increase them! Start slow!!!
    /**
     * Too low → robot moves sluggishly and never quite reaches the target
        Too high → robot overshoots and oscillates back and forth
        Just right → moves confidently and stops cleanly

        A practical tuning process:

        Put robot on field, give it a target 2 meters away
        Start P at 1.0, watch what happens
        Keep doubling (2.0, 4.0) until it starts oscillating
        Back off to about half that value
        That's your P!
     */
    // PID controllers will drive in a straight line only - we can use this for navigating between poles but not from neutral to alliance!!!
    private final PIDController m_xController = new PIDController(0.0, 0, 0);
    private final PIDController m_yController = new PIDController(0.0, 0, 0);

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();

    // TODO - update to work better for our bot size
    private static final double POSITION_TOLERANCE = 0.1; // meters

    public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        m_drivetrain = drivetrain;
        m_targetPose = targetPose;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_xController.setTolerance(POSITION_TOLERANCE);
        m_yController.setTolerance(POSITION_TOLERANCE);
    }

    @Override
    public void execute() {
        Pose2d current = LimelightHelpers.getBotPose2d_wpiBlue("limelight");

        double xSpeed = m_xController.calculate(current.getX(), m_targetPose.getX());
        double ySpeed = m_yController.calculate(current.getY(), m_targetPose.getY());

        m_drivetrain.applyRequest(() -> m_driveRequest
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        Pose2d current = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        double distance = current.getTranslation().getDistance(m_targetPose.getTranslation());
        return distance < POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> m_driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
}