// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.cameraserver.CameraServer;


public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final SendableChooser<Boolean> m_autoDefault = new SendableChooser<>();

    private final SendableChooser<String> m_autoLocation = new SendableChooser<>();
    private final SendableChooser<String> m_autoColor = new SendableChooser<>();
    private final SendableChooser<String> m_autoStrategy = new SendableChooser<>();
    private final SendableChooser<Double> m_timeToShoot = new SendableChooser<>();

    private final SendableChooser<Boolean> m_alignToHub = new SendableChooser<>();


    // Subsystems
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    // Increase these from 0 to 1 with 1 being full speed driving/turning
    private double SlowMoDrive = 0.75;
    private double SlowMoTurn = 1.5;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Autonomous commands
    // Simple drive forward for about 2 meters for 1 seconds
    private final DriveForwardCommand m_DriveForwardCommand = new DriveForwardCommand(drivetrain);

    public RobotContainer() {
        System.out.println("Alliance at configure time: " + DriverStation.getAlliance());

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            () -> drivetrain.getState().Pose,           // how to get current pose
            drivetrain::resetPose,                       // how to reset pose
            () -> drivetrain.getState().Speeds,          // current chassis speeds
            (speeds, feedforwards) -> drivetrain.setControl(  // how to drive
                new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0, 0),   // translation PID
                new PIDConstants(5.0, 0, 0)    // rotation PID
            ),
            TunerConstants.PP_CONFIG,           // robot config
            () -> false,
            drivetrain
        );

        // Register named commands for PathPlanner
        NamedCommands.registerCommand("StartIntake", m_intake.runIntakeCommand());
        NamedCommands.registerCommand("StopIntake",  m_intake.runOnce(() -> {}));

        NamedCommands.registerCommand("SlapArmDown", m_intake.runSlapDownCommand());
        NamedCommands.registerCommand("SlapArmUp",   m_intake.runSlapUpCommand());

        // Register your path options
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            System.out.println("Alliance detection worked - safe to remove m_autocolor SelectableChoose from RobotContainer.java!");
            m_autoColor.setDefaultOption("Red", "Red");
            m_autoColor.addOption("Blue", "Blue");
        } else {
            m_autoColor.setDefaultOption("Blue", "Blue");
            m_autoColor.addOption("Red", "Red");
        }


        m_autoLocation.setDefaultOption("North", "North");
        m_autoLocation.addOption("South", "South");

        m_autoStrategy.setDefaultOption("Alpha", "Alpha");
        m_autoStrategy.addOption("Bravo", "Bravo");

        // Push it to SmartDashboard so drive team can see it
        SmartDashboard.putData("Alliance Color", m_autoColor);
        SmartDashboard.putData("Starting Location", m_autoLocation);
        SmartDashboard.putData("Auton Strategy", m_autoStrategy);

        m_autoDefault.setDefaultOption("Use Default Strategy", true);
        m_autoDefault.addOption("Use Alpha|Bravo Strategy", false);
        SmartDashboard.putData("Override Auton Strategy", m_autoDefault);

        m_timeToShoot.setDefaultOption("3.0", 3.0);
        m_timeToShoot.addOption("1.0", 1.0);
        m_timeToShoot.addOption("2.0", 2.0);
        m_timeToShoot.addOption("4.0", 4.0);
        m_timeToShoot.addOption("5.0", 5.0);

        m_alignToHub.setDefaultOption("No Shooting Align", false);
        m_alignToHub.addOption("Align Shooting", true);
        SmartDashboard.putData("Align To Hub", m_alignToHub);

        SmartDashboard.putData("Auton Shoot Duration.2", m_timeToShoot);

        configureBindings();
    }

     private void configureBindings() {
         // Note that X is defined as forward according to WPILib convention,
         // and Y is defined as to the left according to WPILib convention.
         drivetrain.setDefaultCommand(
                 // Drivetrain will execute this command periodically
                 drivetrain
                         // Drive forward with negative Y (forward)
                         .applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * SlowMoDrive)
                                 // Drive left with negative X (left)
                                 .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * SlowMoDrive)
                                 // Drive counterclockwise with negative X (left)
                                 .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate * SlowMoTurn)));

      
         /* DRIVER CONTROLS */

        m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    //     // Left Bumper -> Run tube intake
    //     m_driverController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());

    //     // Right Bumper -> Run tube intake in reverse
    //     m_driverController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());

        /*** OPERATOR CONTROLS ***/

    // Right Trigger -> Run fuel intake in reverse
    m_operatorController
      .rightTrigger(OIConstants.kTriggerButtonThreshold)
      .whileTrue(m_intake.runIntakeCommand());

    // Left Trigger -> Run fuel intake in reverse
    m_operatorController
      .leftTrigger(OIConstants.kTriggerButtonThreshold)
      .whileTrue(m_intake.runExtakeCommand());

    // Y Button -> Run intake and run the shooter flywheel and feeder
    m_operatorController.y().toggleOnTrue(m_shooter.runShooterCommand().alongWith(m_intake.runIntakeCommand()));
    m_operatorController.x().toggleOnTrue(m_shooter.runShooterCommand());
    m_operatorController.a().toggleOnTrue(m_intake.runSlapUpCommand());
    m_operatorController.b().toggleOnTrue(m_intake.runSlapDownCommand());

        // B Button -> Elevator/Arm to human player position, set ball intake to stow when idle
        
        // // Right Trigger -> Run ball intake, set to leave out when idle
        // // m_operatorController
        //         // .rightTrigger(Constants.kTriggerButtonThreshold)
        //         // .whileTrue(m_algaeSubsystem.runIntakeCommand());

        // // Left Trigger -> Run ball intake in reverse, set to stow when idle
        // m_operatorController
        //         .leftTrigger(Constants.kTriggerButtonThreshold)
        //         .whileTrue(m_algaeSubsystem.reverseIntakeCommand());

        // m_operatorController.b().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation)
        //         .alongWith(m_algaeSubsystem.stowCommand()));

        // // A Button -> Elevator/Arm to level 2 position
         //m_operatorController.a().onTrue(m_intakeSubSystem.setSetpointCommand(Setpoint.kLevel2));

        // // X Button -> Elevator/Arm to level 3 position
        // m_operatorController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

        // // Y Button -> Elevator/Arm to level 4 position
        // m_operatorController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));


    } // end of configureBindings

    public Command getAutonomousCommand() {
        try {
            /** 
             * All Auton Paths are structed in the format: $Color$Location$Strategy
             * Color = "Red" or "Blue"
             * Location = "North" or "South"
             * Stratergy = "Alpha" or "Bravo"
             * The default Strategy is "RedNorthAlpha"
            **/
            String selectedColor = m_autoColor.getSelected();
            String selectedLocation = m_autoLocation.getSelected();
            String selectedStrat = m_autoStrategy.getSelected();
            Boolean autoDefault = m_autoDefault.getSelected();

            Double timeToShoot = m_timeToShoot.getSelected();

            String selectedPath = selectedColor + selectedLocation + selectedStrat;

            if (Boolean.TRUE.equals(autoDefault)) {
                System.out.println("Overriding path with default path");
                selectedPath = selectedColor + selectedLocation + "Default";
            }

            System.out.println("Selected Path = " + selectedPath);

            // Manually curated list of starting positions based on with PathPlanner we are using.
            Pose2d targetPosition = AutonUtils.getDesiredPose(selectedPath);
            System.out.println(targetPosition.toString());
            PathPlannerPath path = PathPlannerPath.fromPathFile(selectedPath);

            return new SequentialCommandGroup(
                // Step 0: Bootstrap pose from MegaTag1 so gyro is correct before pathfinding
                Commands.run(() -> {
                    LimelightHelpers.SetRobotOrientation("limelight-robot", drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
                    PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-robot");
                    if (LimelightHelpers.validPoseEstimate(mt1)) {
                        System.out.println("[Localize] Got fix! X=" + mt1.pose.getX() + " Y=" + mt1.pose.getY() + ", Deg=" + mt1.pose.getRotation().getDegrees());
                        drivetrain.resetPose(mt1.pose);
                    } else {
                        System.out.println("[Localize] No valid estimate yet... tagCount=" + (mt1 != null ? mt1.tagCount : "null"));
                    }
                }, drivetrain).until(() -> 
                    LimelightHelpers.validPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-robot"))
                ).withTimeout(60.0),
                // Step 1: Bootstrap pose from Strategy - ONLY trenches allowed!!!
                // drivetrain.runOnce(() -> {
                //     SmartDashboard.putString("Starting Position", startingPosition.toString());
                //     drivetrain.resetPose(startingPosition);
                // }),
                // Step 2: Drive to shoot position
                drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Driving to shoot position")),
                AutoBuilder.pathfindToPose(targetPosition, DriveToPoseCommand.CONSTRAINTS),

                // Step 3: Shoot preloaded fuel
                Boolean.TRUE.equals(m_alignToHub.getSelected()) ? alignToHubCommand() : Commands.none(),
                drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Shooting")),
                m_shooter.runShooterCommand().withTimeout(timeToShoot).deadlineWith(m_intake.runIntakeCommand()),

                // Step 4: Always run the selected path (intake via event markers)
                drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Running path2")),
                AutoBuilder.followPath(path),

                // Step 5: Shoot again only if Alpha/Bravo strategy (not default)
                Boolean.FALSE.equals(autoDefault)
                    ? new SequentialCommandGroup(
                        drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Shooting again")),
                        Boolean.TRUE.equals(m_alignToHub.getSelected()) ? alignToHubCommand() : Commands.none(),
                        m_shooter.runShooterCommand().withTimeout(timeToShoot * 2).deadlineWith(m_intake.runIntakeCommand())
                    )
                    : Commands.none(),

                drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Done!"))
            );

        } catch (FileVersionException | IOException | IllegalArgumentException | ParseException e) {
            e.printStackTrace();
            return m_DriveForwardCommand;
        }
    }
    public Command alignToHubCommand() {
        // Determine which tags to use based on alliance
        boolean isRed = DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red;

        int[] hubTags = isRed ? new int[]{9, 10} : new int[]{25, 26};

        return Commands.sequence(
            // Set filter to only see hub center tags
            Commands.runOnce(() -> 
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-robot", hubTags)
            ),

            // Align to hub
            drivetrain.applyRequest(() -> {
                double tx = LimelightHelpers.getTX("limelight-robot");
                double kP = 0.05; // tune this!
                double rotationRate = -tx * kP;

                return drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationRate);
            })
            .until(() -> 
                LimelightHelpers.getTV("limeligh-robot") && // make sure we actually see a tag!
                Math.abs(LimelightHelpers.getTX("limelight-robot")) < 2.0
            )
            .withTimeout(1.0),

            // Clear the filter after aligning
            Commands.runOnce(() ->
                LimelightHelpers.SetFiducialIDFiltersOverride("limeligh-robot", new int[]{})
            )
        );
    }
} // end of RobotContainer