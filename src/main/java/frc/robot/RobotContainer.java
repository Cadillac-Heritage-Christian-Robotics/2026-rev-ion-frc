// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.Set;

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
import frc.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final SendableChooser<Boolean> m_useDeferred = new SendableChooser<>();

    private final SendableChooser<String> m_autoLocation = new SendableChooser<>();
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
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Autonomous commands
    // Simple drive forward for about 2 meters for 1 seconds
    private final DriveForwardCommand m_DriveForwardCommand = new DriveForwardCommand(drivetrain);

    public RobotContainer() {
        // Register named commands for PathPlanner
        NamedCommands.registerCommand("StartIntake", m_intake.runIntakeCommandAuton());

        NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {
            SmartDashboard.putString("Command | Intake", "Stop");
            m_intake.setIntakePower(0.0);
            m_intake.setConveyorPower(0.0);
        }));

        NamedCommands.registerCommand("SlapArmDown", m_intake.runSlapDownCommandAuton());
        NamedCommands.registerCommand("SlapArmUp",   m_intake.runSlapUpCommandAuton());
        
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
            () -> DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent(), // automatically mirror auton stuff if we are red
            drivetrain
        );


        // Register your path options
        m_useDeferred.setDefaultOption("Improved", true);
        m_useDeferred.addOption("Existing", false);

        m_autoLocation.setDefaultOption("North", "North");
        m_autoLocation.addOption("South", "South");

        m_autoStrategy.setDefaultOption("Default", "Default");
        m_autoStrategy.addOption("Alpha", "Alpha");
        m_autoStrategy.addOption("Bravo", "Bravo");
        m_autoStrategy.addOption("Default With Action", "DefaultAction");

        // Push it to SmartDashboard so drive team can see it
        SmartDashboard.putData("Drive Backward Strategy", m_useDeferred);
        SmartDashboard.putData("Starting Location", m_autoLocation);
        SmartDashboard.putData("Auton Strategy", m_autoStrategy);

        m_timeToShoot.setDefaultOption("3.0", 3.0);
        m_timeToShoot.addOption("1.0", 1.0);
        m_timeToShoot.addOption("2.0", 2.0);
        m_timeToShoot.addOption("4.0", 4.0);
        m_timeToShoot.addOption("5.0", 5.0);
        m_timeToShoot.addOption("6.0", 6.0);
        m_timeToShoot.addOption("7.0", 7.0);
        m_timeToShoot.addOption("8.0", 8.0);

        m_alignToHub.setDefaultOption("No Shooting Align", false);
        m_alignToHub.addOption("Align Shooting", true);
        SmartDashboard.putData("Align To Hub", m_alignToHub);

        SmartDashboard.putData("Auton Shoot Duration", m_timeToShoot);

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
        
        // Turn wheels in 'x' shape to act as a brake mechanism
        m_driverController.rightTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(drivetrain.applyRequest(() -> brake));


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
            drivetrain.setPoseBootstrapped(false);

            Boolean useDeferred = m_useDeferred.getSelected();

            String selectedLocation = m_autoLocation.getSelected();
            String selectedStrat = m_autoStrategy.getSelected();

            Double timeToShoot = m_timeToShoot.getSelected();

            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

            // It's dumb I know but since Red is "mirror" we have to manually mirror north/south
            if (isRed) {
                switch (selectedLocation) {
                    case "North" -> selectedLocation = "South";
                    case "South" -> selectedLocation = "North";
                }
            }

            String selectedPath = "Blue" + selectedLocation + selectedStrat;

            Boolean usingDefault = selectedStrat.equals("Default");

            System.out.println("Selected Path = " + selectedPath);

            // Manually curated list of starting positions based on with PathPlanner we are using.
            Pose2d targetPosition = AutonUtils.getDesiredPose(selectedPath);
            System.out.println(targetPosition.toString());

            PathPlannerPath path = PathPlannerPath.fromPathFile(selectedPath);

            double offset = -1.0;
            if (isRed) {
                offset = 1.0;
            }

            Command orientBotCommand = orientBotCommand();
            Command driveBackwardCommand = driveBackwardCommand(offset, useDeferred);

            return new SequentialCommandGroup(
                // Step 0: Clear April Tag filters and Bootstrap pose from MegaTag1 so gyro is correct before pathfinding
                Commands.runOnce(() -> LimelightHelpers.SetFiducialIDFiltersOverride("limelight-robot", new int[]{})),
                orientBotCommand,
                // Step 1: Drive back 1 meter to avoid crashing sideways into trench/ramp wall
                driveBackwardCommand,
                // Step 2: Drive to target shooting position
                drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Driving to shoot position")),
                AutoBuilder.pathfindToPoseFlipped(targetPosition, DriveToPoseCommand.CONSTRAINTS),

                // Step 3.1: Optionally align to Hub based on april tags
                // Filters for hub tags, centers the bot on them, clears filters
                drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Optional Alignment")),
                getAlignCommand(),

                // Step 3.2 Shoot preloaded fuel
                drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Shooting")),
                m_shooter.runShooterCommand().withTimeout(timeToShoot).deadlineWith(m_intake.runIntakeCommand()),

                // Step 4: Always run the selected path (intake and slap arm via event markers)
                drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Running Path")),
                AutoBuilder.pathfindThenFollowPath(path, DriveToPoseCommand.CONSTRAINTS),

                // Step 5: Shoot again only if Alpha/Bravo strategy (not default)
                drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Done with path")),
                Boolean.FALSE.equals(usingDefault)
                    ? new SequentialCommandGroup(
                        drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Shooting again")),
                        getAlignCommand(),
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

    public Command orientBotCommand() {
        return Commands.run(() -> {
                    LimelightHelpers.SetRobotOrientation("limelight-robot", drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
                    PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-robot");
                    if (LimelightHelpers.validPoseEstimate(mt1)) {
                        System.out.println("[Localize] Got fix! X=" + mt1.pose.getX() + " Y=" + mt1.pose.getY() + ", Deg=" + mt1.pose.getRotation().getDegrees());
                        drivetrain.resetPose(mt1.pose);
                        drivetrain.setPoseBootstrapped(true);
                    } else {
                        System.out.println("[Localize] No valid estimate yet... tagCount=" + (mt1 != null ? mt1.tagCount : "null"));
                    }
                }, drivetrain).until(() -> 
                    LimelightHelpers.validPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-robot"))
                ).withTimeout(2.0);
    }

    /**
     * We want to drive backward 1 meter.
     * Existing logic that worked in sim and practice field was to simply pathFindToPose
     * 
     * However in real field on Red alliance we were facing the wrong way and drove the wrong way
     * 
     * If we useDeferred in our shuffleboard then we will delay this command until we have a valid pose estimate during auton
     * 
     * This function is merely to offer the option as we have not battle tested it yet
     * @param offset
     * @param useDeferred
     * @return
     */
    public Command driveBackwardCommand(final double offset, final boolean useDeferred) {
        if (useDeferred) {
            return Commands.defer(() -> 
                AutoBuilder.pathfindToPose(new Pose2d(
                    drivetrain.getState().Pose.getX() + offset,
                    drivetrain.getState().Pose.getY(),
                    drivetrain.getState().Pose.getRotation()
                ), DriveToPoseCommand.CONSTRAINTS),
                Set.of(drivetrain)
            );
        } else {
            return  AutoBuilder.pathfindToPose(new Pose2d(drivetrain.getState().Pose.getX() + offset,
                                                          drivetrain.getState().Pose.getY(), 
                                                          drivetrain.getState().Pose.getRotation()
                                                          ), 
                                                DriveToPoseCommand.CONSTRAINTS);
        }
    }

    private Command getAlignCommand() {
        return Boolean.TRUE.equals(m_alignToHub.getSelected()) ? alignToHubCommand() : Commands.none();
    }

    public Command alignToHubCommand() {
        // Determine which tags to use based on alliance
        boolean isRed = DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red;

        int[] hubTags = isRed ? new int[]{9, 10} : new int[]{25, 26};

        return Commands.sequence(
            // Set filter to only see hub center tags
            drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Running Alignment")),

            Commands.runOnce(() -> 
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-robot", hubTags)
            ),

            // Align to hub
            drivetrain.applyRequest(() -> {
                double tx = LimelightHelpers.getTX("limelight-robot");
                SmartDashboard.putNumber("Alignment TX", tx);
                double kP = 0.15; // tune this!
                double rotationRate = -tx * kP;

                return drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationRate);
            })
            .until(() -> 
                LimelightHelpers.getTV("limelight-robot") && // make sure we actually see a tag!
                Math.abs(LimelightHelpers.getTX("limelight-robot")) < 2.0
            )
            .withTimeout(5.0),
            drivetrain.runOnce(() -> SmartDashboard.putString("Auton Phase", "Done with Alignment")),

            // Clear the filter after aligning
            Commands.runOnce(() ->
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight-robot", new int[]{})
            )
        );
    }
} // end of RobotContainer