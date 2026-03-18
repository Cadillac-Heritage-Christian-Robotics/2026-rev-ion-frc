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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final SendableChooser<Boolean> m_autoDefault = new SendableChooser<>();

    private final SendableChooser<String> m_autoLocation = new SendableChooser<>();
    private final SendableChooser<String> m_autoColor = new SendableChooser<>();
    private final SendableChooser<String> m_autoStrategy = new SendableChooser<>();


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
            () -> DriverStation.getAlliance()
                    .filter(a -> a == Alliance.Red)
                    .isPresent(),               // flip paths for red alliance
            drivetrain
        );

        // Register named commands for PathPlanner
        NamedCommands.registerCommand("StartIntake",  m_intake.runIntakeCommand());
        NamedCommands.registerCommand("StopIntake",   m_intake.runOnce(() -> {}));
        NamedCommands.registerCommand("SlapArmUp",    m_intake.runSlapUpCommand().withTimeout(0.5)); // TODO add stop logic
        NamedCommands.registerCommand("SlapArmDown",  m_intake.runSlapDownCommand().withTimeout(0.5)); // TODO add stop logic
        NamedCommands.registerCommand("StartShoot",   m_shooter.runShooterCommand());
        NamedCommands.registerCommand("StopShoot",    m_shooter.runOnce(() -> {}));

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

        m_autoDefault.setDefaultOption("True", true);
        m_autoDefault.setDefaultOption("False", false);
        SmartDashboard.putData("Override Auton Strategy", m_autoDefault);

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
    m_operatorController.a().toggleOnTrue(m_intake.runSlapUpCommand().withTimeout(0.3));
    m_operatorController.b().toggleOnTrue(m_intake.runSlapDownCommand().withTimeout(0.25));

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

            String selectedPath = selectedColor + selectedLocation + selectedStrat;

            if (autoDefault == true) {
                System.out.println("Overriding path with default path");
                selectedPath = selectedColor + selectedLocation + "Default";
            }

            System.out.println("Selected Path = " + selectedPath);

            // Manually curated list of starting positions based on with PathPlanner we are using.
            Pose2d startingPosition = AutonUtils.getStartingPose(selectedPath);

            PathPlannerPath path = PathPlannerPath.fromPathFile(selectedPath);

            // Step 1: Wait a moment for Limelight to get a confident pose fix (one time only)
            Command waitForPose = new WaitCommand(0.25);

            // Repeating loop: drive to start, shoot, reload
            Command loop = new SequentialCommandGroup(

                // Step 2: Pathfind to the shoot position (AutonUtils already knows Blue vs Red!)
                new DriveToPoseCommand(startingPosition),

                // Step 3: Shoot for a fixed time
                m_shooter.runShooterCommand().withTimeout(3.0), // TODO calibrate for time to shoot 8 ammo

                // Step 4: Then re-run the Path loop
                // Takes into account raising and lower of slap arm over Ramp and activating intake in neutral zone!
                AutoBuilder.followPath(path)

            ).repeatedly();

            return new SequentialCommandGroup(waitForPose, loop);

        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
            return m_DriveForwardCommand;
        }
    }
} // end of RobotContainer