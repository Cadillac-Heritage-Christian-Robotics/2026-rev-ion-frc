// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;

public class IntakeSubsystem extends SubsystemBase {
  final TalonFXConfiguration conveyorConfig = new TalonFXConfiguration();

  // Initialize intake SPARK. We will use open loop control for this. 
  private SparkFlex intakeMotor =
     new SparkFlex(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  private SparkFlex slapMotor =
   new SparkFlex(IntakeSubsystemConstants.kSlapMotorCanId, MotorType.kBrushless);

  // Initialize conveyor SPARK. We will use open loop control for this.
  //  private TalonFX conveyorMotor =
  //     new TalonFX(20); 
    private TalonFX conveyorMotor =
     new TalonFX(IntakeSubsystemConstants.kConveyorMotorCanId);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    intakeMotor.configure(
        Configs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
          
    slapMotor.configure(
      Configs.IntakeSubsystem.intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    // System.out.println("---> IntakeSubsystem initialized");
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  private void setSlapPower(double power) {
    slapMotor.set(power);
  }  

  /** Set the conveyor motor power in the range of [-1, 1]. */
   private void setConveyorPower(double power) {
     conveyorMotor.set(power);
   }

  /**
   * Command to run the intake and conveyor motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kIntake);
          this.setConveyorPower(ConveyorSetpoints.kExtake);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Intaking");
  }

  public Command runSlapCommand() {
    return this.startEnd(
      () -> {
        this.setSlapPower(ArmSetpoints.kLevel1);
      }, () -> {
        this.setSlapPower(0.0);
      }).withName(Upward);
    }

  /**
   * Command to reverse the intake motor and coveyor motors. When the command is interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command runExtakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kExtake);
          this.setConveyorPower(ConveyorSetpoints.kIntake);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Extaking");
  }

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Intake | Intake | Applied Output", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake | Conveyor | Applied Output", conveyorMotor.getConfigurator().apply(conveyorConfig).value);
  }

}
