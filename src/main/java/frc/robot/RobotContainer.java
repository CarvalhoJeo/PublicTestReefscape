// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.joysticks.ControlBoard;
import frc.robot.joysticks.OperatorController;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private ControlBoard driverController = ControlBoard.getInstance();
  private OperatorController operatorPanel = OperatorController.getInstance();

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final SuperStructure superStructure = new SuperStructure();

  public RobotContainer() {
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
  }

  private void configureBindings() {

    driverController.a()
        .whileTrue(Commands.runEnd(() -> superStructure.scorer.setPivotDutyCycle(0.5),
            () -> superStructure.scorer.setPivotDutyCycle(0.0), superStructure));

    driverController.b()
        .whileTrue(Commands.runEnd(() -> superStructure.scorer.setElevatorTestPosition(1),
            () -> superStructure.scorer.setElevatorDutyCycle(0.0), superStructure));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
