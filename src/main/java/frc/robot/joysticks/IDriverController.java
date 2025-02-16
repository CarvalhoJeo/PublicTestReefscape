package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDriverController {
  double getXtranslation();

  double getYtranslation();

  double getCOS_Joystick();

  double getSIN_Joystick();

  boolean turboActivate();

  boolean notUsingJoystick();

  Trigger y();

  Trigger b();

  Trigger a();

  Trigger x();

  boolean rotateLeft();

  boolean rotateRight();

  Trigger resetGyro();

  boolean isForcingDriverControl();
}
