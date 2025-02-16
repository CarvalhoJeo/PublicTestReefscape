package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard implements IDriverController, IOperatorController {
  private static ControlBoard mInstance = null;

  public static ControlBoard getInstance() {
    if (mInstance == null) {
      mInstance = new ControlBoard();
    }

    return mInstance;
  }

  IDriverController mDriverController;
  IOperatorController mOperatorController;

  private ControlBoard() {
    mDriverController = DriverController.getInstance();
    mOperatorController = OperatorController.getInstance();
  }

  @Override
  public double getXtranslation() {
    return mDriverController.getXtranslation();
  }

  @Override
  public double getYtranslation() {
    return mDriverController.getYtranslation();
  }

  @Override
  public double getCOS_Joystick() {
    return mDriverController.getCOS_Joystick();
  }

  @Override
  public double getSIN_Joystick() {
    return mDriverController.getSIN_Joystick();
  }

  @Override
  public boolean turboActivate() {
    return mDriverController.turboActivate();
  }

  @Override
  public boolean notUsingJoystick() {
    return mDriverController.notUsingJoystick();
  }

  @Override
  public Trigger a() {
    return mDriverController.a();
  }

  @Override
  public Trigger y() {
    return mDriverController.y();
  }

  @Override
  public Trigger x() {
    return mDriverController.x();
  }

  @Override
  public Trigger b() {
    return mDriverController.b();
  }

  @Override
  public boolean rotateLeft() {
    return mDriverController.rotateLeft();
  }

  @Override
  public boolean rotateRight() {
    return mDriverController.rotateRight();
  }

  @Override
  public Trigger resetGyro() {
    return mDriverController.resetGyro();
  }

  @Override
  public boolean isForcingDriverControl() {
    return mDriverController.isForcingDriverControl();
  }

  public Object getDoNotScore() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDoNotScore'");
  }

}
