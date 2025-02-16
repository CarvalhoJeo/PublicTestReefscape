package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.scorer.IScorer;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class SuperStructure extends SubsystemBase {
  public IScorer scorer;

  private ControlBoard controlBoard = ControlBoard.getInstance();

  private PowerDistribution powerDistributionHub;

  private CustomDoubleLogger batteryVoltageLogEntry = new CustomDoubleLogger("/Robot/BatteryVoltage");

  private CustomDoubleLogger totalCurrentDrawLogEntry = new CustomDoubleLogger("/Robot/TotalCurrentDraw");

  public SuperStructure() {
    this.scorer = ScorerSubsystem.getInstance();
    this.powerDistributionHub = new PowerDistribution();
    this.batteryVoltageLogEntry.append(this.powerDistributionHub.getVoltage());
    this.totalCurrentDrawLogEntry.append(this.powerDistributionHub.getTotalCurrent());
  }

  @Override
  public void periodic() {
    this.scorer.periodic();
    this.batteryVoltageLogEntry.append(this.powerDistributionHub.getVoltage());
    this.totalCurrentDrawLogEntry.append(this.powerDistributionHub.getTotalCurrent());
  }

  // public boolean scorerHasCoral() {
  // return this.scorer.hasCoral();
  // }

  // public boolean intakeHasCoral() {
  // return this.scorer.hasCoral();
  // }

  public void setCoastToRobot() {
    this.scorer.setCoastScorer();
  }

  public void setBrakeToRobot() {
    this.scorer.setBrakeScorer();
  }

  public boolean isRobotAbleToScore() {
    // if (this.controlBoard.getDoNotScore()) {
    return false;
    // }
    // return this.scorer.isPivotAtSetPointForAutoScore()
    // && this.scorer.isElevatorAtSetPointForScoring();
    // }
  }
}
