package frc.robot.subsystems.scorer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.PivotConstants;

public class ScorerSubsystem implements IScorer {

  private static ScorerSubsystem instance;
  private IMotor elevatorMotorLeader = new SparkFlexMotor(ElevatorConstants.ID_elevatorLeaderMotor, "ELEVATOR_MASTER");
  private IMotor elevatorMotorFollower = new SparkFlexMotor(ElevatorConstants.ID_elevatorFollowerMotor,
      "ELEVATOR_FOLLOWER");

  private final IMotor pivotMotor = new SparkMAXMotor(PivotConstants.ID_pivotMotor, true, "PIVOT");
  private final IMotor endEffectorMotor = new SparkMAXMotor(EndEffectorConstants.ID_endEffectorMotor, "END_EFFECTOR");

  private boolean hasCoral = false;
  private double previousVelocity = 0;
  private boolean elevatorHasHomed = false;
  private double goalElevator = 0;
  private double goalPivot = 0;

  @Logged(name = "State", importance = Importance.INFO)
  private String state = "START";

  @Logged(name = "Target Branch Height", importance = Importance.INFO)
  private String branchHeightTarget = "NONE";

  @Logged(name = "Target Reef Face To Remove Algae", importance = Importance.INFO)
  private String reefFaceTarget = "NONE";

  private boolean manualControl = true;

  public static ScorerSubsystem getInstance() {
    if (instance == null) {
      instance = new ScorerSubsystem();
    }
    return instance;
  }

  private ScorerSubsystem() {
    setConfigsElevator();
    setConfigsPivot();
    setConfigsEndEffector();
  }

  private void setConfigsElevator() {
    elevatorMotorLeader.setMotorBrake(true);
    elevatorMotorFollower.setMotorBrake(true);
    elevatorMotorLeader.setLoopRampRate(0.5);
    elevatorMotorFollower.setLoopRampRate(0.5);
    elevatorMotorFollower.setFollower(ElevatorConstants.ID_elevatorLeaderMotor, true);
    elevatorMotorLeader.setPositionFactor(ElevatorConstants.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS);
    elevatorMotorLeader.configureMotionProfiling(
        ElevatorConstants.tunning_values_elevator.PID.P,
        ElevatorConstants.tunning_values_elevator.PID.I,
        ElevatorConstants.tunning_values_elevator.PID.D,
        ElevatorConstants.tunning_values_elevator.PID.arbFF,
        ElevatorConstants.tunning_values_elevator.MAX_VELOCITY,
        ElevatorConstants.tunning_values_elevator.MAX_ACCELERATION,
        ElevatorConstants.tunning_values_elevator.POSITION_ERROR_ALLOWED);
    elevatorMotorFollower.burnFlash();
  }

  private void setConfigsPivot() {
    pivotMotor.setMotorBrake(true);
    pivotMotor.setLoopRampRate(0.2);
    pivotMotor.setPositionFactor(PivotConstants.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_DEGREES);
    pivotMotor.configureMotionProfiling(
        PivotConstants.tunning_values_pivot.PID.P,
        PivotConstants.tunning_values_pivot.PID.I,
        PivotConstants.tunning_values_pivot.PID.D,
        PivotConstants.tunning_values_pivot.PID.arbFF,
        PivotConstants.tunning_values_pivot.MAX_VELOCITY,
        PivotConstants.tunning_values_pivot.MAX_ACCELERATION,
        PivotConstants.tunning_values_pivot.POSITION_ERROR_ALLOWED);
    pivotMotor.burnFlash();
  }

  private void setConfigsEndEffector() {
    endEffectorMotor.setMotorBrake(true);
    endEffectorMotor.setLoopRampRate(0.1);
    endEffectorMotor.setVelocityFactor(EndEffectorConstants.VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM);
    endEffectorMotor.burnFlash();
  }

  @Override
  public void periodic() {
    if (!manualControl) {
      setScorerStructureGoals();
    }
    SmartDashboard.putNumber("Pivot Position", pivotMotor.getPosition());
    SmartDashboard.putNumber("Elevator Position", elevatorMotorLeader.getPosition());
    SmartDashboard.putNumber("EndEffector Velocity", endEffectorMotor.getVelocity());
    SmartDashboard.putString("state", state);
  }

  private void setScorerStructureGoals() {
    if (goalElevator > elevatorMotorLeader.getPosition()) {
      if (pivotSecureForElevator()) {
        elevatorMotorLeader.setPositionReferenceMotionProfiling(limitGoalElevator(goalElevator),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        pivotMotor.setPositionReferenceMotionProfiling(limitGoalPivot(goalPivot),
            PivotConstants.tunning_values_pivot.PID.arbFF);
      } else {
        elevatorMotorLeader.setPositionReferenceMotionProfiling(elevatorMotorLeader.getPosition(),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        pivotMotor.setPositionReferenceMotionProfiling(limitGoalPivot(goalPivot),
            PivotConstants.tunning_values_pivot.PID.arbFF);
      }
    } else {
      if (!elevatorSecureForPivot()
          && goalPivot > PivotConstants.tunning_values_pivot.setpoints.UNSECURE_POSITON_FOR_ROTATION_WITH_ELEVATOR_UP) {
        elevatorMotorLeader.setPositionReferenceMotionProfiling(limitGoalElevator(goalElevator),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        pivotMotor.setPositionReferenceMotionProfiling(pivotMotor.getPosition(),
            PivotConstants.tunning_values_pivot.PID.arbFF);
      } else {
        elevatorMotorLeader.setPositionReferenceMotionProfiling(limitGoalElevator(goalElevator),
            ElevatorConstants.tunning_values_elevator.PID.arbFF);
        pivotMotor.setPositionReferenceMotionProfiling(limitGoalPivot(goalPivot),
            PivotConstants.tunning_values_pivot.PID.arbFF);
      }
    }
  }

  public boolean isRobotAbleToScore() {
    return false;
  }

  public void runCoralIntakeDetection() {
    if (endEffectorMotor.getVelocity() < previousVelocity
        - EndEffectorConstants.tunning_values_endeffector.VELOCITY_FALL_FOR_INTAKE_DETECTION) {
      hasCoral = true;
    }
    previousVelocity = endEffectorMotor.getVelocity();
  }

  @Override
  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void intakeFromHP() {
    runCoralIntakeDetection();
    state = "INTAKING_FROM_HP";
  }

  @Override
  public void prepareToPlaceCoralOnBranch() {

  }

  @Override
  public void removeAlgaeFromBranch(Pose3d reefFaceToRemove) {

  }

  @Override
  public void moveScorerToDefaultPosition() {
    goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT;
    goalPivot = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE;
    state = "DEFAULT";
  }

  @Override
  public void homeElevator() {
    if (!elevatorHasHomed) {
      elevatorMotorLeader.set(0.1);
      this.state = "HOMING_ELEVATOR";
    } else if (Math.abs(elevatorMotorLeader.getVelocity()) < 0.1) {
      this.state = "ELEVATOR_HOMED";
      elevatorMotorLeader.set(0);
      elevatorMotorLeader.setPosition(0);
      elevatorHasHomed = true;
    }
  }

  @Override
  public void placeCoral() {
    hasCoral = false;
    elevatorHasHomed = false;
    this.state = "PLACING_CORAL";
  }

  @Override
  public boolean isSecuredToPlaceCoral() {
    return false;
  }

  @Override
  public boolean hasPlaced() {
    return false;
  }

  private double limitGoalElevator(double goal) {
    if (goal >= ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT) {
      return ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT;
    } else if (goal <= ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT) {
      return ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT;
    } else {
      return goal;
    }
  }

  private double limitGoalPivot(double goal) {
    if (goal > PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE;
    } else if (goal < PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE;
    } else {
      return goal;
    }
  }

  @Override
  public void setElevatorTestPosition(double testPosition) {
    goalElevator = testPosition;
    this.state = "ELEVATOR_TEST_POSITION_" + testPosition;
  }

  @Override
  public void setPivotTestPosition(double testPosition) {
    goalPivot = testPosition;
    this.state = "PIVOT_TEST_POSITION_" + testPosition;
  }

  private boolean pivotSecureForElevator() {
    return pivotMotor.getPosition() > PivotConstants.tunning_values_pivot.setpoints.SECURE_FOR_ELEVATOR_UP;
  }

  private boolean elevatorSecureForPivot() {
    return elevatorMotorLeader
        .getPosition() < ElevatorConstants.tunning_values_elevator.setpoints.SECURE_FOR_PIVOT_ROTATION;
  }

  @Override
  public void setCoastScorer() {
    elevatorMotorLeader.setMotorBrake(false);
    elevatorMotorFollower.setMotorBrake(false);
    pivotMotor.setMotorBrake(false);
  }

  @Override
  public void setBrakeScorer() {
    elevatorMotorLeader.setMotorBrake(true);
    elevatorMotorFollower.setMotorBrake(true);
    pivotMotor.setMotorBrake(true);
  }

  @Override
  public void setElevatorDutyCycle(double dutyCycle) {
    if (elevatorMotorLeader.getPosition() <= ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT
        && dutyCycle > 0) {
      elevatorMotorLeader.set(dutyCycle);
    } else if (elevatorMotorLeader.getPosition() >= ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT
        && dutyCycle < 0) {
      elevatorMotorLeader.set(dutyCycle);
    } else {
      elevatorMotorLeader.set(0);
    }
  }

  @Override
  public void setPivotDutyCycle(double dutyCycle) {
    if (pivotMotor.getPosition() <= PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE
        && dutyCycle > 0) {
      pivotMotor.set(dutyCycle);
    } else if (pivotMotor.getPosition() >= PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE
        && dutyCycle < 0) {
      pivotMotor.set(dutyCycle);
    } else {
      pivotMotor.set(0);
    }
  }

  @Override
  public void setEndEffectorDutyCycle(double dutyCycle) {
    if (dutyCycle > 0) {
      pivotMotor.set(dutyCycle);
    } else {
      pivotMotor.set(0);
    }
  }
}
