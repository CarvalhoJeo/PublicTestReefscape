package frc.Java_Is_UnderControl.Motors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Java_Is_UnderControl.Driver.Phoenix6Util;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomIntegerLogger;

public class TalonFXMotor implements IMotor {
  private TalonFX motor;

  private GravityTypeValue gravityType;

  private final MotionMagicVoltage m_angleVoltageSetter = new MotionMagicVoltage(0);

  private final VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0);

  private TalonFXConfiguration configuration = new TalonFXConfiguration();

  private double targetPosition = Double.NaN;

  private double targetVelocity = Double.NaN;

  private double targetOutput = Double.NaN;

  private boolean factoryDefaultOcurred = false;

  private MotionMagicVoltage magicVoltage = new MotionMagicVoltage(0);

  private CustomDoubleLogger appliedOutputLog;

  private CustomDoubleLogger targetOutputLog;

  private CustomDoubleLogger currentLog;

  private CustomDoubleLogger positionLog;

  private CustomDoubleLogger velocityLog;

  private CustomDoubleLogger temperatureLog;

  private CustomIntegerLogger faultsLog;

  private CustomDoubleLogger targetPositionLog;

  private CustomDoubleLogger targetSpeedLog;

  private MotorOutputConfigs configs = new MotorOutputConfigs();

  private TalonFXConfigurator cfg;

  private String motorName;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  public Velocity<VoltageUnit> quasistaticVoltagePerSecond = null;
  public Voltage dynamicVoltage = null;
  public Time timeoutSysID = null;

  private SysIdRoutine sysIdRoutine;

  public TalonFXMotor(int id, GravityTypeValue gravityType, String motorName) {
    this.gravityType = gravityType;
    motor = new TalonFX(id);
    this.factoryDefault();
    this.clearStickyFaults();
    this.setCurrentLimit(80);
    this.setupLogs(id);
    this.updateLogs();
    this.factoryDefault();
    this.clearStickyFaults();
    this.motorName = motorName;
  }

  public TalonFXMotor(int id, GravityTypeValue gravityType, String motorName, String canivoreBus) {
    this.gravityType = gravityType;
    motor = new TalonFX(id, canivoreBus);
    this.factoryDefault();
    this.clearStickyFaults();
    this.setCurrentLimit(80);
    this.setupLogs(id);
    this.updateLogs();
    this.factoryDefault();
    this.clearStickyFaults();
    this.motorName = motorName;
  }

  public TalonFXMotor(int id, String motorName, String canivoreBus) {
    this(id, GravityTypeValue.Elevator_Static, motorName, canivoreBus);
  }

  public TalonFXMotor(int id, String motorName) {
    this(id, GravityTypeValue.Elevator_Static, motorName);
  }

  private void setupLogs(int motorId) {
    this.appliedOutputLog = new CustomDoubleLogger("/motors/" + motorId + "/appliedOutput");
    this.targetOutputLog = new CustomDoubleLogger("/motors/" + motorId + "/targetOutput");
    this.currentLog = new CustomDoubleLogger("/motors/" + motorId + "/current");
    this.positionLog = new CustomDoubleLogger("/motors/" + motorId + "/position");
    this.velocityLog = new CustomDoubleLogger("/motors/" + motorId + "/velocity");
    this.temperatureLog = new CustomDoubleLogger("/motors/" + motorId + "/temperature");
    this.faultsLog = new CustomIntegerLogger("/motors/" + motorId + "/faults");
    this.targetPositionLog = new CustomDoubleLogger("/motors/" + motorId + "/targetPosition");
    this.targetSpeedLog = new CustomDoubleLogger("/motors/" + motorId + "/targetSpeed");
    StringLogEntry descriptionLog = new StringLogEntry(DataLogManager.getLog(),
        "/motors/" + motorId + "/Description");
    descriptionLog.append(this.motor.getDescription());
    BooleanLogEntry isInvertedLog = new BooleanLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/isInverted");
    isInvertedLog.append(isInverted());
  }

  @Override
  public void updateLogs() {
    this.appliedOutputLog.append(this.motor.getDutyCycle().getValueAsDouble());
    this.currentLog.append(this.motor.getStatorCurrent().getValueAsDouble());
    this.positionLog.append(Units.rotationsToDegrees(this.motor.getPosition().getValueAsDouble()));
    this.velocityLog.append(this.motor.getVelocity().getValueAsDouble());
    this.temperatureLog.append(this.motor.getDeviceTemp().getValueAsDouble());
    this.faultsLog.append(this.motor.getFaultField().getValue());
    this.targetPositionLog.append(this.targetPosition);
    this.targetSpeedLog.append(this.targetVelocity);
  }

  @Override
  public String getMotorName() {
    return this.motorName;
  }

  @Override
  public void factoryDefault() {
    if (!factoryDefaultOcurred) {
      configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      Phoenix6Util.checkErrorAndRetry(() -> motor.getConfigurator().apply(configuration, 100));
      m_angleVoltageSetter.UpdateFreqHz = 0;
      m_velocityVoltageSetter.UpdateFreqHz = 0;
    }
    factoryDefaultOcurred = true;
  }

  @Override
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1       Applied Motor Output, Fault Information, Limit Switch
   *                         Information
   * @param CANStatus2       Selected Sensor Position (PID 0), Selected Sensor
   *                         Velocity (PID 0), Brushed Supply Current
   *                         Measurement, Sticky Fault Information
   * @param CANStatus3       Quadrature Information
   * @param CANStatus4       Analog Input, Supply Battery Voltage, Controller
   *                         Temperature
   * @param CANStatus8       Pulse Width Information
   * @param CANStatus10      Motion Profiling/Motion Magic Information
   * @param CANStatus12      Selected Sensor Position (Aux PID 1), Selected Sensor
   *                         Velocity (Aux PID 1)
   * @param CANStatus13      PID0 (Primary PID) Information
   * @param CANStatus14      PID1 (Auxiliary PID) Information
   * @param CANStatus21      Integrated Sensor Position (Talon FX), Integrated
   *                         Sensor Velocity (Talon FX)
   * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless
   *                         Stator Current Measurement
   */
  public void configureCANStatusFrames(
      int CANStatus1,
      int CANStatus2,
      int CANStatus3,
      int CANStatus4,
      int CANStatus8,
      int CANStatus10,
      int CANStatus12,
      int CANStatus13,
      int CANStatus14,
      int CANStatus21,
      int CANStatusCurrent) {
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
    // CANStatus2);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
    // CANStatus3);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
    // CANStatus4);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
    // CANStatus8);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets,
    // CANStatus10);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
    // CANStatus12);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,
    // CANStatus13);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1,
    // CANStatus14);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated,
    // CANStatus21);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current,
    // CANStatusCurrent);

    // TODO: Configure Status Frame 2 thru 21 if necessary
    // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html#setting-status-frame-periods
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F, double Izone) {
    configurePIDF(P, I, D, F);
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F) {
    this.cfg = motor.getConfigurator();
    cfg.refresh(configuration.Slot0);
    cfg.apply(
        configuration.Slot0.withKP(P).withKI(I).withKD(D).withKS(F).withGravityType(gravityType));
  }

  @Override
  public void configureFeedForward(double Kg, double Ks, double Kv) {
    this.cfg = motor.getConfigurator();
    cfg.refresh(configuration.Slot0);
    cfg.apply(
        configuration.Slot0.withKG(Kg).withKS(Ks).withKV(Kv));
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    this.cfg = motor.getConfigurator();
    cfg.refresh(configuration.ClosedLoopGeneral);
    configuration.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.apply(configuration.ClosedLoopGeneral);
  }

  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    motor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setInverted(boolean isInverted) {
    cfg.refresh(configuration.MotorOutput);
    configuration.MotorOutput.withInverted(
        isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
    cfg.apply(configuration.MotorOutput);
  }

  @Override
  public void setInvertedEncoder(boolean inverted) {
    // do nothing
  }

  @Override
  public void burnFlash() {
    // do nothing
  }

  @Override
  public void setFollower(int leaderIDcan, boolean invert) {
    motor.setControl(new Follower(leaderIDcan, invert));
  }

  @Override
  public void set(double percentOutput) {
    targetOutput = percentOutput;
    targetPosition = Double.NaN;
    targetVelocity = Double.NaN;
    motor.set(percentOutput);
  }

  @Override
  public void set(Voltage percentOutput) {
    targetOutput = percentOutput.in(Volts);
    targetPosition = Double.NaN;
    targetVelocity = Double.NaN;
    motor.set(percentOutput.in(Volts));
  }

  @Override
  public void setPositionReference(double position) {
    setPositionReferenceArbFF(position, 0);
  }

  public void setPositionReferenceArbFF(double position, double feedforward) {
    double positionInRotations = Units.degreesToRotations(position);
    targetPosition = position;
    targetOutput = Double.NaN;
    targetVelocity = Double.NaN;
    motor.setControl(new PositionDutyCycle(positionInRotations).withFeedForward(feedforward));
  }

  @Override
  public void configureMotionProfiling(double P, double I, double D, double ff, double maxVelocity,
      double maxAcceleration, double positionErrorAllowed) {
    cfg.refresh(configuration.Slot0);
    configuration.Slot0.withKP(P).withKI(I).withKD(D);
    var motionMagicConfigs = configuration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = maxVelocity;
    motionMagicConfigs.MotionMagicAcceleration = maxAcceleration;
    cfg.apply(configuration);
  }

  @Override
  public void configureMotionProfiling(double P, double I, double D, double kS, double kV, double kA,
      double maxVelocity,
      double maxAcceleration, double jerk) {
    cfg.refresh(configuration.Slot0);
    configuration.Slot0.withKP(P).withKI(I).withKD(D).withKS(kS).withKV(kV).withKA(kA);
    var motionMagicConfigs = configuration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = maxVelocity;
    motionMagicConfigs.MotionMagicAcceleration = maxAcceleration;
    motionMagicConfigs.MotionMagicJerk = jerk;
    cfg.apply(configuration);
  }

  public void setPositionReferenceMotionProfiling(double position, double arbFF) {
    motor.setControl(magicVoltage.withPosition(position));
  }

  public void setVelocityReference(double velocity, double feedforward) {
    targetVelocity = velocity;
    targetOutput = Double.NaN;
    targetPosition = Double.NaN;
    motor.setControl(new VelocityVoltage(velocity).withFeedForward(feedforward));
  }

  @Override
  public double getVoltage() {
    return motor.getMotorVoltage().getValueAsDouble();
  }

  public double getDutyCycleSetpoint() {
    return motor.get();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getAppliedOutput() {
    return motor.getDutyCycle().getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Override
  public double getPosition() {
    return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  @Override
  public void setPositionFactor(double factor) {
    this.cfg = motor.getConfigurator();
    cfg.refresh(configuration.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(factor));
    cfg.apply(configuration);
  }

  @Override
  public void setVelocityFactor(double factor) {
    this.cfg = motor.getConfigurator();
    cfg.refresh(configuration.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(factor));
    cfg.apply(configuration);
  }

  @Override
  public void setPosition(double position) {
    motor.setPosition(position);
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    // Do not implement
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    this.cfg = motor.getConfigurator();
    cfg.refresh(configuration.CurrentLimits);
    cfg.apply(
        configuration.CurrentLimits.withStatorCurrentLimit(currentLimit)
            .withStatorCurrentLimitEnable(true));
  }

  @Override
  public void setLoopRampRate(double rampRate) {
    this.cfg = motor.getConfigurator();
    cfg.refresh(configuration.ClosedLoopRamps);
    cfg.apply(configuration.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(rampRate));
  }

  private boolean isInverted() {
    motor.getConfigurator().refresh(configs);
    if (configs.Inverted == InvertedValue.Clockwise_Positive) {
      return false;
    } else {
      return true;
    }
  }

  @Override
  public Object getMotor() {
    return motor;
  }

  @Override
  public void configureSysID(double quasistaticVoltagePerSecond, double dynamicVoltage, double timeoutSysID) {
    this.quasistaticVoltagePerSecond = Volts.of(quasistaticVoltagePerSecond).per(Second);
    this.dynamicVoltage = Volts.of(dynamicVoltage);
    this.timeoutSysID = Seconds.of(timeoutSysID);
  }

  @Override
  public void setSysID(Subsystem currentSubsystem) {
    this.sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(this.quasistaticVoltagePerSecond, this.dynamicVoltage, this.timeoutSysID),
        new SysIdRoutine.Mechanism(
            voltage -> {
              this.set(voltage);
            },
            log -> {
              log.motor(this.motorName)
                  .voltage(m_appliedVoltage.mut_replace(this.getAppliedOutput() * RobotController.getBatteryVoltage(),
                      Volts))
                  .linearPosition(m_distance.mut_replace(this.getPosition(), Meters))
                  .linearVelocity(m_velocity.mut_replace(this.getVelocity(), MetersPerSecond));
            },
            currentSubsystem));
  }

  @Override
  public void setTwoSysIDMotors(Subsystem currentSubsystem, IMotor otherMotor) {
    this.sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(this.quasistaticVoltagePerSecond, this.dynamicVoltage, this.timeoutSysID),
        new SysIdRoutine.Mechanism(
            voltage -> {
              this.set(voltage);
              otherMotor.set(voltage);
            },
            log -> {
              log.motor(this.motorName)
                  .voltage(m_appliedVoltage.mut_replace(this.getAppliedOutput() * RobotController.getBatteryVoltage(),
                      Volts))
                  .linearPosition(m_distance.mut_replace(this.getPosition(), Meters))
                  .linearVelocity(m_velocity.mut_replace(this.getVelocity(), MetersPerSecond));

              log.motor(otherMotor.getMotorName())
                  .voltage(m_appliedVoltage
                      .mut_replace(otherMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(otherMotor.getPosition(), Meters))
                  .linearVelocity(m_velocity.mut_replace(otherMotor.getVelocity(), MetersPerSecond));
            },
            currentSubsystem));
  }

  @Override
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.sysIdRoutine.quasistatic(direction);
  }

  @Override
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.sysIdRoutine.dynamic(direction);
  }

}
