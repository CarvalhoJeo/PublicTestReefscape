package frc.Java_Is_UnderControl.Motors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomIntegerLogger;

public class SparkFlexMotor implements IMotor {

  private SparkFlex motor;

  private SparkFlexConfig config;

  private boolean factoryDefaultOcurred = false;

  private final TrapezoidProfile m_profile;

  private double maxVelocity = 0;

  private double rampRate = Double.NaN;

  private double maxAcceleration = 0;

  private boolean usingAlternateEncoder = false;

  private CustomDoubleLogger appliedOutputLog;
  private CustomDoubleLogger targetOutputLog;
  private CustomDoubleLogger currentLog;
  private CustomDoubleLogger positionLog;
  private CustomDoubleLogger velocityLog;
  private CustomDoubleLogger temperatureLog;
  private CustomIntegerLogger faultsLog;
  private CustomDoubleLogger targetPositionLog;
  private CustomDoubleLogger targetSpeedLog;

  boolean isInverted = false;

  private double targetPercentage;
  private double targetPosition;
  private double targetVelocity;

  private SimpleMotorFeedforward feedforward;

  private double velocityFF;
  private String motorName;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  public Velocity<VoltageUnit> quasistaticVoltagePerSecond = null;
  public Voltage dynamicVoltage = null;
  public Time timeoutSysID = null;

  private SysIdRoutine sysIdRoutine;

  public SparkFlexMotor(int motorID, String motorName) {
    this(motorID, false, motorName);
  }

  public SparkFlexMotor(int motorID, boolean usingAlternateEncoder, String motorName) {
    this.motor = new SparkFlex(motorID, MotorType.kBrushless);
    this.config = new SparkFlexConfig();
    this.m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(this.maxVelocity, this.maxAcceleration));
    this.motorName = motorName;
    this.usingAlternateEncoder = usingAlternateEncoder;
    this.setAlternateEncoder(usingAlternateEncoder);
    this.setupLogs(motorID, usingAlternateEncoder);
    this.updateLogs();

  }

  private void setAlternateEncoder(boolean usingAlternateEncoder) {
    if (usingAlternateEncoder) {
      this.config.closedLoop
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
      this.config.externalEncoder.countsPerRevolution(8192);
    } else {
      this.config.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }
  }

  private void setupLogs(int motorId, boolean isAlternateEncoder) {
    this.appliedOutputLog = new CustomDoubleLogger("/motors/" + motorId + "/appliedOutput");
    this.targetOutputLog = new CustomDoubleLogger("/motors/" + motorId + "/targetOutput");
    this.currentLog = new CustomDoubleLogger("/motors/" + motorId + "/current");
    this.positionLog = new CustomDoubleLogger("/motors/" + motorId + "/position");
    this.velocityLog = new CustomDoubleLogger("/motors/" + motorId + "/velocity");
    this.temperatureLog = new CustomDoubleLogger("/motors/" + motorId + "/temperature");
    this.faultsLog = new CustomIntegerLogger("/motors/" + motorId + "/faults");
    this.targetPositionLog = new CustomDoubleLogger("/motors/" + motorId + "/targetPosition");
    this.targetSpeedLog = new CustomDoubleLogger("/motors/" + motorId + "/targetSpeed");
    StringLogEntry firmwareVersionLog = new StringLogEntry(DataLogManager.getLog(),
        "/motors/" + motorId + "/firmwareVersion");
    firmwareVersionLog.append(this.motor.getFirmwareString());
    BooleanLogEntry isAlternateEncoderLog = new BooleanLogEntry(DataLogManager.getLog(),
        "/motors/" + motorId + "/isAlternateEncoder");
    isAlternateEncoderLog.append(isAlternateEncoder);
    BooleanLogEntry isFollowerLog = new BooleanLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/isFollower");
    isFollowerLog.append(this.motor.isFollower());
    BooleanLogEntry isInvertedLog = new BooleanLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/isInverted");
    isInvertedLog.append(isInverted);
  }

  @Override
  public void updateLogs() {
    this.appliedOutputLog.append(this.motor.getAppliedOutput());
    this.targetOutputLog.append(this.targetPercentage);
    this.currentLog.append(this.motor.getOutputCurrent());
    this.positionLog.append(this.motor.getEncoder().getPosition());
    this.velocityLog.append(this.motor.getEncoder().getVelocity());
    this.temperatureLog.append(this.motor.getMotorTemperature());
    // this.faultsLog.append(this.motor.getFaults());
    this.targetPositionLog.append(this.targetPosition);
    this.targetSpeedLog.append(this.targetVelocity);
  }

  private void configureSparkFlex(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  @Override
  public String getMotorName() {
    return this.motorName;
  }

  // To restore factory default its necessary to wait until revlib 2025 is
  // officially launched
  @Override
  public void factoryDefault() {
    if (!factoryDefaultOcurred) {
    }
    factoryDefaultOcurred = true;
  }

  @Override
  public void clearStickyFaults() {
    configureSparkFlex(() -> motor.clearFaults());
  }

  // To implement the Kg, Ks and Kv its necessary to wait until revlib 2025 is
  // officially launched
  @Override
  public void configureFeedForward(double Kg, double Ks, double Kv) {
    config.closedLoop.velocityFF(Kg);
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F, double Izone) {
    this.configurePIDF(P, I, D, F);
    config.closedLoop.iZone(Izone);
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F) {
    config.closedLoop.pidf(P, I, D, F);
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingMaxInput(maxInput);
    config.closedLoop.positionWrappingMinInput(minInput);
  }

  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    config.idleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setInverted(boolean inverted) {
    this.config.inverted(inverted);

    if (inverted == true) {
      isInverted = true;
    } else {
      isInverted = false;
    }
  }

  @Override
  public void setInvertedEncoder(boolean inverted) {
    config.encoder.inverted(inverted);
  }

  @Override
  public void burnFlash() {
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void set(double percentOutput) {
    if (percentOutput != this.targetPercentage) {
      this.motor.set(percentOutput);
    }
    this.targetPercentage = percentOutput;
    this.targetPosition = Double.NaN;
    this.targetVelocity = Double.NaN;
    this.updateLogs();
  }

  @Override
  public void set(Voltage percentOutput) {
    if (percentOutput.in(Volts) != this.targetPercentage) {
      this.motor.set(percentOutput.in(Volts));
    }
    this.targetPercentage = percentOutput.in(Volts);
    this.targetPosition = Double.NaN;
    this.targetVelocity = Double.NaN;
    this.updateLogs();
  }

  @Override
  public void setPositionReference(double position) {
    if (this.getPosition() != position) {
      motor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    }
    this.targetPercentage = Double.NaN;
    this.targetVelocity = Double.NaN;
    this.targetPosition = position;
    this.updateLogs();
  }

  public void setPositionReferenceArbFF(double position, ClosedLoopSlot feedforward) {
    if (this.getPosition() != position) {
      motor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition, feedforward);
    }
    this.targetPercentage = Double.NaN;
    this.targetVelocity = Double.NaN;
    this.targetPosition = position;
    this.updateLogs();
  }

  @Override
  public void configureMotionProfiling(double P, double I, double D, double ff, double maxVelocity,
      double maxAcceleration,
      double positionErrorAllowed) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    config.closedLoop.maxMotion
        .maxVelocity(maxVelocity, ClosedLoopSlot.kSlot0)
        .maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot0)
        .allowedClosedLoopError(positionErrorAllowed, ClosedLoopSlot.kSlot0);
    config.closedLoop
        .p(P, ClosedLoopSlot.kSlot0)
        .i(I, ClosedLoopSlot.kSlot0)
        .d(D, ClosedLoopSlot.kSlot0)
        .velocityFF(ff, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void configureMotionProfiling(double P, double I, double D, double kS, double kV, double kA,
      double maxVelocity, double maxAcceleration, double jerk) {
    configureMotionProfiling(P, I, D, 0, maxVelocity, maxAcceleration, 0.05);
  }

  public void configureMaxMagic(double P, double I, double D, double S, double V, double A, double maxVelocity,
      double maxAcceleration, double positionErrorAllowed) {
    this.feedforward = new SimpleMotorFeedforward(S, V, A);
    double ff = feedforward.calculate(this.velocityFF);
    config.closedLoop.maxMotion
        .maxVelocity(maxVelocity)
        .maxAcceleration(maxAcceleration)
        .allowedClosedLoopError(positionErrorAllowed);
    config.closedLoop
        .pidf(P, I, D, ff);
  }

  public void setVelocityReference(double velocity, double feedforward) {
    if (this.getVelocity() != velocity) {
      motor.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }
    this.targetPercentage = Double.NaN;
    this.targetVelocity = velocity;
    this.targetPosition = Double.NaN;
    this.updateLogs();
  }

  public void setPositionReferenceMotionProfiling(double position, double arbFF) {
    motor.getClosedLoopController().setReference(position, SparkBase.ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0, arbFF);
  }

  @Override
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  @Override
  public double getDutyCycleSetpoint() {
    return motor.get();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  @Override
  public double getPosition() {
    if (usingAlternateEncoder) {
      return motor.getExternalEncoder().getPosition();

    }
    return motor.getEncoder().getPosition();
  }

  @Override
  public double getVelocity() {
    if (usingAlternateEncoder) {
      return motor.getExternalEncoder().getVelocity();
    }
    return motor.getEncoder().getVelocity();
  }

  @Override
  public void setPositionFactor(double factor) {
    config.encoder.positionConversionFactor(factor);
  }

  @Override
  public void setVelocityFactor(double factor) {
    config.encoder.velocityConversionFactor(factor);
  }

  @Override
  public void setPosition(double position) {
    this.motor.getEncoder().setPosition(position);
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    config.voltageCompensation(nominalVoltage);
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    config.smartCurrentLimit(currentLimit);
  }

  @Override
  public void setLoopRampRate(double rampRate) {
    if (rampRate != this.rampRate) {
      config.closedLoopRampRate(rampRate);
      config.openLoopRampRate(rampRate);
    }
    this.rampRate = rampRate;
  }

  @Override
  public void setFollower(int leaderIDcan, boolean invert) {
    config.follow(leaderIDcan, invert);
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
