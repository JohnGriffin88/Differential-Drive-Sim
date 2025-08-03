package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.configs.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.units.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSubsystem extends SubsystemBase {

  // Core motor and sim setup
  private final TalonFX armMotor = new TalonFX(ArmConstants.canID);
  private final TalonFXSimState simMotorState = armMotor.getSimState();

  // Status signals
  private final StatusSignal<Angle> angleStatus = armMotor.getPosition();
  private final StatusSignal<AngularVelocity> speedStatus = armMotor.getVelocity();
  private final StatusSignal<Voltage> voltageStatus = armMotor.getMotorVoltage();
  private final StatusSignal<Current> currentStatus = armMotor.getStatorCurrent();
  private final StatusSignal<Temperature> tempStatus = armMotor.getDeviceTemp();

  // Control modes
  private final PositionVoltage positionCtrl = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage velocityCtrl = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionCtrl = new MotionMagicVoltage(0);

  // Feedforward model
  private final ArmFeedforward armFF = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  // Simulation
  private final SingleJointedArmSim simArm = new SingleJointedArmSim(
    DCMotor.getKrakenX60(1),
    ArmConstants.gearRatio,
    SingleJointedArmSim.estimateMOI(ArmConstants.armLength, 5),
    ArmConstants.armLength,
    Units.degreesToRadians(ArmConstants.minAngleDeg),
    Units.degreesToRadians(ArmConstants.maxAngleDeg),
    true,
    Units.degreesToRadians(0)
  );

  // Visuals
  private final Mechanism2d visual = new Mechanism2d(1.0, 1.0);
  private final MechanismRoot2d root = visual.getRoot("Pivot", 0.5, 0.1);
  private final MechanismLigament2d visualArm = root.append(new MechanismLigament2d("ArmBar", ArmConstants.armLength, 90));

  public ArmSubsystem() {
    // Configuration
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = ArmConstants.kP;
    config.Slot0.kI = ArmConstants.kI;
    config.Slot0.kD = ArmConstants.kD;

    config.CurrentLimits.StatorCurrentLimit = ArmConstants.statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = ArmConstants.enableStatorLimit;
    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = ArmConstants.enableSupplyLimit;

    config.Feedback.SensorToMechanismRatio = ArmConstants.gearRatio;

    config.MotionMagic.MotionMagicCruiseVelocity = 10.0;
    config.MotionMagic.MotionMagicAcceleration = 20.0;

    config.MotorOutput.NeutralMode = ArmConstants.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    armMotor.getConfigurator().apply(config);
    armMotor.setPosition(0);

    SmartDashboard.putData("Arm Visual", visual);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(angleStatus, speedStatus, voltageStatus, currentStatus, tempStatus);

    SmartDashboard.putNumber("Arm/Position", getPosition());
    SmartDashboard.putNumber("Arm/Velocity", getVelocity());
    SmartDashboard.putNumber("Arm/Voltage", getVoltage());
    SmartDashboard.putNumber("Arm/Current", getCurrent());
    SmartDashboard.putNumber("Arm/Temperature", getTemperature());
  }

  @Override
  public void simulationPeriodic() {
    simMotorState.setSupplyVoltage(12);

    simArm.setInput(simMotorState.getMotorVoltage());
    simArm.update(0.02);

    simMotorState.setRawRotorPosition(
      (simArm.getAngleRads() - Math.toRadians(ArmConstants.minAngleDeg)) *
      ArmConstants.gearRatio / (2.0 * Math.PI)
    );
    simMotorState.setRotorVelocity(
      simArm.getVelocityRadPerSec() * ArmConstants.gearRatio / (2.0 * Math.PI)
    );

    visualArm.setAngle(Units.radiansToDegrees(simArm.getAngleRads()));
    SmartDashboard.putNumber("Sim/AngleDeg", Units.radiansToDegrees(simArm.getAngleRads()));
  }

  public double getPosition() {
    return angleStatus.getValueAsDouble();
  }

  public double getVelocity() {
    return speedStatus.getValueAsDouble();
  }

  public double getVoltage() {
    return voltageStatus.getValueAsDouble();
  }

  public double getCurrent() {
    return currentStatus.getValueAsDouble();
  }

  public double getTemperature() {
    return tempStatus.getValueAsDouble();
  }

  public void moveToAngle(double targetAngleDeg) {
    double targetRot = Math.toRadians(targetAngleDeg) / (2.0 * Math.PI);
    motionCtrl.Position = targetRot;
    armMotor.setControl(motionCtrl);
  }

  public void runAtVelocity(double degPerSec, double accel) {
    double currentDeg = Units.radiansToDegrees(getPosition() * 2.0 * Math.PI);

    if ((currentDeg >= ArmConstants.maxAngleDeg && degPerSec > 0) ||
        (currentDeg <= ArmConstants.minAngleDeg && degPerSec < 0)) {
      degPerSec = 0;
    }

    double rotPerSec = Units.degreesToRadians(degPerSec) / (2.0 * Math.PI);
    double ff = armFF.calculate(getVelocity(), accel);

    armMotor.setControl(velocityCtrl.withVelocity(rotPerSec).withFeedForward(ff));
  }

  public void setRawVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  // ---- Commands ---- //

  public Command moveToAngleCommand(double degrees) {
    return runOnce(() -> moveToAngle(degrees));
  }

  public Command stopArmCommand() {
    return runOnce(() -> runAtVelocity(0, 0));
  }

  public Command runVelocityCommand(double degreesPerSecond) {
    return run(() -> runAtVelocity(degreesPerSecond, 0));
  }
}
