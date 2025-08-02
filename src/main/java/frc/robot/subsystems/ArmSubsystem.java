package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.*;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.units.measure.*;

public class ArmSubsystem extends SubsystemBase {

  // Feedforward
  private final ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS,ArmConstants.kG,ArmConstants.kV,ArmConstants.kA);

  // Motor control variables
  private final TalonFX motor  = new TalonFX(ArmConstants.canID);
  private final TalonFXSimState motorSim; 
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Simulation and visualization related variables
  private final SingleJointedArmSim armSim;
  private final Mechanism2d mech2d = new Mechanism2d(1.0, 1.0);
  private final MechanismRoot2d root = mech2d.getRoot("ArmRoot", 0.5, 0.1);
  private final MechanismLigament2d armLigament = root.append(new MechanismLigament2d("Arm", ArmConstants.armLength, 90));
  private final MotionMagicVoltage armMotionMagicControl;

  // Constructor to initialize everything
  public ArmSubsystem() {
    
    motorSim = motor.getSimState(); //gets the simulation state of the motor
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    armMotionMagicControl = new MotionMagicVoltage(0);

    // Configure motor parameters (PID, current limits, etc.)
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = ArmConstants.kP;
    slot0.kI = ArmConstants.kI;
    slot0.kD = ArmConstants.kD;

    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = ArmConstants.statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = ArmConstants.enableStatorLimit;
    currentLimits.SupplyCurrentLimit = ArmConstants.supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = ArmConstants.enableSupplyLimit;

    config.Feedback.SensorToMechanismRatio = ArmConstants.gearRatio;
    config.MotionMagic.MotionMagicCruiseVelocity = 10.0;
    config.MotionMagic.MotionMagicAcceleration = 20.0;

    config.MotorOutput.NeutralMode = ArmConstants.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast; // Set brake mode
    // config.Feedback.SensorToMechanismRatio = gearRatio; // Set gear ratio for the motor encoder

    motor.getConfigurator().apply(config);
    // Initialize motor position to 0
    motor.setPosition(0);
    
    // Initialize the arm simulation
    armSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1),
      ArmConstants.gearRatio,
      SingleJointedArmSim.estimateMOI(ArmConstants.armLength, 5),
      ArmConstants.armLength,
      Units.degreesToRadians(ArmConstants.minAngleDeg),
      Units.degreesToRadians(ArmConstants.maxAngleDeg),
      true,
      Units.degreesToRadians(0)
    );

    armLigament.setLength(ArmConstants.armLength);

    SmartDashboard.putData("Arm Visualization", mech2d);
  }
  // This method is called periodically to update any information on the SmartDashboard
  @Override
  public void periodic() {
  
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal, statorCurrentSignal, temperatureSignal);
    SmartDashboard.putNumber("Arm/Position (rot)", motor.getPosition().getValueAsDouble());

    //SmartDashboard.putNumber("Arm/Position (rot)", getPosition());
    SmartDashboard.putNumber("Arm/Velocity (rps)", getVelocity());
    SmartDashboard.putNumber("Arm/Voltage", getVoltage());
    SmartDashboard.putNumber("Arm/Current", getCurrent());
    SmartDashboard.putNumber("Arm/Temperature", getTemperature());
  }

  // This method is called periodically for simulation
  @Override
  public void simulationPeriodic() {

    SmartDashboard.putNumber("Sim/TargetRotations", armMotionMagicControl.Position);
    SmartDashboard.putNumber("Sim/SimAngleDeg", Units.radiansToDegrees(armSim.getAngleRads()));
    SmartDashboard.putNumber("Sim/MotorVoltage", motorSim.getMotorVoltage());

    motorSim.setSupplyVoltage(12);

    armSim.setInput(motorSim.getMotorVoltage());
    armSim.update(0.02);

    motorSim.setRawRotorPosition(
        (armSim.getAngleRads() - Math.toRadians(ArmConstants.minAngleDeg))
            * ArmConstants.gearRatio
            / (2.0
            * Math.PI));

    motorSim.setRotorVelocity(
      armSim.getVelocityRadPerSec() * ArmConstants.gearRatio / (2.0 * Math.PI));

    armLigament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }
  
  // Method to get current position of the arm (in rotations)
  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }


  // Method to get current velocity of the arm (in rotations per second)
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  // Method to get current voltage applied to the motor
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  // Method to get current motor current
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  // Method to get current motor temperature
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

   // Method to convert position to radians (for more precision in calculation)
  public double getPositionRadians() {
    return getPosition() * 2.0 * Math.PI;
  }

  // Method to set the arm position (in degrees)
  public void setPosition(double position) {
    double rotations = Math.toRadians(position) / (2.0 * Math.PI);
    armMotionMagicControl.Slot = 0;
    armMotionMagicControl.Position = rotations;
    motor.setControl(armMotionMagicControl);
  }

  // Method to set the arm velocity (in degrees per second)
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

   // Method to set the arm velocity with specified acceleration (in degrees per second)
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    double currentDeg = Units.radiansToDegrees(getPositionRadians());
    if ((currentDeg >= ArmConstants.maxAngleDeg && velocityDegPerSec > 0) ||
        (currentDeg <= ArmConstants.minAngleDeg && velocityDegPerSec < 0)) {
      velocityDegPerSec = 0;
    }
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);
    double ffVolts = feedforward.calculate(getVelocity(), acceleration);
    motor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
  }

  // Method to set the motor voltage directly
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  // Command to set the arm angle to a specific value
  /*
   * 
   * A lambda expression is a short block of code which takes in parameters and returns a value. Lambda expressions are similar to methods,
   * but they do not need a name and they can be implemented right in the body of a method.
   */
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setPosition(angleDegrees));
  }

  // Command to stop the arm by setting velocity to 0
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  // Command to move the arm at a specific velocity
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(velocityDegPerSec));
  }
}
