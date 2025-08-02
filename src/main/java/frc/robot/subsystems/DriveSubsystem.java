// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final double kNEOMaxRPM = 5676;
    private final double kWheelDiameterInches = 6;
    private final double kDrivetrainGearRatio = 10.71;

    private final DifferentialDrivetrainSim m_driveSim;

    private final DifferentialDriveOdometry m_odometry;

    private final SparkMaxSim m_leftMotorSim;
    private final SparkMaxSim m_rightMotorSim;

    StructPublisher<Pose2d> m_publisher;

    // TODO: Insert your drive motors and differential drive here...
    private final SparkMax m_rightMotorFollower;
    private final SparkMax m_rightMotorLeader;
    private final SparkMax m_leftMotorFollower;
    private final SparkMax m_leftMotorLeader;
    private final DifferentialDrive m_drive;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
                KitbotMotor.kDoubleNEOPerSide,
                KitbotGearing.k10p71,
                KitbotWheelSize.kSixInch,
                null);

        m_odometry = new DifferentialDriveOdometry(
                new Rotation2d(),
                m_driveSim.getLeftPositionMeters(),
                m_driveSim.getRightPositionMeters());

        m_publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

        // TODO: Instantiate motors & differential drive, then configure motors here...

        m_leftLeader = new SparkMax(DriveConstants.kLeftLeaderMotorPort, MotorType.kBrushless);
        m_leftFollower = new SparkMax(DriveConstants.kLeftFollowerMotorPort, MotorType.kBrushless);
        m_rightLeader = new SparkMax(DriveConstants.kRightLeaderMotorPort, MotorType.kBrushless);
        m_rightFollower = new SparkMax(DriveConstants.kRightFollowerMotorPort, MotorType.kBrushless);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        globalConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        leftLeaderConfig.apply(globalConfig);
        leftFollowerConfig.apply(globalConfig).follow(m_leftLeader);
        rightLeaderConfig.apply(globalConfig).inverted(true);
        rightFollowerConfig.apply(globalConfig).follow(m_rightLeader);

        m_leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

        m_leftMotorSim = new SparkMaxSim(m_leftLeaderMotor, DCMotor.getNEO(2));
        m_rightMotorSim = new SparkMaxSim(m_rightLeaderMotor, DCMotor.getNEO(2));
    }

    // TODO: Insert your arcadeDrive method here...

    public void arcadeDrive(double forward, double rotation) {
        m_drive.arcadeDrive(forward, rotation);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        m_driveSim.update(0.02);

        m_leftMotorSim.iterate(
                (((kNEOMaxRPM * m_leftLeaderMotor.get()) / kDrivetrainGearRatio) * Math.PI
                        * kWheelDiameterInches) / 60,
                RoboRioSim.getVInVoltage(), 0.02);
        m_rightMotorSim.iterate(
                (((kNEOMaxRPM * m_rightLeaderMotor.get()) / kDrivetrainGearRatio) * Math.PI
                        * kWheelDiameterInches) / 60,
                RoboRioSim.getVInVoltage(), 0.02);

        m_leftMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());
        m_rightMotorSim.setBusVoltage(RoboRioSim.getVInVoltage());

        m_leftMotorSim.setAppliedOutput(m_leftLeaderMotor.getAppliedOutput());
        m_rightMotorSim.setAppliedOutput(m_rightLeaderMotor.getAppliedOutput());

        m_driveSim.setInputs(m_leftMotorSim.getAppliedOutput() * m_leftMotorSim.getBusVoltage(),
                m_rightMotorSim.getAppliedOutput() * m_rightMotorSim.getBusVoltage());

        m_odometry.update(
                m_driveSim.getHeading(),
                m_leftMotorSim.getRelativeEncoderSim().getPosition(),
                m_rightMotorSim.getRelativeEncoderSim().getPosition());

        m_publisher.set(m_odometry.getPoseMeters());
    }
}
