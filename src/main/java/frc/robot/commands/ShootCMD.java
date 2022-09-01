// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ShootCMD extends CommandBase {

    private ShooterSubsystem Shooter;
    private CollectorSubsystem Collector;
    private boolean timeRecorded = false;
    private double startTime = 0;
    private double setPoint;

    public Constants constants = new Constants();
    private CANSparkMax leftSpinnyBoi;
    private CANSparkMax rightSpinnyBoi;
    private SparkMaxPIDController leftShootController;
    private SparkMaxPIDController rightShootController;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private double shootPower = 0.0;
    private double collectorSpeed;
    private double kSpinupRadPerSec;


    SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(
      constants.SHOOTER_LEFT_ksVolts,
      constants.SHOOTER_LEFT_kvVoltSecondsPerMeter,
      constants.SHOOTER_LEFT_kaVoltSecondsPerMeter);

    SimpleMotorFeedforward RightFeedforward = new SimpleMotorFeedforward(
      constants.SHOOTER_RIGHT_kaVoltSecondsPerMeter,
      constants.SHOOTER_RIGHT_ksVolts,
      constants.SHOOTER_LEFT_kvVoltSecondsPerMeter);

    public ShootCMD(ShooterSubsystem Shooter, CollectorSubsystem collector, double RPM, double collectorSpeed) {
        this.Shooter = Shooter;
        this.setPoint = RPM;
        this.kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(this.setPoint);
        this.Collector = collector;
        this.collectorSpeed = collectorSpeed;

        this.leftSpinnyBoi = this.Shooter.getLeftSparkMax();
        this.leftShootController = this.Shooter.getLeftSparkMax().getPIDController();
        this.leftEncoder = this.Shooter.getLeftSparkMax().getEncoder();

        this.rightSpinnyBoi = this.Shooter.getRightSparkMax();
        this.rightShootController = this.Shooter.getRightSparkMax().getPIDController();
        this.rightEncoder = this.Shooter.getRightSparkMax().getEncoder();

        addRequirements(Shooter);
        addRequirements(collector);
    }

    public void initialize() {
        timeRecorded = false;
        startTime = 0;
    }

    public void execute() {
        leftShootController.setReference(-setPoint, CANSparkMax.ControlType.kVelocity);
        rightShootController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);

        if (!timeRecorded) {
            timeRecorded = true;
            startTime = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() > startTime + 500) {
            new CollectCnd(collectorSpeed, Collector, false, false);
        }

       
    }

    public void end(boolean interupted) {
        Shooter.setMotors(0);
        Collector.setMotor(0, false);
        timeRecorded = false;
        startTime = 0;
    }

    public boolean isFinished() {
        return false;
    }
}
