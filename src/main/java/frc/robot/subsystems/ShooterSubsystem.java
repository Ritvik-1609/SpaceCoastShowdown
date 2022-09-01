// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Lemonlight;
import frc.robot.Lemonlight;

public class ShooterSubsystem extends SubsystemBase {

  Constants constants = new Constants();
  public CANSparkMax leftSpinnyBoi;
  private CANSparkMax rightSpinnyBoi;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  Lemonlight lemonlight = new Lemonlight();
  double num = 30;

  
  
  public ShooterSubsystem() {
    leftSpinnyBoi = new CANSparkMax(constants.LEFT_SHOOTER_CAN_ID, MotorType.kBrushless);
    rightSpinnyBoi = new CANSparkMax(constants.RIGHT_SHOOTER_CAN_ID, MotorType.kBrushless);

    leftSpinnyBoi.setIdleMode(IdleMode.kCoast);
    rightSpinnyBoi.setIdleMode(IdleMode.kCoast);


    //rightSpinnyBoi.follow(leftSpinnyBoi);
    //rightSpinnyBoi.setInverted(true);

    leftEncoder = leftSpinnyBoi.getEncoder();
    rightEncoder = rightSpinnyBoi.getEncoder();

    REVPhysicsSim.getInstance().addSparkMax(leftSpinnyBoi, DCMotor.getNeo550(1)); // check param
    REVPhysicsSim.getInstance().addSparkMax(rightSpinnyBoi, DCMotor.getNeo550(1)); // check param
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Distance: ", (lemonlight.distanceGrab() / 12));
    SmartDashboard.putNumber("SetPoint: ", lemonlight.VtoRPM());
    SmartDashboard.putNumber("Manual Setpoint: ", constants.manualShootSpeed);
    SmartDashboard.putNumber("Left SparkMax RPM: ", (leftEncoder.getVelocity()));
    SmartDashboard.putNumber("Right SparkMax RPM: ", (rightEncoder.getVelocity()));


  }

  public void setMotors(double speed) {
    leftSpinnyBoi.set(speed);
    //rightSpinnyBoi.set(-speed);
  }

  public void setVoltage(double voltage) {
    leftSpinnyBoi.setVoltage(voltage);
    rightSpinnyBoi.setVoltage(-voltage);
  }

  // public void setReferences(double setPoint, CANSparkMax.ControlType ctrl) {

  //   rightSpinnyBoi.getPIDController().setReference(-setPoint, ctrl);
  //   leftSpinnyBoi.getPIDController().setReference(setPoint, ctrl);

  // }

  // ask about max RPM
  public void setmMotors() {
    leftSpinnyBoi.set(1450 / 5700);
    //rightSpinnyBoi.set(-1450 / 5700);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
   // m_flyWheelPlantSim.setInput((leftSpinnyBoi.getEncoder().getVelocity() / constants.MaxRPM) * RobotController.getBatteryVoltage());
    //m_flyWheelPlantSim.update(.020);
  }

  public CANSparkMax getRightSparkMax() {
    return leftSpinnyBoi;
  }

  public CANSparkMax getLeftSparkMax() {
    return rightSpinnyBoi;
  }

  // public LinearSystemLoop<N1, N1, N1> getLoop(){
  //   return m_loop;
  // }

  // public void setFlyWheel(double RPM){
  //   double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(RPM);

  //   // PID controller.
  //   // We just pressed the trigger, so let's set our next reference
  //   m_loop.setNextR(VecBuilder.fill(kSpinupRadPerSec)); 

  //   // Correct our Kalman filter's state vector estimate with encoder data.
  //   m_loop.correct(
  //       VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(leftSpinnyBoi.getEncoder().getVelocity()))); 
  //   // Update our LQR to generate new voltage commands and use the voltages to predict the next
  //   // state with out Kalman filter.
  //   m_loop.predict(0.020);

  //   // Send the new calculated voltage to the motors.
  //   // voltage = duty cycle * battery voltage, so
  //   // duty cycle = voltage / battery voltage

  //   double nextVoltage = m_loop.getU(0);
  //   //nextVoltage = (nextVoltage/12) * 5700;
  //   num = nextVoltage;
   
    

  //   setReferences((nextVoltage), CANSparkMax.ControlType.kVoltage);



  // }
}
