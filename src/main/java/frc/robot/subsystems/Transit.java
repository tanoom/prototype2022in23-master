// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transit extends SubsystemBase {
  
  TalonSRX intakeC = new TalonSRX(Constants.Intake3);
  TalonSRX intakeB = new TalonSRX(Constants.Intake2);
  TalonSRX intakeD = new TalonSRX(Constants.Intake4);
  TalonFX shooter = new TalonFX(Constants.Shooter);
  TalonFX intakeA = new TalonFX(Constants.Intake1);
  Solenoid arm = new Solenoid(Constants.PneumaticHub, PneumaticsModuleType.REVPH, 4);
  private boolean solenoid_down = true;
  private boolean isShooting = false;
  private boolean isIntaking = false;
  public double setShooterSpeed = 0;
  public int inUse = 0;
  public static double ShooterSpeed = Constants.ShooterMaxSpeed;

  /** Creates a new Transit. */
  public Transit() {
    intakeD.configVoltageCompSaturation(12);
    intakeD.enableVoltageCompensation(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean shoot(double speed) {
    setShooterSpeed = speed;
    shooter.set(ControlMode.Velocity, speed);
    var curSpeed = shooter.getSelectedSensorVelocity();
    return Math.abs((speed-curSpeed))<Constants.ShooterSpeedError;
  }

  public void stopShoot() {
    setShooterSpeed = 0;
    shooter.set(ControlMode.PercentOutput, 0);
  }

  public void setLiftPower(){
    if(isIntaking){
      if(solenoid_down){
        intakeA.set(ControlMode.PercentOutput, -0.9);
      }else{
        intakeA.set(ControlMode.PercentOutput, 0);
      }
      intakeB.set(ControlMode.PercentOutput, -0.7);
    }
    intakeC.set(ControlMode.PercentOutput, -0.7);
    if(isShooting&&!isIntaking)
      intakeD.set(ControlMode.PercentOutput, 0.3);
    else
      intakeD.set(ControlMode.PercentOutput, -0.2);
  }

  public void setLiftPower(double power){
    if(isIntaking){
      if(solenoid_down){
        intakeA.set(ControlMode.PercentOutput, -power);
      }else{
        intakeA.set(ControlMode.PercentOutput, 0);
      }
      intakeB.set(ControlMode.PercentOutput, -power);
    }
    intakeC.set(ControlMode.PercentOutput, -power);
    if(isShooting&&!isIntaking)
      intakeD.set(ControlMode.PercentOutput, power);
    else
      intakeD.set(ControlMode.PercentOutput, -power);
  }

  public void setLiftPower(double powerA, double powerB, double powerC, double powerD){
    if(isIntaking){
      if(solenoid_down){
        intakeA.set(ControlMode.PercentOutput, -powerA);
      }else{
        intakeA.set(ControlMode.PercentOutput, 0);
      }
      intakeB.set(ControlMode.PercentOutput, -powerB);
    }
    intakeC.set(ControlMode.PercentOutput, -powerC);
    intakeD.set(ControlMode.PercentOutput, powerD);
  }

  public void stopLift(){
    intakeB.set(ControlMode.PercentOutput, 0.0);
    intakeC.set(ControlMode.PercentOutput, 0.0);
    intakeD.set(ControlMode.PercentOutput, 0.0);
    intakeA.set(ControlMode.PercentOutput, 0);
  }

  public void setShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }

  public void setIntaking(boolean isIntaking) {
    this.isIntaking = isIntaking;
  }

  public void armDown(){
    arm.set(true);
    solenoid_down = true;
  }

  public void armUp(){
    arm.set(false);
    solenoid_down = false;
  }

  public double getShooterSpeed(){
    return shooter.getSelectedSensorVelocity();
  }
}
