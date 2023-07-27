// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  // TalonFX climberLeft = new TalonFX(12);
  TalonFX climberRight = new TalonFX(13);
  /** Creates a new Climber. */
  public Climber() {}

  public void setPower(double power){
    // climberLeft.set(ControlMode.PercentOutput, -power);
    climberRight.set(ControlMode.PercentOutput, power);
  }
  
  public void setPower(double leftpower, double rightpower){
    // climberLeft.set(ControlMode.PercentOutput, -leftpower);
    climberRight.set(ControlMode.PercentOutput, rightpower);
  }

  public double getRightSelectedPosition(){
    return climberRight.getSelectedSensorPosition();
  }

  // public double getLeftSelectedPosition(){
  //   return climberLeft.getSelectedSensorPosition();
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
