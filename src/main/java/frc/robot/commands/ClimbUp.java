// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbUp extends CommandBase {
  private Climber m_climber;
  public static boolean async_control = false;
  /** Creates a new ClimbUp. */
  public ClimbUp(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(climber);
    m_climber = climber;
    SmartDashboard.putString("Sub Mode", "S");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_driveMode==Constants.DriveMode.CLIMBUP){
      if(m_climber.getRightSelectedPosition()<1){
        if(async_control){
          var leftpower = -MathUtil.applyDeadband(RobotContainer.m_JoystickLeft.getY(), 0.2);
          var rightpower = -MathUtil.applyDeadband(RobotContainer.m_JoystickRight.getY(), 0.2);
          m_climber.setPower(leftpower, rightpower);
        }else{
          var power = -MathUtil.applyDeadband(RobotContainer.m_JoystickLeft.getY(), 0.2);
          m_climber.setPower(power);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public void setAsync(boolean async_control){
    ClimbUp.async_control = async_control;
    if(async_control){
      SmartDashboard.putString("Sub Mode", "A");
    }else{
      SmartDashboard.putString("Sub Mode", "S");
    }
  }

  public boolean getAsync(){
    return async_control;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
