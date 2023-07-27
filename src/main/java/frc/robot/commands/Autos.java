// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public final class Autos {

  private static HolonomicDriveController m_controller = new HolonomicDriveController(
    new PIDController(1, 0, 0), new PIDController(1, 0, 0),
    new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(6.28, 3.14)));
  
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase follow_trajectory(Constants.AutoMode nPath, 
                                              DriveTrain driveTrain, 
                                              Transit transit){
    switch(nPath){
      case PATH_BOTTOM:
        return Commands.sequence(
          Commands.runOnce(()->transit.armDown()),
          Commands.deadline(
            new SwerveControllerCommand(PathPlan.t101(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
            new UpLift(transit)),
          Commands.runOnce(()->transit.armUp()),
          new Shoot(transit),
          Commands.deadline(
            new SwerveControllerCommand(PathPlan.t102(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
            new UpLift(transit)),
          Commands.deadline(new Detrude(transit), Commands.waitSeconds(3)),
          Commands.runOnce(()->driveTrain.drive(0,0,0,false), driveTrain));


      case PATH_BOTTOM_MIDDLE_HOMING:
        return Commands.sequence(
          Commands.runOnce(()->transit.armDown()),
          Commands.deadline(
            new SwerveControllerCommand(PathPlan.t105_1(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
            new UpLift(transit)),
          new Shoot(transit)

        );


      case PATH_MIDDLE_BOTTOM:
        return Commands.sequence(
          Commands.runOnce(()->transit.armDown()),
          Commands.deadline(
            new SwerveControllerCommand(PathPlan.t202(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
            new UpLift(transit)),
          Commands.runOnce(()->transit.armUp()),
          new Shoot(transit),
          Commands.runOnce(()->driveTrain.drive(0,0,0,false), driveTrain));

      case PATH_MIDDLE_HOMING://Start at middle, Shoot, Go to terminal, Shoot
        return Commands.sequence(
          Commands.deadline(
            new SwerveControllerCommand(PathPlan.t203_1(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
            new UpLift(transit)),
          new Shoot(transit),
          Commands.runOnce(()->transit.armDown()),
          Commands.deadline(
            new SwerveControllerCommand(PathPlan.t203_2(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
            new UpLift(transit)),
          Commands.deadline(new UpLift(transit), Commands.waitSeconds(3)),
          Commands.deadline(
            new SwerveControllerCommand(PathPlan.t203_3(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
            new UpLift(transit)),
          Commands.runOnce(()->transit.armUp()),
          new Shoot(transit),
          Commands.runOnce(()->driveTrain.drive(0,0,0,false), driveTrain));

        case PATH_TOP:
          return Commands.sequence(
            Commands.runOnce(()->transit.armDown()),
            Commands.deadline(
              new SwerveControllerCommand(PathPlan.t302(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
              new UpLift(transit)),
            new Shoot(transit),
            Commands.runOnce(()->driveTrain.drive(0,0,0,false), driveTrain));

        case PATH_TOP_MIDDLE:
          return Commands.sequence(
            Commands.runOnce(()->transit.armDown()),
            Commands.deadline(
              new SwerveControllerCommand(PathPlan.t302_1_1(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
              new UpLift(transit)),
            new Shoot(transit),
            Commands.deadline(
              new SwerveControllerCommand(PathPlan.t302_1_2(), driveTrain::getPose2d, driveTrain.m_kinematics, m_controller, driveTrain::setModuleStates, driveTrain), 
              new UpLift(transit)),
            new Shoot(transit),
            Commands.runOnce(()->driveTrain.drive(0,0,0,false), driveTrain));



        default:
        return Commands.none();
    }
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
