// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;


public class LiftSubsystem extends SubsystemBase {
  private SparkMax liftR,liftL;
  
  /** Creates a new AlgaeSubsystem. */
  public LiftSubsystem() {
    liftR = new SparkMax(LiftConstants.LiftMotorIDR, MotorType.kBrushless);
    liftL = new SparkMax(LiftConstants.LiftMotorIDL, MotorType.kBrushless);
  }


  public void setLiftSpeed(double LiftSpeed){
    liftL.set(LiftSpeed);
    
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
