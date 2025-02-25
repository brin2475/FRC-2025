// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;



public class CoralSubsystem extends SubsystemBase {
  private SparkMax elbow, wrist, roller ;
  
  /** Creates a new AlgaeSubsystem. */
  public CoralSubsystem() {
    elbow = new SparkMax(CoralConstants.ElbowID, MotorType.kBrushless);
    wrist = new SparkMax(CoralConstants.WristID, MotorType.kBrushless);
    roller = new SparkMax(CoralConstants.CRollerID, MotorType.kBrushless);

  }


  public void setRollerSpeed(double RollerSpeed){
    roller.set(RollerSpeed);
    
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
