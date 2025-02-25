// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;


public class AlgaeSubsystem extends SubsystemBase {
  private SparkMax pivot,roller;
  
  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
    pivot = new SparkMax(AlgaeConstants.PivotID, MotorType.kBrushless);
    roller = new SparkMax(AlgaeConstants.ARollerID, MotorType.kBrushless);
  }


  public void setPivotSpeed(double pivotSpeed){
    pivot.set(pivotSpeed);
    
  }

  public double  setRollerSpeed(double intakeSpeed){
    roller.set(intakeSpeed);
    return intakeSpeed;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
