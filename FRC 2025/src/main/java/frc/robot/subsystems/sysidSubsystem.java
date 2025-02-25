// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

public class sysidSubsystem extends SubsystemBase {
    /** Creates a new sysidSubsystem. */
    private final VoltageOut m_voltReq;
    private final SysIdRoutine sysIdRoutine;
    private final TalonFX motor0;
    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFX motor3;

    public sysidSubsystem() {
        m_voltReq = new VoltageOut(0.0);
        motor0 = new TalonFX(Constants.Swerve.Mod0.driveMotorID);
        motor1 = new TalonFX(Constants.Swerve.Mod1.driveMotorID);
        motor2 = new TalonFX(Constants.Swerve.Mod2.driveMotorID);
        motor3 = new TalonFX(Constants.Swerve.Mod3.driveMotorID);
        SignalLogger.setPath("/media/sda1/");

       

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(4),
                        null,
                        (state) -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (Volts) -> {
                            motor0.setControl(m_voltReq.withOutput(Volts.in(Volt)));
                            motor1.setControl(m_voltReq.withOutput(Volts.in(Volt)));
                            motor2.setControl(m_voltReq.withOutput(Volts.in(Volt)));
                            motor3.setControl(m_voltReq.withOutput(Volts.in(Volt)));
                        },
                        null,
                        this
                )
        );
    }

    public void startLogging() {
        SignalLogger.start();
        System.out.println("Logger started");
    }

    public void stopLogging() {
        SignalLogger.stop();
        System.out.println("Logger stopped");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}