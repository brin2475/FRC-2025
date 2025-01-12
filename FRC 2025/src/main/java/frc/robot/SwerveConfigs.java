package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public final class SwerveConfigs {
    public SparkMaxConfig swerveAngleConfig = new SparkMaxConfig();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public SwerveConfigs(){

        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;



        // Swerve Angle Motor Configurations 
        SparkMaxConfig swerveAngleConfig = new SparkMaxConfig();

        // Motor Inverts and Neutral Mode 
        swerveAngleConfig.inverted(false)
                        .idleMode(IdleMode.kBrake);

        // Gear Ratio and Wrapping Config 
        swerveAngleConfig.encoder.positionConversionFactor(Constants.Swerve.angleGearRatio)
                                .velocityConversionFactor(Constants.Swerve.angleGearRatio);
                                swerveAngleConfig.closedLoop.positionWrappingEnabled(true);

        // Current Limiting 
        swerveAngleConfig.smartCurrentLimit(Constants.Swerve.angleCurrentLimit);

        // PID Config 
        swerveAngleConfig.closedLoop.pid(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
        swerveAngleConfig.openLoopRampRate(Constants.Swerve.openLoopRamp);

        swerveAngleConfig.closedLoopRampRate(Constants.Swerve.closedLoopRamp);



        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
    }
}