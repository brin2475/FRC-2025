package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private SparkMax mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    
   

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

         
        /* Angle Motor Config */
        SparkMaxConfig swerveAngleConfig = new SparkMaxConfig();

        // Motor Inverts and Neutral Mode
        swerveAngleConfig.inverted(true)
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

        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleMotor.configure(swerveAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

      

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = optimize(desiredState, getState().angle);

        // Set the angle motor to the desired position
        SparkClosedLoopController anglePIDController = mAngleMotor.getClosedLoopController();
        anglePIDController.setReference(desiredState.angle.getDegrees(), SparkMax.ControlType.kPosition);

        // Set the speed of the drive motor
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute() {
            // Calculate the absolute position using the CANcoder
            double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations(); 
        
            // Set the angle motor's position to the absolute position
            SparkClosedLoopController anglePIDController = mAngleMotor.getClosedLoopController(); 
            anglePIDController.setReference(Math.IEEEremainder(absolutePosition, 1.0) * 360.0, SparkMax.ControlType.kPosition); 
        }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference),
                getCANcoder()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference),
                getCANcoder()
        );
    }

    public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }
}