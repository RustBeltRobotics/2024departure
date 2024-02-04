package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final CANSparkMax shooterMotor1;
    private final CANSparkMax shooterMotor2;
    private final SparkPIDController shooter1PidController;
    // private final SparkPIDController shooter2PidController;
    public double kP, kI, kD, kIz, kMaxOutput, kMinOutput, kFF;

    public Shooter(){
        //set motor things
        shooterMotor1 = new CANSparkMax(97, MotorType.kBrushless);
        shooterMotor1.restoreFactoryDefaults();
        shooterMotor1.setIdleMode(IdleMode.kBrake);
        shooterMotor1.setInverted(false);
        shooterMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        shooterMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        shooterMotor2 = new CANSparkMax(97, MotorType.kBrushless);
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor2.setIdleMode(IdleMode.kBrake);
        shooterMotor2.setInverted(false);
        shooterMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        shooterMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        shooterMotor2.follow(shooterMotor1, true); //TODO: check

        // PID coefficients
        kP = 7e-5; 
        kI = 0;
        kD = 0; 
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1; 
        kMinOutput = -1;

        //set pid things
        shooter1PidController = shooterMotor1.getPIDController();
        shooter1PidController.setP(kP);
        shooter1PidController.setI(kI);
        shooter1PidController.setD(kD);
        shooter1PidController.setIZone(kIz);
        shooter1PidController.setFF(kFF);
        shooter1PidController.setOutputRange(kMinOutput, kMaxOutput);
        shooter1PidController.setPositionPIDWrappingEnabled(true);

        // shooter2PidController = shooterMotor2.getPIDController();
        // shooter2PidController.setP(kP);
        // shooter2PidController.setI(kI);
        // shooter2PidController.setD(kD);
        // shooter2PidController.setIZone(kIz);
        // shooter2PidController.setFF(kFF);
        // shooter2PidController.setOutputRange(kMinOutput, kMaxOutput);
        // shooter2PidController.setPositionPIDWrappingEnabled(true);
    }
    public void spinShooter(double velocity){
        shooter1PidController.setReference(velocity, ControlType.kVelocity);
        // shooter2PidController.setReference(velocity, ControlType.kVelocity);
    }
    public void stopShooter(){
        shooter1PidController.setReference(0, ControlType.kVelocity);
        // shooter2PidController.setReference(0, ControlType.kVelocity);
    }
}
