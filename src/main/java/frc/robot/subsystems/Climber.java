package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final CANSparkMax climberMotor1;
    private final CANSparkMax climberMotor2;
    private final SparkPIDController climber1PidController;
    // private final SparkPIDController shooter2PidController;
    public double kP, kI, kD, kIz, kMaxOutput, kMinOutput, kFF;

    public Climber(){
        //set motor things
        climberMotor1 = new CANSparkMax(97, MotorType.kBrushless);
        climberMotor1.restoreFactoryDefaults();
        climberMotor1.setIdleMode(IdleMode.kBrake);
        climberMotor1.setInverted(false);
        climberMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        climberMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        climberMotor2 = new CANSparkMax(97, MotorType.kBrushless);
        climberMotor2.restoreFactoryDefaults();
        climberMotor2.setIdleMode(IdleMode.kBrake);
        climberMotor2.setInverted(false);
        climberMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        climberMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);
        climberMotor2.follow(climberMotor1, true); //TODO: check

        // PID coefficients
        kP = 7e-5; 
        kI = 0;
        kD = 0; 
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1; 
        kMinOutput = -1;

        //set pid things
        climber1PidController = climberMotor1.getPIDController();
        climber1PidController.setP(kP);
        climber1PidController.setI(kI);
        climber1PidController.setD(kD);
        climber1PidController.setIZone(kIz);
        climber1PidController.setFF(kFF);
        climber1PidController.setOutputRange(kMinOutput, kMaxOutput);
        climber1PidController.setPositionPIDWrappingEnabled(true);

        // shooter2PidController = climberMotor2.getPIDController();
        // shooter2PidController.setP(kP);
        // shooter2PidController.setI(kI);
        // shooter2PidController.setD(kD);
        // shooter2PidController.setIZone(kIz);
        // shooter2PidController.setFF(kFF);
        // shooter2PidController.setOutputRange(kMinOutput, kMaxOutput);
        // shooter2PidController.setPositionPIDWrappingEnabled(true);
    }
    public void spinShooter(double velocity){
        climber1PidController.setReference(velocity, ControlType.kVelocity);
        // shooter2PidController.setReference(velocity, ControlType.kVelocity);
    }
    public void stopShooter(){
        climber1PidController.setReference(0, ControlType.kVelocity);
        // shooter2PidController.setReference(0, ControlType.kVelocity);
    }
}
