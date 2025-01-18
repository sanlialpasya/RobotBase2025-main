package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeIOSpark implements IntakeIO {
  private final SparkMax algaeMotor1;
  private final SparkMax algaeMotor2;
  private final SparkMax coralMotor1;
  private final SparkMax coralMotor2;
 
  
  private final SparkMaxConfig algeaMotor1Config;
  private final SparkMaxConfig algeaMotor2Config;
  private final SparkMaxConfig coralMotor1Config;
  private final SparkMaxConfig coralMotor2Config;
 
  
  private final SparkMaxConfig config;


  public IntakeIOSpark() {
    // find actual motor IDs/ these are RANDOM numbers
    algaeMotor1 = new SparkMax(17, MotorType.kBrushless);
    algaeMotor2 = new SparkMax(27, MotorType.kBrushless);
    coralMotor1 = new SparkMax(15, MotorType.kBrushless);
    coralMotor2 = new SparkMax(10, MotorType.kBrushless);
   

    // ask about gear ratios for all motors
    

    algeaMotor1Config
       .smartCurrentLimit(15)
       .idleMode(IdleMode.kBrake)
       .voltageCompensation(10.0);

    algeaMotor2Config
        .smartCurrentLimit(15)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(10.0);
  
    
    coralMotor1Config
        .smartCurrentLimit(15)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(10.0);


    coralMotor2Config
        .smartCurrentLimit(15)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(10.0);
    
  }

 
  @Override
  public void setAlgaeVoltage(double voltage) {
    //to take algea they should turn the opposite sides
    algaeMotor1.setVoltage(voltage);
    algaeMotor2.setVoltage(-voltage);
  }

  @Override
  public void setCoralIntakeVoltage(double voltage) {
    //to take the corals they should turn the opposite sides
    coralMotor1.setVoltage(voltage);
    coralMotor2.setVoltage(-voltage);
  }



   @Override
    public void configurePID(double kP, double kI, double kD) {
        /*
         * Configure the closed loop controller. We want to make sure we set the feedback sensor
         * as the primary encoder. Also, we apply the given PID constants and set output ranges.
         */
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for velocity control in slot 0
                .p(kP)
                .i(kI)
                .d(kD)
                // velocityFF is a feedforward for velocity mode; here just an example value
                .velocityFF(1.0 / 5767)
                // Limit output from -1 to 1 (full reverse to full forward)
                .outputRange(-1, 1);
    }
}
