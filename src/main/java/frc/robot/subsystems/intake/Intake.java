package frc.robot.subsystems.intake;

import static frc.robot.subsystems.arm.ArmConstants.gains;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Gains/kP", gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/Gains/kI", gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Gains/kD", gains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Gains/kS", gains.ffkS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/Gains/kV", gains.ffkV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/Gains/kA", gains.ffkA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/Gains/kG", gains.ffkG());


  IntakeIO io;
  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

 

  public Intake(IntakeIO io) {
    this.io = io;
  }
  

  public void setAlgaeVoltage(double voltage) {
    io.setAlgaeVoltage(voltage);
  }

  public void setCoralIntakeVoltage(double voltage) {
    io.setCoralIntakeVoltage(voltage);
  }

  private double targetPosition = 0.0;
  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.577, 0.0);

  public void setWristPositionDegrees(double position) {
    double targetPosition = Math.toRadians(position);
  }

  public double getTargetWristPosition() {
    return targetPosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public double getCoralWristIntakeCurrent() {
    return inputs.coralWristCurrent;
  }

  public double getWristPosition() {
    return inputs.coralWristPosition;
  }

  public void wristAngle(double position) {
    io.wristAngle(position);
  }

  public void setWristVoltage(double voltage) {
    System.out.println(getWristPosition());
    io.setWristVoltage(voltage);
  }

  public void resetAngle(double radians) {}
}
