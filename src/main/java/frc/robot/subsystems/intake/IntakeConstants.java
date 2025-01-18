package frc.robot.subsystems.intake;

import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.ModeSetter;

public class IntakeConstants {
    
    //intakein alabileceği en düşük açı
public static final double  minAngle = 0.0;

//intakein alabilecği maximum açı
public static final double maxAngle = 35.0;

// Açıları radyan yapmak için soft hale getir
public static final double kSoftLimitReverse = Units.degreesToRadians(minAngle);
public static final double kSoftLimitForward = Units.degreesToRadians(maxAngle);

// intakein başlangıç noktasını tanımla / DEĞİŞTİRİLECEK
public static final Translation2d intakeOrigin = new Translation2d(0, 0);

public static final int coralMotor1ID = 15;

public static final int coralMotor2ID = 10;



public static final boolean coralMotor1Inverted = true;
public static final boolean coralMotor2Inverted = false;

public static final double kIntakeZeroCosineOffset = 0.0;

public static final double intakeLength = Units.inchesToMeters(0.35);

 public static final TrapezoidProfile.Constraints intakeMotionConstraint =
            new TrapezoidProfile.Constraints(2.0, 2.0);


public static final Gains gains = 
    switch (ModeSetter.currentMode){

                case SIM -> new Gains(75, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0);
                case REAL, REPLAY -> new Gains(10.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);

    };

    public record Gains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}

}
