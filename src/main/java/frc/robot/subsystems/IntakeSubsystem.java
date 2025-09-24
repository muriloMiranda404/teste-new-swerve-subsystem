package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase{
    
    SparkMax turnIntake;
    SparkMax getCoral;

    DutyCycleEncoder encoder;

    PIDController controller;

    DigitalInput coralswitch;
    double setpoint;

    double output;
    boolean outputIsMoreThan0;

    ElevatorSubsystem elevatorSubsystem;

    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private IntakeSubsystem(){

        turnIntake = new SparkMax(Intake.INTAKE_MOTOR, SparkMax.MotorType.kBrushless);
        getCoral = new SparkMax(Intake.CORAL_MOTOR, SparkMax.MotorType.kBrushless);

        encoder = new DutyCycleEncoder(Intake.INTAKE_ENCODER);
        encoder.setDutyCycleRange(0, 360);

        controller = Intake.INTAKE_PID;
        controller.setTolerance(Intake.INTAKE_TOLERANCE);

        coralswitch = new DigitalInput(Intake.CORAL_SWITCH);

        output = 0;
        setpoint = 0;
        this.outputIsMoreThan0 = false;

        elevatorSubsystem = ElevatorSubsystem.getInstance();
    }

    public static IntakeSubsystem getInstance(){
        if(intakeSubsystem == null){
            return new IntakeSubsystem();
        }
        return intakeSubsystem;
    }

    public void setSpeed(double speed){
        getCoral.set(speed);
    }

    public boolean isTouched(){
        return !coralswitch.get();
    }

    public void stopCoralMotor(){
        this.getCoral.stopMotor();
    }

    public void stopIntakeMotor(){
        this.turnIntake.stopMotor();
    }

    public double getSetpoint(){
        return controller.getSetpoint();
    }

    public double getDistance(){
        return encoder.get() * 360.0;
    }
    
    public void setPosition(double setpoint){
        double ang = getDistance();
        this.setpoint = setpoint;

        if(setpoint < 55.0){

            setpoint = 55.0;
        
        } else if(setpoint > 230.0){
         
            setpoint = 230.0;
        
        }

        this.output = controller.calculate(ang, setpoint);
        
        turnIntake.set(output);

        if(output > 0){
            outputIsMoreThan0 = true;
        } else{
            outputIsMoreThan0 = false;
        }
        // System.out.println("setpoint: " + setpoint);
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public boolean OutputIsMoreThan0(){
        return outputIsMoreThan0;
    }

    public double getOutput(){
        return output;
    }

    public void setReference(double valor){
        if(getDistance() != valor){
            turnIntake.getClosedLoopController().setReference(valor, SparkBase.ControlType.kPosition);
        }
    }

    @Override
    public void periodic() {
        // System.out.println("setpoint do intake: " + intakeSubsystem.getSetpoint());
    }
}
