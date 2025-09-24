package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem extends SubsystemBase{
    
    SparkMax rightMotor;
    SparkMax leftMotor;

    DigitalInput downSwitch;
    DigitalInput upSwitch;

    PIDController controller;

    Encoder encoder;

    double output;
    boolean outputIsMoreThan0;

    double setpoint;

    public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public static ElevatorSubsystem getInstance(){
        if(elevatorSubsystem == null){
            return new ElevatorSubsystem();
        }
        return elevatorSubsystem;
    }

    private ElevatorSubsystem(){

        rightMotor = new SparkMax(Elevator.RIGHT_ELEVATOR_MOTOR, SparkMax.MotorType.kBrushless);
        leftMotor = new SparkMax(Elevator.LEFT_ELEVATOR_MOTOR, SparkMax.MotorType.kBrushless);

        downSwitch = new DigitalInput(Elevator.DOWN_SWITCH);
        upSwitch = new DigitalInput(Elevator.UP_SWITCH);

        encoder = new Encoder(Elevator.ENCODER_ELEV_A, Elevator.ENCODER_ELEV_B);
        encoder.setDistancePerPulse(360.0/2048.0);

        controller = Elevator.ELEVATOR_PID;
        controller.setTolerance(Elevator.ELEVATOR_TOLERANCE);

        output = 0;
        setpoint = 0;
        outputIsMoreThan0 = false;
    }

    public double getDistance(){
        return encoder.getDistance();
    }

    public double getSetpoint(){
        return setpoint;
    }

    public void setElevatorPosition(double setpoint){
        double ang = getDistance() * -1;
        this.setpoint = setpoint;

        if(setpoint < 0){
            setpoint = 0.0;
        } else if(setpoint > 1480.0){
            setpoint = 1480.0;
        }

        output = controller.calculate(ang, setpoint);
        
        if(upSwitch.get()){
            if(output > 0){
                output = 0.0;
            }

            if(setpoint > 1480.0){
                setpoint = 1480.0;
            }
        }

        if(downSwitch.get() && output < 0.0){

            if(output < 0){
                output = 0.0;
            }

            if(setpoint < 0){
                setpoint = 0.0;
            }

        }

        rightMotor.set(output);
        leftMotor.set(-output);
    }

    public boolean outputIsMoreThan0(){
        return outputIsMoreThan0;
    }

    public double getOutput(){
        return output;
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public void stopElevator(){
    rightMotor.setVoltage(0);
    leftMotor.setVoltage(0);
   }

   public boolean upSwitchGet(){
    boolean trava = false;

    if(upSwitch.get() && trava == false){
        trava = true;
    } else{
        trava = false;
    }

    return trava;
   }
   
    public void resetEncoder(){
        encoder.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("setpoint do elevador", setpoint);
        // System.out.println("setpoint do elevador: " + setpoint);
    }
}
