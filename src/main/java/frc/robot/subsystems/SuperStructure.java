package frc.robot.subsystems;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.ElevatorPositions;
import frc.robot.Constants.Intake.IntakePositions;

public class SuperStructure extends SubsystemBase{
    
    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LedSubsystem ledSubsystem;

    private double elevatorInput, intakeInput;

    public static SuperStructure mInstance = null;
    private Color color;

    private SuperStructure(){
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.ledSubsystem = LedSubsystem.getInstance();
        this.elevatorInput = ElevatorPositions.HOME;
        this.intakeInput = IntakePositions.DEFAULT_POSITION;
        this.color = new Color();
    }

    public static SuperStructure getInstance(){
        if(mInstance == null){
            mInstance = new SuperStructure();
        }
        return mInstance;
    }

    @Override
    public void periodic() {
        this.elevatorSubsystem.periodic();
        this.intakeSubsystem.periodic();
        this.elevatorSubsystem.setElevatorPosition(elevatorInput);
        this.intakeSubsystem.setPosition(intakeInput);
        setLedColor(color);
    }

    public enum StatesToScore{
        L1(1),
        L2(2),
        L3(3),
        L4(4),

        ALGAE_L2(5),
        ALGAE_L3(6),
        PROCESSADOR(7);

        int button;
        StatesToScore(int button){
            this.button = button;
        }
    }

    public enum LedColors{
        L1_COLOR,
        L2_COLOR,
        L3_COLOR,
        L4_COLOR,
        CORAL_IN_INTAKE
    }

    private void setLedColor(Color color){
       if(intakeSubsystem.isTouched()){
            ledSubsystem.setPattern(LEDPattern.gradient(GradientType.kDiscontinuous, color, Color.kRed));;
       } else {
            ledSubsystem.setColor(color);
       }
    }
        public Command ScoreRobot(StatesToScore state){
        return run(() ->{
            switch (state) {
                case L1:
                    if(this.elevatorInput != ElevatorPositions.HOME && this.intakeInput != IntakePositions.ABERTURA_COMUMM)this.intakeInput = IntakePositions.ABERTURA_COMUMM;
                    if(intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.HOME;
                        if(elevatorSubsystem.atSetpoint() && intakeSubsystem.getSetpoint() == IntakePositions.ABERTURA_COMUMM){
                            this.intakeInput = IntakePositions.DEFAULT_POSITION;
                            color = Color.kGreen;
                        }
                    }
                    break;

                case L2:
                    this.intakeInput = IntakePositions.PUT_CORAL_ALTERNATIVE;
                    if(this.intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.L2;
                        color = Color.kYellow;
                    }
                    break;

                case L3:
                    this.intakeInput = IntakePositions.PUT_CORAL_ALTERNATIVE;
                    if(intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.L3;
                        color = Color.kPurple;
                    }
                    break;

                case L4:
                    if(this.elevatorInput != ElevatorPositions.L4) this.intakeInput = IntakePositions.PUT_CORAL;
                    if(intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.L4;
                        if(elevatorSubsystem.atSetpoint() && intakeSubsystem.getSetpoint() == IntakePositions.PUT_CORAL){
                            this.intakeInput = IntakePositions.OPEN_L4;
                            color = Color.kDarkBlue;
                        }
                    }
                    break;

                case ALGAE_L2:
                    this.intakeInput = IntakePositions.CONTROL_BALL;
                    if(intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.ALGAE_L2;
                    }
                    break;

                case ALGAE_L3:
                    this.intakeInput = IntakePositions.CONTROL_BALL;
                    if(this.intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.ALGAE_L3;
                    }
                    break;

                case PROCESSADOR:
                    this.intakeInput = IntakePositions.CONTROL_BALL;
                    if(this.intakeSubsystem.atSetpoint()){
                        this.elevatorInput = ElevatorPositions.HOME;
                    }
                default:
                    break;
            }
        });
    }

    public double getOutput(){
        return elevatorSubsystem.getOutput();
    }

    public void getCoraOnIntake(){
        if(!intakeSubsystem.isTouched()){
            intakeSubsystem.setSpeed(0.2);
        }
    }

    public boolean atSetpoint(){
        return intakeSubsystem.atSetpoint() && elevatorSubsystem.atSetpoint();
    }
}
