package frc.robot.subsystems.Led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Led;

public class LedSubsystem extends SubsystemBase{

    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    public static LedSubsystem mIntance = null;

    private LedSubsystem(){
        this.led = new AddressableLED(Led.LED_PORT);
        this.buffer = new AddressableLEDBuffer(Led.LED_LENGTH);

        this.led.setLength(buffer.getLength());
        this.led.start();
    }

    public static LedSubsystem getInstance() {
        if (mIntance == null) {
            mIntance = new LedSubsystem();
        }
        return mIntance;
    }
    
    @Override
    public void periodic() {
        this.led.setData(buffer);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(buffer));
    }

    public void setColor(Color color){
        setPattern(LEDPattern.solid(color));
    }

    public void setPattern(LEDPattern pattern){
        pattern.applyTo(buffer);
        led.setData(buffer);
    }

    public void setRainbow() {
        Distance ledSpace = Meters.of(1.0 / 60.0);
        LEDPattern rainbowPattern = LEDPattern.rainbow(255, 128);
        LEDPattern scrollingRainbowPattern = 
            rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpace);

        scrollingRainbowPattern.applyTo(buffer);
    }

    public void setCoralCatched() {
        LEDPattern blue = LEDPattern.solid(Color.kBlue);
        blue.applyTo(buffer);
    }

    public void off() {
        LEDPattern offPattern = LEDPattern.kOff;
        offPattern.applyTo(buffer);
    } 
}
