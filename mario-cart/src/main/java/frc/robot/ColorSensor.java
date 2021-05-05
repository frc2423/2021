package frc.robot;

import com.revrobotics.ColorSensorV3;

import java.util.HashMap;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;
import frc.team2423.devices.Device;
import edu.wpi.first.wpilibj.I2C;


public class ColorSensor extends Device{

    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatcher;
    private HashMap<String, Color> colors = new HashMap<String, Color>();

    public ColorSensor(){
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard); // idk waht this means
        colorMatcher =  new ColorMatch();
    }

    public void setConfidence(double confidence){
        colorMatcher.setConfidenceThreshold(confidence);
    }

    public void addColor(String name ,double r, double g, double b){
        Color color = ColorMatch.makeColor(r, g, b);
        colors.put(name, color);
        colorMatcher.addColorMatch(color);
    }

    public Color getColor(){
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult out = colorMatcher.matchColor(detectedColor);
        if(out != null) {
            return out.color;
        }
        return null;
    }

    public Color getRawColor(){
        return colorSensor.getColor();
    }

    public boolean isColor(String name){ // mmmmmmm bri'ish
        if(!colors.containsKey(name)){
            return false;
        }
        Color colour = getColor();
        if(colour == null) {
            return false;
        }            
        return colour.equals(colors.get(name));
  
    }



}