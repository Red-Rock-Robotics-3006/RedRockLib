package redrocklib.logging;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardBoolean {
    private boolean defaultValue;
    @AutoLogOutput(key = "{key}")
    private boolean lastValue;
    private String key;

    public SmartDashboardBoolean(String key, boolean defaultValue){
        this.key = key;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;

        SmartDashboard.putBoolean(this.key, this.defaultValue);
    }

    public void putBoolean(boolean val){
        SmartDashboard.putBoolean(this.key, val);
    }

    public void setDefaultValue(boolean val){
        this.defaultValue = val;
    }

    public boolean getValue(){
        this.lastValue = SmartDashboard.getBoolean(this.key, this.defaultValue);

        return this.lastValue;
    }

    public boolean hasChanged(){
        return this.lastValue != this.getValue();
    }
}