package frc.robot.utils.DriveStationIO;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveStationIO {
    private static DriveStationIO instance;

    /*
     * Single instance
     */
    private DriveStationIO(){}

    public static DriveStationIO getInstance(){
        if(instance == null) instance = new DriveStationIO();
        return instance;
    }

    /*
     * Alliance info
     */

    /**
     * check if the robot is in Blue Alliance
     * @return true if <b>Blue</b>, false if <b>Red</b> or <b>no Alliance</b>
     */
    public boolean isBlue(){
        return (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() == Alliance.Blue : false; 
    }

    /**
     * check if the robot is in Red Alliance
     * @return true if <b>Red</b>, false if <b>Blue</b> or <b>no Alliance</b>
     */
    public boolean isRed(){
        return (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() == Alliance.Red : false; 
    }

    /**
     * get Alliance selected
     * @param defaultResult return value if the Alliance is not present
     * @return Alliance Type
     */
    public Alliance getAlliance(Alliance defaultResult){
        return (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() : defaultResult;
    }

    /**
     * get Alliance selected
     * @return Alliance Type, Blue Alliance if the Alliance is not present
     */
    public Alliance getAlliance(){
        return getAlliance(Alliance.Blue);
    }

    /*
     * Modes
     */
    public boolean isAuto(){
        return DriverStation.isAutonomous();
    }

    public boolean isAutoEnabled(){
        return DriverStation.isAutonomousEnabled();
    }

    public boolean isTeleop(){
        return DriverStation.isTeleop();
    }

    public boolean isTeleopEnabled(){
        return DriverStation.isTeleopEnabled();
    }

    public boolean isEnabled(){
        return DriverStation.isEnabled();
    }

    public boolean isDisabled(){
        return DriverStation.isDisabled();
    }
}

