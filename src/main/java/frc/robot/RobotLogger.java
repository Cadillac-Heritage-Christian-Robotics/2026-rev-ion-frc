package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;

public class RobotLogger {
    public static void info(String subsystem, String message) {
        DataLogManager.log("[INFO][" + subsystem + "] " + message);
    }

    public static void warn(String subsystem, String message) {
        DataLogManager.log("[WARN][" + subsystem + "] " + message);
    }

    public static void error(String subsystem, String message) {
        DataLogManager.log("[ERROR][" + subsystem + "] " + message);
    }

    public static void debug(String subsystem, String message) {
        DataLogManager.log("[DEBUG][" + subsystem + "] " + message);
    }
}