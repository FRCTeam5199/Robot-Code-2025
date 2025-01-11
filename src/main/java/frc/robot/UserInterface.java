package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UserInterface {
    private static ShuffleboardTab autonTab;
    private static ShuffleboardTab teleopTab;
    private static ShuffleboardTab controlTab;
    private static ShuffleboardTab testTab;

    private static Map<String, SimpleWidget> autonComponents = new HashMap<>();
    private static Map<String, SimpleWidget> teleopComponents = new HashMap<>();
    private static Map<String, SimpleWidget> controlComponents = new HashMap<>();
    private static Map<String, SimpleWidget> testComponents = new HashMap<>();

    private UserInterface() {}

    public static void init() {
        initalizeTabs();
        Shuffleboard.selectTab("Autons");
    }

    private static void initalizeTabs() {
        autonTab = Shuffleboard.getTab("Auton");
        teleopTab = Shuffleboard.getTab("Teleop");
        controlTab = Shuffleboard.getTab("Control");

        if (!DriverStation.isFMSAttached()) {
            testTab = Shuffleboard.getTab("Test");
        }
    }

    public static void setTab(String key) {
        Shuffleboard.selectTab(key);
    }

    public static void putData(String key, Sendable data) {
        SmartDashboard.putData(key, data);
    }

    public static Sendable getData(String key) {
        return SmartDashboard.getData(key);
    }

    public static void createAutonComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        autonComponents.put(name, autonTab.add(name, defaultValue).withWidget(widget).withPosition(0, 0).withSize(positionX, positionY).withProperties(properties));
    }

    public static void createTeleopComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        teleopComponents.put(name, autonTab.add(name, defaultValue).withWidget(widget).withPosition(0, 0).withSize(positionX, positionY).withProperties(properties));
    }

    public static void createControlComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        controlComponents.put(name, autonTab.add(name, defaultValue).withWidget(widget).withPosition(0, 0).withSize(positionX, positionY).withProperties(properties));
    }

    public static void createTestComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        testComponents.put(name, autonTab.add(name, defaultValue).withWidget(widget).withPosition(0, 0).withSize(positionX, positionY).withProperties(properties));
    }

    public static void setAutonComponent(String key, Object value) {
        autonComponents.get(key).getEntry(key).setValue(value);
    }

    public static void setTeleopComponent(String key, Object value) {
        teleopComponents.get(key).getEntry(key).setValue(value);
    }

    public static void setControlComponent(String key, Object value) {
        controlComponents.get(key).getEntry(key).setValue(value);
    }

    public static void setTestComponent(String key, Object value) {
        testComponents.get(key).getEntry(key).setValue(value);
    }

    public static void update() {
        Shuffleboard.update();
    }
}
