package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UserInterface {
    private static ShuffleboardTab autonTab, teleopTab, controlTab, testTab;

    @SuppressWarnings("rawtypes")
    private static Map<String, ShuffleboardComponent> autonComponents, teleopComponents, controlComponents, testComponents = new HashMap<>();

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
    
    public static ShuffleboardTab getTab(String key) {
        return Shuffleboard.getTab(key);
    }

    public static void putData(String key, Sendable data) {
        SmartDashboard.putData(key, data);
    }

    public static Sendable getData(String key) {
        return SmartDashboard.getData(key);
    }

    public static void addAutonSendableChooserComponent(String name, SendableChooser sendableChooser, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        autonComponents.put(name, autonTab.add(name, sendableChooser).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties));
    }

    public static void createAutonComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        autonComponents.put(name, autonTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties));
    }

    public static void createTeleopComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        teleopComponents.put(name, teleopTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties));
    }

    public static void createControlComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        controlComponents.put(name, controlTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties));
    }

    public static void createTestComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        testComponents.put(name, testTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties));
    }

    public static ShuffleboardComponent getAutonComponent(String key, Object value) {
        return autonComponents.get(key);
    }

    public static ShuffleboardComponent getTeleopComponent(String key, Object value) {
        return teleopComponents.get(key);
    }

    public static ShuffleboardComponent getControlComponent(String key, Object value) {
        return controlComponents.get(key);
    }

    public static ShuffleboardComponent getTestComponent(String key, Object value) {
        return testComponents.get(key);
    }

    // public static void setAutonComponent(String key, Object value) {
    //     autonComponents.replace(key, );
    // }

    // public static void setTeleopComponent(String key, Object value) {
    //     teleopComponents.get(key);//.getEntry(key).setValue(value);
    // }

    // public static void setControlComponent(String key, Object value) {
    //     controlComponents.get(key);//.getEntry(key).setValue(value);
    // }

    // public static void setTestComponent(String key, Object value) {
    //     testComponents.get(key);//.getEntry(key).setValue(value);
    // }

    public static void update() {
        Shuffleboard.update();
    }
}
