package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ScoreCommands;

public class UserInterface {
    private static ShuffleboardTab autonTab, teleopTab, controlTab, testTab;

    public static Map<String, GenericEntry> autonComponents = new HashMap<String, GenericEntry>();
    public static Map<String, GenericEntry> teleopComponents = new HashMap<String, GenericEntry>();
    public static Map<String, GenericEntry> controlComponents = new HashMap<String, GenericEntry>();
    public static Map<String, GenericEntry> testComponents = new HashMap<String, GenericEntry>();

    private UserInterface() {}

    public static void init() {
        initalizeTabs();
        initalizeComponents();
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

    private static void initalizeComponents() {
        Map<String, Object> allianceComponentProperties = new HashMap<>();
        allianceComponentProperties.put("True Color", "#0050FF");
        
        createAutonComponent("Alliance", false, BuiltInWidgets.kBooleanBox, 0, 0, 1, 1, allianceComponentProperties);

        createControlComponent("Reset All", false, BuiltInWidgets.kToggleButton, 0, 0, 1, 1, null);
        createControlComponent("Reset Elevator", false, BuiltInWidgets.kToggleButton, 0, 1, 1, 1, null);
        createControlComponent("Reset Arm", false, BuiltInWidgets.kToggleButton, 0, 2, 1, 1, null);
        createControlComponent("Reset Wrist", false, BuiltInWidgets.kToggleButton, 0, 3, 1, 1, null);
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

    @SuppressWarnings("rawtypes")
    public static void addSendableChooserComponent(String name, SendableChooser sendableChooser, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        autonTab.add(name, sendableChooser).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties);
    }

    public static void createAutonComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        autonComponents.putIfAbsent(name, autonTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties).getEntry());
    }

    public static void createTeleopComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        teleopComponents.putIfAbsent(name, teleopTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties).getEntry());
    }

    public static void createControlComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        controlComponents.putIfAbsent(name, controlTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties).getEntry());
    }

    public static void createTestComponent(String name, Object defaultValue, WidgetType widget, int positionX, int positionY, int width, int height, Map<String, Object> properties) {
        testComponents.putIfAbsent(name, testTab.add(name, defaultValue).withWidget(widget).withPosition(positionX, positionY).withSize(width, height).withProperties(properties).getEntry());
    }

    public static GenericEntry getAutonComponent(String key) {
        return autonComponents.get(key);
    }

    public static GenericEntry getTeleopComponent(String key) {
        return teleopComponents.get(key);
    }

    public static GenericEntry getControlComponent(String key) {
        return controlComponents.get(key);
    }

    public static GenericEntry getTestComponent(String key) {
        return testComponents.get(key);
    }

    public static void setAutonComponent(String key, Object value) {
        if (autonComponents.get(key).setValue(value)) { autonComponents.replace(key, autonComponents.get(key)); }
    }

    public static void setTeleopComponent(String key, Object value) {
        if (teleopComponents.get(key).setValue(value)) { teleopComponents.replace(key, autonComponents.get(key)); }
    }

    public static void setControlComponent(String key, Object value) {
        controlComponents.get(key).setValue(value);
    }

    public static void setTestComponent(String key, Object value) {
        if (testComponents.get(key).setValue(value)) { testComponents.replace(key, autonComponents.get(key)); }
    }

    public static void update() {
        Shuffleboard.update();
    }
}
