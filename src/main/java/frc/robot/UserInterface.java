package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class UserInterface {
    public static Map<String, ShuffleboardComponent> components = new HashMap<String, ShuffleboardComponent>();

    private UserInterface() {}

    /**
     * Gets or creates a Shuffleboard Tab
     */
    public ShuffleboardTab getTab(String tabName) {
        return Shuffleboard.getTab(tabName);
    }

    public void selectTab(String tabName) {
        Shuffleboard.selectTab(tabName);
    }

    public void selectTab(int tabIndex) {
        Shuffleboard.selectTab(tabIndex);
    }

    public void createComponent(String tabName, String componentName, Object componentValue, WidgetType componentType, int[] componentPosition, int[] componentSize, Map<String, Object> componentProperties) {
        components.put(componentName, Shuffleboard.getTab(tabName).add("", 0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(componentPosition[0], componentPosition[1])
        .withSize(componentSize[0], componentSize[1])
        .withProperties(componentProperties));
    }

    public void createComponent(String tabName, String componentName, Object componentValue, WidgetType componentType, int[] componentPosition, int[] componentSize) {
        components.put(componentName, Shuffleboard.getTab(tabName).add(componentName, 0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(componentPosition[0], componentPosition[1])
        .withSize(componentSize[0], componentSize[1]));
    }

    public void createComponent(String tabName, String componentName, Sendable componentValue, WidgetType componentType, int[] componentPosition, int[] componentSize, Map<String, Object> componentProperties) {
        components.put(componentName,Shuffleboard.getTab(tabName).add(componentName, componentValue)
        .withWidget(componentType)
        .withPosition(componentPosition[0], componentPosition[1])
        .withSize(componentSize[0], componentSize[1])
        .withProperties(componentProperties));
    }

    public void createComponent(String tabName, String componentName, Sendable componentValue, WidgetType componentType, int[] componentPosition, int[] componentSize) {
        components.put(componentName, Shuffleboard.getTab(tabName).add(componentName, componentValue)
        .withWidget(componentType)
        .withPosition(componentPosition[0], componentPosition[1])
        .withSize(componentSize[0], componentSize[1]));
    }

    public ShuffleboardComponent getComponentData(String componentName) {
        return components.get(componentName);
    }
}
