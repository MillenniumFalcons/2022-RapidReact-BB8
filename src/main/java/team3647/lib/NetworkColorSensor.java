// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.lib;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public final class NetworkColorSensor {
    public final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    public final NetworkTableEntry proximityEntry;
    public final NetworkTableEntry rawColorsEntry;

    private final double[] kEmptyDoubleThree = new double[3];
    private final int kMaxReadDistance;

    private static final int kRedIndex = 0;
    private static final int kGreenIndex = 1;
    private static final int kBlueIndex = 2;

    public enum Color {
        NONE("None"),
        RED("Red"),
        GREEN("Green"),
        BLUE("Blue");
        public final String str;

        private Color(String name) {
            str = name;
        }
    }

    private double proximity;
    private double[] rawColors = new double[3];

    private Color currentColor = Color.NONE;

    public NetworkColorSensor(String proximity, String color, int maxReadDistance) {
        this.proximityEntry = networkTableInstance.getEntry(proximity);
        this.rawColorsEntry = networkTableInstance.getEntry(color);

        kMaxReadDistance = maxReadDistance;
        this.proximityEntry.addListener(this::processNTEvent, EntryListenerFlags.kUpdate);
    }

    private synchronized void processNTEvent(EntryNotification notification) {
        proximity = proximityEntry.getDouble(0.0);
        rawColors = rawColorsEntry.getDoubleArray(kEmptyDoubleThree);
        currentColor =
                updateColor(
                        rawColors[kRedIndex],
                        rawColors[kGreenIndex],
                        rawColors[kBlueIndex],
                        isReadColor());
    }

    public synchronized double getProximity() {
        return proximity;
    }

    public synchronized boolean isReadColor() {
        return this.proximity > kMaxReadDistance;
    }

    private static Color updateColor(
            double red, double green, double blue, boolean withinDistance) {
        Color result;
        if (green > red && green > blue || !withinDistance) {
            result = Color.NONE;
        } else if (blue > red) {
            result = Color.BLUE;
        } else {
            result = Color.RED;
        }
        return result;
    }

    public synchronized String getColorAsString() {
        return this.currentColor.str;
    }

    public synchronized Color getColor() {
        return this.currentColor;
    }
}
