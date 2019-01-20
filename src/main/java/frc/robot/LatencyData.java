/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;

/**
 * Tracks a number of data point sand allows updating of old points which then correct later points
 */
public class LatencyData implements PIDSource{
    private double[] data;
    private double[] timestamps;
    private int currentIndex = 0;
    private int furthestIndex = 0; // Never resets, once curIndex has wrapped will always in length-1

    public LatencyData(int points) {
        data = new double[points];
        timestamps = new double[points];
    }

    public void addDataPoint(double point) {
        if (currentIndex > furthestIndex) {
            furthestIndex = currentIndex;
        }
        timestamps[currentIndex] = Timer.getFPGATimestamp(); // Java also has System.getNanoTime() which might work
        data[currentIndex] = point;
        currentIndex++;
        if (currentIndex >= data.length) {
            currentIndex = 0;
        }
    }

    public double getCurrentPoint() {
        return data[Math.floorMod(currentIndex-1, data.length)];
    }

    /**
     * Correct all points since timestamp based on point
     * @returns Whether the point was successfully applied
     */
    public boolean addCorrectedData(double point, double timestamp) {
        int indexBefore = Math.floorMod((currentIndex-1), data.length);
        while (timestamps[indexBefore] > timestamp) {
            indexBefore--;
            indexBefore = Math.floorMod(indexBefore, data.length);
            if (timestamps[indexBefore] == 0) {
                // No valid data is there for the time
                return false;
            }
        }
        int indexAfter = (indexBefore+1) % data.length;
        // Linear interpolation
        double originalPoint = data[indexBefore] + (timestamp-timestamps[indexBefore])
            * ((data[indexAfter]-data[indexBefore])/
            (double)(timestamps[indexAfter]-timestamps[indexBefore]));
        double difference = point - originalPoint;
        // Iternate from currentIndex-1 (most recent point) backwards to indexBefore, apply difference
        for (int i = Math.floorMod((currentIndex-1), data.length); Math.floorMod(i, data.length) != indexBefore; i--) {
            data[Math.floorMod(i, data.length)] += difference;
        }
        return true;
    }

    public void clear() {
        for (int i = 0; i < data.length; i++) {
            data[i] = 0;
            timestamps[i] = 0;
        }
        currentIndex = 0;
        furthestIndex = 0;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        // Not supported
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return getCurrentPoint();
	}
}
