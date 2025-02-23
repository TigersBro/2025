// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ShuffleBoard9638Constants;

/** Add your docs here. */
public class ShuffleBoard9638 {

    private static Timer syncTimer;
    private static ScheduledExecutorService executor;
    private static Runnable runnable;

    // Add A Double to Shuffleboard
    public static void addDouble(String tab,
            String name,
            Double addDouble) {
        if (tab.isEmpty()) {
            tab = ShuffleBoard9638Constants.DEFAULT_TAB;
        }
        Shuffleboard.getTab(tab).addDouble(name, () -> addDouble);
    }

    public static void addDouble(
            String name,
            Double addDouble) {
        Shuffleboard.getTab(ShuffleBoard9638Constants.DEFAULT_TAB).addDouble(name, () -> addDouble);
    }

    // Add a String to Shuffleboard
    public static void addString(String name,
            String addString) {
        ShuffleboardTab tab = Shuffleboard.getTab(ShuffleBoard9638Constants.DEFAULT_TAB);
        tab.add(name, addString);

        // Shuffleboard.getTab(ShuffleBoard9638Constants.DEFAULT_TAB).getTitle("button").addString(name,
        // () -> addString.toString());
    }

    public static void addString(String tabname,
            String name,
            String addString) {
        if (tabname.isEmpty()) {
            tabname = Constants.ShuffleBoard9638Constants.DEFAULT_TAB;
        }
        ShuffleboardTab tab = Shuffleboard.getTab(ShuffleBoard9638Constants.DEFAULT_TAB);
        tab.add(name, addString);
    
        //Shuffleboard.getTab(tab).addString(name, addString.toString());
    }

    public static void startUpdate() {
        if (syncTimer.isRunning()) {
        } else {
            Executors.newScheduledThreadPool(1);
            executor.scheduleAtFixedRate(get_runnable(), 0, ShuffleBoard9638Constants.UPDATE_PERIOD,
                    java.util.concurrent.TimeUnit.SECONDS);

        }

    }

    public static Runnable get_runnable() {
        runnable = new Runnable() {
            public void run() {
                update_all_variables();
            }
        };
        return runnable;

    }

    public static void update_all_variables() {

    }

}
