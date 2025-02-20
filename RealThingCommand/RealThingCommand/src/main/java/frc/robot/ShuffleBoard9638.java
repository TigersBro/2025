// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
/** Add your docs here. */
public class ShuffleBoard9638 {


    //() -> armEncoder.getPosition()

    
    public static void addDouble( String tab ,
                                  String name,
                           Double addDouble )
    {
        if (tab.isEmpty())
        {
            tab = "Test";
        }
        Shuffleboard.getTab(tab).addDouble("Encoder Position", () -> addDouble);    
    }
    public static void addString( String name ,
                           String addString )
    {
        Shuffleboard.getTab(Constants.ShuffleBoard9638Constants.DEFAULT_TAB).addString("Encoder Position", () -> addString.toString() );    
    }
    public static void addString( String tab, 
                                String name ,
                           String addString )
    {
        if (tab.isEmpty())
        {
            tab = "Constants.ShuffleBoard9638Constants.DEFAULT_TAB";
        }
        Shuffleboard.getTab(tab).addString("Encoder Position", () -> addString.toString() );    
    }

}
