/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5571.robot;

/**
 * Add your docs here.
 */
public class QueueCommand {

    /**
     * 0 - Forward/Backward
     * 1 - Left/Right
     * 
     * Positive - Forward/Right
     * Negative - Backward/Left
     */
    int _type;
    int _value;
    
    public QueueCommand(int type, int value) {
        _type = type;
        _value = value;
    }

    public int getType() {
        return _type;
    }
    
    public int getValue() {
        return _value;
    }

    public int getLeftTargetValue() {
        return _value * _type;
    }

    public int getRightTargetValue() {
        return _value;
    }
    
}
