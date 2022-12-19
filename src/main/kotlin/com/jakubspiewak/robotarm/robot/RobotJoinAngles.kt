package com.jakubspiewak.robotarm.robot

import com.jakubspiewak.robotarm.V3
import com.jakubspiewak.robotarm.Vector3

data class RobotJoinAngles(
    val j0: Number,
    val j1: Number,
    val j2: Number,
    val j3: Number,
    val j4: Number,
    val j5: Number,
    val wCenter: V3
) {
}