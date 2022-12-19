package com.jakubspiewak.robotarm.robot

import java.math.BigDecimal

/*
elements: ║ ═

    J2 ════ J3 ════ J4
    ║               ║
    ║               J5
    ║               ║
    J1              END
    ║
    ║
    ║
    J0

 */
data class RobotArmLengths(
    // length between join j0 and join j1
    val l0: BigDecimal,
    // length between join j1 and join j2
    val l1: BigDecimal,
    // length between join j2 and join j3
    val l2: BigDecimal,
    // length between join j3 and join j4
    val l3: BigDecimal,
    // length between join j4and join j5
    val l4: BigDecimal,
    // length between join j5 and end-effector
    val l5: BigDecimal
)
