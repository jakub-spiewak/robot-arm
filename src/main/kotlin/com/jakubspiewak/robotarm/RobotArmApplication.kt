package com.jakubspiewak.robotarm

import org.springframework.boot.autoconfigure.SpringBootApplication
import org.springframework.boot.runApplication

@SpringBootApplication
class RobotArmApplication

fun main(args: Array<String>) {
    runApplication<RobotArmApplication>(*args)
}
