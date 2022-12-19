package com.jakubspiewak.robotarm

import com.jakubspiewak.robotarm.robot.Robot
import com.jakubspiewak.robotarm.robot.RobotArmLengths
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.springframework.context.annotation.Configuration
import org.springframework.web.bind.annotation.PostMapping
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController
import org.springframework.web.servlet.config.annotation.CorsRegistry
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer

data class Vector3(val x: Double, val y: Double, val z: Double,val rx: Double, val ry: Double, val rz: Double)
data class V3(val x: Number, val y: Number, val z: Number)

@RestController
@RequestMapping("/")
class RobotArmController {
    private val robot: Robot = Robot(RobotArmLengths(40.0, 40.0, 20.0, 20.0, 10.0, 10.0))

    @PostMapping
    fun computeXYZ(@RequestBody position: Vector3) =
        robot.inverseKinematics(Vector3D(position.x, position.y, position.z), Vector3D(position.rx, position.ry, position.rz))
}

@Configuration
class WebConfiguration : WebMvcConfigurer {
    override fun addCorsMappings(registry: CorsRegistry) {
        registry.addMapping("/**").allowedMethods("*")
    }
}