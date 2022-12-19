package com.jakubspiewak.robotarm

import com.jakubspiewak.robotarm.robot.Robot
import com.jakubspiewak.robotarm.robot.RobotArmLengths
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.util.BigReal
import org.springframework.context.annotation.Configuration
import org.springframework.web.bind.annotation.PostMapping
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController
import org.springframework.web.servlet.config.annotation.CorsRegistry
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer
import java.math.BigDecimal

data class Vector3(val x: Double, val y: Double, val z: Double, val rx: Double, val ry: Double, val rz: Double)
data class V3(val x: Number, val y: Number, val z: Number)

@RestController
@RequestMapping("/")
class RobotArmController {
    private val robot: Robot = Robot(
        RobotArmLengths(
            BigDecimal(40.0),
            BigDecimal(40.0),
            BigDecimal(20.0),
            BigDecimal(20.0),
            BigDecimal(10.0),
            BigDecimal(10.0)
        )
    )

    @PostMapping
    fun computeXYZ(@RequestBody position: Vector3) =
        robot.inverseKinematics(
            MatrixUtils.createFieldVector(
                arrayOf(BigReal(position.x), BigReal(position.y), BigReal(position.z))
            ),
            MatrixUtils.createFieldVector(
                arrayOf(BigReal(position.rx), BigReal(position.ry), BigReal(position.rz))
            )
        )
}

@Configuration
class WebConfiguration : WebMvcConfigurer {
    override fun addCorsMappings(registry: CorsRegistry) {
        registry.addMapping("/**").allowedMethods("*")
    }
}