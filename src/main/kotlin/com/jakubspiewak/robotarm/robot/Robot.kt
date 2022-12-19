package com.jakubspiewak.robotarm.robot

import com.jakubspiewak.robotarm.V3
import com.jakubspiewak.robotarm.robot.TranslationMatrixUtils.Companion.htm
import com.jakubspiewak.robotarm.robot.TranslationMatrixUtils.Companion.rotXYZ
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.linear.RealVector
import kotlin.math.*

class Robot(private val arms: RobotArmLengths) {

    private fun computeFirst3JoinsAngles(position: RealVector): Triple<Double, Double, Double> {
        val a1 = arms.l0
        val a2 = arms.l1
        val a3 = arms.l2 + arms.l3

        val x = position.getEntry(0)
        val y = position.getEntry(1)
        val z = position.getEntry(2)

        val r1 = sqrt(x.pow2() + y.pow2())
        val r2 = z - a1
        val r3 = sqrt(r1.pow2() + r2.pow2())

        val phi1 = acos((a3.pow2() - a2.pow2() - r3.pow2()) / (-2 * a2 * r3)).let { if (it.isNaN()) 0.0 else it }
        val phi2 = atan2(r2, r1)
        val phi3 = acos((r3.pow2() - a2.pow2() - a3.pow2()) / (-2 * a2 * a3)).let { if (it.isNaN()) PI else it }

        val theta1 = atan2(y, x)
        val theta2 = -(PI / 2) + phi2 + phi1
        val theta3 = -(PI / 2) + phi3

        return Triple(theta1, theta2, theta3)
    }

    fun inverseKinematics(position: Vector3D, orientation: Vector3D): RobotJoinAngles {
        val o06: RealMatrix = MatrixUtils.createColumnRealMatrix(doubleArrayOf(position.x, position.y, position.z))
        val d6 = this.arms.l4 + this.arms.l5

        val r06 = rotXYZ(orientation)
        val r06x = r06.getColumnMatrix(2)
        val o0C = o06.add(r06x.scalarMultiply(d6))

        val theta123 = computeFirst3JoinsAngles(o0C.getColumnVector(0))

        val r03 = htm03(theta123).getRotMatrix()
        val r36 = r03.transpose().multiply(r06)

        val (theta1, theta2, theta3) = theta123
        val theta4 = atan2(r36[1][2], r36[0][2])
        val theta5 = asin(r36[2][2])
        val theta6 = atan2(-r36[2][1], r36[2][0])
        println(Triple(theta4, theta5, theta6))

        // control print

        println("$$$$$$$$$$$$$$")
        htm06(theta123, Triple(theta4, theta5, theta6)).getRotMatrix().printMatrix()
        println("##############")
        r06.printMatrix()
        println("$$$$$$$$$$$$$$")

        return RobotJoinAngles(
            theta1,
            theta2,
            theta3,
            theta4,
            theta5,
            theta6,
            V3(o0C.data[0][0], o0C.data[1][0], o0C.data[2][0])
        )
    }

    private fun htm01(theta: Double): RealMatrix = htm(theta, PI / 2, 0.0, arms.l0)
    private fun htm12(theta: Double): RealMatrix = htm(theta + PI / 2, 0.0, arms.l1, 0.0)
    private fun htm23(theta: Double): RealMatrix = htm(theta, PI / 2, 0.0, 0.0)

    private fun htm03(theta: Triple<Double, Double, Double>) = htm01(theta.first)
        .multiply(htm12(theta.second))
        .multiply(htm23(theta.third))

    private fun htm34(theta: Double): RealMatrix = htm(theta, PI / 2, 0.0, arms.l2 + arms.l3)
    private fun htm45(theta: Double): RealMatrix = htm(theta + PI / 2, PI / 2, 0.0, 0.0)
    private fun htm56(theta: Double): RealMatrix = htm(theta, 0.0, 0.0, arms.l4 + arms.l5)

    private fun htm36(theta: Triple<Double, Double, Double>) = htm34(theta.first)
        .multiply(htm45(theta.second))
        .multiply(htm56(theta.third))

    private fun htm06(theta123: Triple<Double, Double, Double>, theta456: Triple<Double, Double, Double>) =
        htm03(theta123).multiply(htm36(theta456))
}

private operator fun RealMatrix.get(i: Int): DoubleArray {
   return this.getRow(i)
}
