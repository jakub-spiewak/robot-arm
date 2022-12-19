package com.jakubspiewak.robotarm.robot

import ch.obermuhlner.math.big.BigDecimalMath
import com.jakubspiewak.robotarm.V3
import com.jakubspiewak.robotarm.robot.TranslationMatrixUtils.Companion.htm
import com.jakubspiewak.robotarm.robot.TranslationMatrixUtils.Companion.rotXYZ
import org.apache.commons.math3.linear.FieldMatrix
import org.apache.commons.math3.linear.FieldVector
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.util.BigReal
import java.math.BigDecimal
import java.math.BigDecimal.ZERO
import java.math.MathContext

class Robot(private val arms: RobotArmLengths) {

    private fun computeFirst3JoinsAngles(position: FieldVector<BigReal>): Triple<BigDecimal, BigDecimal, BigDecimal> {
        val a1 = arms.l0
        val a2 = arms.l1
        val a3 = arms.l2 + arms.l3

        val x = position.getEntry(0).bigDecimalValue()
        val y = position.getEntry(1).bigDecimalValue()
        val z = position.getEntry(2).bigDecimalValue()

        val r1 = BigDecimalMath.sqrt(x.pow2() + y.pow2(), MathContext.DECIMAL128)
        val r2 = z.subtract(a1)
        val r3 = BigDecimalMath.sqrt(r1.pow2()+r2.pow2(), MathContext.DECIMAL128)

        val phi1 = BigDecimalMath.acos((a3.pow2() - a2.pow2() - r3.pow2()) / (BigDecimal(-2) * a2 * r3), MathContext.DECIMAL128)
        val phi2 = BigDecimalMath.atan2(r2, r1, MathContext.DECIMAL128)
        val phi3 = BigDecimalMath.acos((r3.pow2() - a2.pow2() - a3.pow2()) / (BigDecimal(-2) * a2 * a3), MathContext.DECIMAL128)

        val theta1 = BigDecimalMath.atan2(y, x, MathContext.DECIMAL128)
        val theta2 = -(BigDecimalMath.pi(MathContext.DECIMAL128) / BigDecimal(2)) + phi2 + phi1
        val theta3 = -(BigDecimalMath.pi(MathContext.DECIMAL128) / BigDecimal(2)) + phi3

        return Triple(theta1, theta2, theta3)
    }

    fun inverseKinematics(position: FieldVector<BigReal>, orientation: FieldVector<BigReal>): RobotJoinAngles {
        val o06: FieldMatrix<BigReal> = MatrixUtils.createColumnFieldMatrix(position.toArray())
        val d6 = this.arms.l4 + this.arms.l5

        val r06 = rotXYZ(orientation)
        val r06x = r06.getColumnMatrix(2)
        val o0C = o06.add(r06x.scalarMultiply(BigReal(d6)))

        val theta123 = computeFirst3JoinsAngles(o0C.getColumnVector(0))

        val r03 = htm03(theta123).getRotMatrix()
        val r36 = r03.transpose().multiply(r06)

        val (theta1, theta2, theta3) = theta123
        val theta4 = BigDecimalMath.atan2(r36[1][2], r36[0][2], MathContext.DECIMAL128)
        val theta5 = BigDecimalMath.asin(r36[2][2], MathContext.DECIMAL128)
        val theta6 = BigDecimalMath.atan2(-r36[2][1], r36[2][0], MathContext.DECIMAL128)
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
            V3(o0C.data[0][0].bigDecimalValue(), o0C.data[1][0].bigDecimalValue(), o0C.data[2][0].bigDecimalValue())
        )
    }

    private fun htm01(theta: BigDecimal): FieldMatrix<BigReal> = htm(theta, BigDecimalMath.pi(MathContext.DECIMAL128) / BigDecimal(2), ZERO, arms.l0)
    private fun htm12(theta: BigDecimal): FieldMatrix<BigReal> = htm(theta + BigDecimalMath.pi(MathContext.DECIMAL128) /BigDecimal(2), ZERO, arms.l1, ZERO)
    private fun htm23(theta: BigDecimal): FieldMatrix<BigReal> = htm(theta, BigDecimalMath.pi(MathContext.DECIMAL128) /BigDecimal(2), ZERO, ZERO)

    private fun htm03(theta: Triple<BigDecimal, BigDecimal, BigDecimal>) = htm01(theta.first)
        .multiply(htm12(theta.second))
        .multiply(htm23(theta.third))

    private fun htm34(theta: BigDecimal): FieldMatrix<BigReal> = htm(theta, BigDecimalMath.pi(MathContext.DECIMAL128) /BigDecimal(2), ZERO, arms.l2 + arms.l3)
    private fun htm45(theta: BigDecimal): FieldMatrix<BigReal> = htm(theta + BigDecimalMath.pi(MathContext.DECIMAL128) /BigDecimal(2), BigDecimalMath.pi(MathContext.DECIMAL128) /BigDecimal(2), ZERO, ZERO)
    private fun htm56(theta: BigDecimal): FieldMatrix<BigReal> = htm(theta, ZERO, ZERO, arms.l4 + arms.l5)

    private fun htm36(theta: Triple<BigDecimal, BigDecimal, BigDecimal>) = htm34(theta.first)
        .multiply(htm45(theta.second))
        .multiply(htm56(theta.third))

    private fun htm06(theta123: Triple<BigDecimal, BigDecimal, BigDecimal>, theta456: Triple<BigDecimal, BigDecimal, BigDecimal>) =
        htm03(theta123).multiply(htm36(theta456))
}

private operator fun FieldMatrix<BigReal>.get(i: Int): List<BigDecimal> {
   return this.getRow(i).map { it.bigDecimalValue() }
}
