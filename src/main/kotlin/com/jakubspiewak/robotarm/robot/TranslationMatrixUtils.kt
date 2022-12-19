package com.jakubspiewak.robotarm.robot

import ch.obermuhlner.math.big.BigDecimalMath
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.linear.FieldMatrix
import org.apache.commons.math3.linear.FieldVector
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.util.BigReal
import org.apache.commons.math3.util.BigReal.ONE
import org.apache.commons.math3.util.BigReal.ZERO
import java.math.BigDecimal
import java.math.MathContext
import kotlin.math.abs
import kotlin.math.acos

class TranslationMatrixUtils {
    companion object {

        fun rotX(alpha: BigDecimal): FieldMatrix<BigReal> {
            return MatrixUtils.createFieldMatrix(
                arrayOf(
                    arrayOf(ONE, ZERO, ZERO),
                    arrayOf(ZERO, alpha.cos(), alpha.sin().negate()),
                    arrayOf(ZERO, alpha.sin(), alpha.cos()),
                )
            )
        }

        fun rotY(alpha: BigDecimal): FieldMatrix<BigReal> {
            return MatrixUtils.createFieldMatrix(
                arrayOf(
                    arrayOf(alpha.cos(), ZERO, alpha.sin()),
                    arrayOf(ZERO, ONE, ZERO),
                    arrayOf(alpha.sin().negate(), ZERO, alpha.cos()),
                )
            )
        }

        fun rotZ(alpha: BigDecimal): FieldMatrix<BigReal> {
            return MatrixUtils.createFieldMatrix(
                arrayOf(
                    arrayOf(alpha.cos(), alpha.sin().negate(), ZERO),
                    arrayOf(alpha.sin(), alpha.cos(), ZERO),
                    arrayOf(ZERO, ZERO, ONE),
                )
            )
        }

        fun translateXYZ(vector: Vector3D): RealMatrix {
            val x = vector.x
            val y = vector.y
            val z = vector.z
            return MatrixUtils.createRealMatrix(
                arrayOf(
                    doubleArrayOf(1.0, 0.0, 0.0, x),
                    doubleArrayOf(0.0, 1.0, 0.0, y),
                    doubleArrayOf(0.0, 0.0, 1.0, z),
                    doubleArrayOf(0.0, 0.0, 0.0, 1.0)
                )
            )
        }

        fun rotXYZ(angles: FieldVector<BigReal>): FieldMatrix<BigReal> {
            return rotZ(angles.getEntry(2).bigDecimalValue())
                .multiply(rotY(angles.getEntry(1).bigDecimalValue()))
                .multiply(rotX(angles.getEntry(0).bigDecimalValue()))
        }

        fun htm(theta: BigDecimal, alpha: BigDecimal, r: BigDecimal, d: BigDecimal): FieldMatrix<BigReal> {
            val sinT = theta.sin()
            val cosT = theta.cos()
            val sinA = alpha.sin()
            val cosA = alpha.cos()

            return MatrixUtils.createFieldMatrix(
                arrayOf(
                    arrayOf(cosT, cosA.multiply(sinT).negate(), sinA .multiply( sinT),  cosT.multiply(BigReal(r))),
                    arrayOf(sinT, cosA.multiply(cosT), sinA.multiply( cosT).negate(), sinT.multiply(BigReal(r))),
                    arrayOf(ZERO, sinA, cosA, BigReal(d)),
                    arrayOf(ZERO, ZERO, ZERO, ONE)
                )
            )
        }

        fun cosLaw(c: Double, a: Double, b: Double): Double {
            val a1 = a.pow2() - b.pow2() - c.pow2()
            val a2 = -2 * a * b
            println(Triple(c, a, b))
            println(Pair(a1, a2))
            return acos(abs(a1 / a2))
        }

    }
}

fun FieldMatrix<BigReal>.printMatrix() {
    println()
    for (i in 0 until this.rowDimension) {
        for (j in 0 until this.columnDimension) {
//            val value = BigDecimal(this.getEntry(i, j)).setScale(20, RoundingMode.HALF_UP)
            val value = this.getEntry(i, j).bigDecimalValue()
            print("$value ")
        }
        println()
    }
}

fun RealMatrix.getRotMatrix(): RealMatrix = this.getSubMatrix(0, 2, 0, 2)
fun FieldMatrix<BigReal>.getRotMatrix(): FieldMatrix<BigReal> = this.getSubMatrix(0, 2, 0, 2)


fun Double.pow2(): Double = this * this

fun BigDecimal.sin(): BigReal {
    return BigReal(BigDecimalMath.sin(this, MathContext.DECIMAL128))
}

fun BigDecimal.cos(): BigReal {
    return BigReal(BigDecimalMath.cos(this, MathContext.DECIMAL128))
}

fun BigDecimal.pow2(): BigDecimal {
    return BigDecimalMath.pow(this, BigDecimal(2), MathContext.DECIMAL128)
}