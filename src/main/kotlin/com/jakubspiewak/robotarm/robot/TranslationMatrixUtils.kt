package com.jakubspiewak.robotarm.robot

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix
import java.math.BigDecimal
import java.math.RoundingMode
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.sin

class TranslationMatrixUtils {
    companion object {

        fun rotX(alpha: Double): RealMatrix {
            return MatrixUtils.createRealMatrix(
                arrayOf(
                    doubleArrayOf(1.0, 0.0, 0.0),
                    doubleArrayOf(0.0, cos(alpha), -sin(alpha)),
                    doubleArrayOf(0.0, sin(alpha), cos(alpha)),
                )
            )
        }

        fun rotY(alpha: Double): RealMatrix {
            return MatrixUtils.createRealMatrix(
                arrayOf(
                    doubleArrayOf(cos(alpha), 0.0, sin(alpha)),
                    doubleArrayOf(0.0, 1.0, 0.0),
                    doubleArrayOf(-sin(alpha), 0.0, cos(alpha)),
                )
            )
        }

        fun rotZ(alpha: Double): RealMatrix {
            return MatrixUtils.createRealMatrix(
                arrayOf(
                    doubleArrayOf(cos(alpha), -sin(alpha), 0.0),
                    doubleArrayOf(sin(alpha), cos(alpha), 0.0),
                    doubleArrayOf(0.0, 0.0, 1.0),
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

        fun rotXYZ(angles: Vector3D): RealMatrix {
            return rotZ(angles.z)
                .multiply(rotY(angles.y))
                .multiply(rotX(angles.x))
        }

        fun htm(theta: Double, alpha: Double, r: Double, d: Double): RealMatrix {
            val sinT = sin(theta)
            val cosT = cos(theta)
            val sinA = sin(alpha)
            val cosA = cos(alpha)

            return MatrixUtils.createRealMatrix(
                arrayOf(
                    doubleArrayOf(cosT, -cosA * sinT, sinA * sinT, r * cosT),
                    doubleArrayOf(sinT, cosA * cosT, -sinA * cosT, r * sinT),
                    doubleArrayOf(0.0, sinA, cosA, d),
                    doubleArrayOf(0.0, 0.0, 0.0, 1.0)
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

fun RealMatrix.printMatrix() {
    println()
    for (i in 0 until this.rowDimension) {
        for (j in 0 until this.columnDimension) {
            val value = BigDecimal(this.getEntry(i, j)).setScale(20, RoundingMode.HALF_UP)
            print("$value ")
        }
        println()
    }
}

fun RealMatrix.getRotMatrix(): RealMatrix = this.getSubMatrix(0, 2, 0, 2)

fun Double.pow2(): Double = this * this



