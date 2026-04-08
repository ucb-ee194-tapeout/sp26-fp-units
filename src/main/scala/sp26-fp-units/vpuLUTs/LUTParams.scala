package sp26FPUnits

import chisel3._
import chisel3.util._

trait LUTParams {
    def lutFixedToBf16Sqrt(x: UInt, exp: UInt, fra: UInt, lutValM: Int = 1, lutValN: Int = 16, neg: Bool = false.B): UInt = {
        val inW = lutValM + lutValN
        require(x.getWidth == inW, s"lutFixedToBf16 expects ${inW}-bit UInt input")

        val expW  = 8
        val fracW = 7
        val bias  = 127

        // Index of highest 1 bit in x
        val highestIdx = Wire(UInt(log2Ceil(inW).W))
        highestIdx := 0.U
        for (i <- 0 until inW) {
            when(x(i)) { highestIdx := i.U }
        }

        // Since x represents real_value * 2^lutValN
        // Real exponent = highestIdx - lutValN
        val unbiasedExp = highestIdx.zext - lutValN.S
        val bfExpSigned = bias.S + ((unbiasedExp + exp.zext - bias.S) >> 1) // Divide by 2 since we're doing sqrt
        val expUnsigned = bfExpSigned.asUInt(7,0)

        // Shift so leading 1 lands in bit 7 of an 8-bit mantissa: [1].[7 frac bits]
        val shiftAmt = Wire(UInt(log2Ceil(inW + 1).W))
        shiftAmt := Mux(highestIdx > 7.U, highestIdx - 7.U, 0.U)

        // Get mantissa after shifting
        val mantPre = x >> shiftAmt
        val frac = mantPre(6, 0)

        // Check for special cases
        val isInputZero = (exp === 0.U) && (fra === 0.U)
        val isInputSubnormal = (exp === 0.U) && (fra =/= 0.U)
        val isInputInf = (exp === 255.U) && (fra === 0.U)
        val isInputNaN = (exp === 255.U) && (fra =/= 0.U)

        // Assigning output
        val result = Wire(UInt(16.W))

        when (isInputZero || isInputSubnormal || isInputNaN) {
          // Assign 0
          result := Cat(neg, 0.U(expW.W), 0.U(fracW.W)) 
        } .elsewhen(isInputInf || expUnsigned >= 255.U) {
          // Assign Inf
          result := Cat(neg, Fill(expW, 1.U(1.W)), 0.U(fracW.W)) 
        } .otherwise {
          // Normal case
          result := Cat(neg, expUnsigned, frac)
        }

        result
    }

    def lutFixedToBf16Rcp(x: UInt, exp: UInt, fra: UInt,  lutValM: Int = 9, lutValN: Int = 12, neg: Bool = false.B): UInt = {
        val inW = lutValM + lutValN
        require(x.getWidth == inW, s"lutFixedToBf16 expects ${inW}-bit UInt input")

        val expW  = 8
        val fracW = 7
        val bias  = 127
        val twobias = 2 * bias

        // Index of highest 1 bit in x
        val highestIdx = Wire(UInt(log2Ceil(inW).W))
        highestIdx := 0.U
        for (i <- 0 until inW) {
            when(x(i)) { highestIdx := i.U }
        }

        // Since x represents real_value * 2^lutValN
        // Real exponent = highestIdx - lutValN
        val unbiasedExp = highestIdx.zext - lutValN.S
        val bfExpSigned = twobias.S + unbiasedExp - exp.zext
        val expUnsigned = bfExpSigned.asUInt(7,0)

        // Shift so leading 1 lands in bit 7 of an 8-bit mantissa: [1].[7 frac bits]
        val shiftAmt = Wire(UInt(log2Ceil(inW + 1).W))
        shiftAmt := Mux(highestIdx > 7.U, highestIdx - 7.U, 0.U)

        // Get mantissa after shifting
        val mantPre = x >> shiftAmt
        val frac = mantPre(6, 0)

        // Check for special cases
        val isInputZero = (exp === 0.U) && (fra === 0.U)
        val isInputSubnormal = (exp === 0.U) && (fra =/= 0.U)
        val isInputInf = (exp === 255.U) && (fra === 0.U)
        val isInputNaN = (exp === 255.U) && (fra =/= 0.U)

        val result = Wire(UInt(16.W))

        when (isInputInf || isInputNaN) {
          // Assign 0
          result := Cat(neg, 0.U(expW.W), 0.U(fracW.W)) 
        } .elsewhen(isInputZero || isInputSubnormal || expUnsigned >= 255.U) {
          // Assign Inf
          result := Cat(neg, Fill(expW, 1.U(1.W)), 0.U(fracW.W)) 
        } .otherwise {
          // Normal case
          result := Cat(neg, expUnsigned, frac)
        }

        result
    }

    def lutFixedToBf16Log(x: UInt, exp: UInt, fra: UInt,  lutValM: Int = 9, lutValN: Int = 12, neg: Bool = false.B): UInt = {
        val inW = lutValM + lutValN
        require(x.getWidth == inW, s"lutFixedToBf16 expects ${inW}-bit UInt input")

        val expW  = 8
        val fracW = 7
        val bias  = 127

        // Index of highest 1 bit in x
        val highestIdx = Wire(UInt(log2Ceil(inW).W))
        highestIdx := 0.U
        for (i <- 0 until inW) {
            when(x(i)) { highestIdx := i.U }
        }

        // Since x represents real_value * 2^lutValN
        // Real exponent = highestIdx - lutValN
        val unbiasedExp = highestIdx.zext - lutValN.S
        val bfExpSigned = unbiasedExp + bias.S
        val expUnsigned = bfExpSigned.asUInt(7,0)

        // Shift so leading 1 lands in bit 7 of an 8-bit mantissa: [1].[7 frac bits]
        val shiftAmt = Wire(UInt(log2Ceil(inW + 1).W))
        shiftAmt := Mux(highestIdx > 7.U, highestIdx - 7.U, 0.U)

        // Get mantissa after shifting
        val mantPre = x >> shiftAmt
        val frac = mantPre(6, 0)

        // Check for special cases
        val isZero = (x === 0.U)
        val isInputZero = ((exp === 0.U) && (fra === 0.U))
        val isInputSubnormal = (exp === 0.U) && (fra =/= 0.U)
        val isInputInf = (exp === 255.U) && (fra === 0.U)
        val isInputNaN = (exp === 255.U) && (fra =/= 0.U)

        val result = Wire(UInt(16.W))

        when (isInputNaN || isZero) {
          // Assign 0
          result := Cat(neg, 0.U(expW.W), 0.U(fracW.W)) 
        } .elsewhen(isInputZero || isInputSubnormal || isInputInf || expUnsigned >= 255.U) {
          // Assign Inf
          result := Cat(neg, Fill(expW, 1.U(1.W)), 0.U(fracW.W)) 
        } .otherwise {
          // Normal case
          result := Cat(neg, expUnsigned, frac)
        }

        result
    }
}