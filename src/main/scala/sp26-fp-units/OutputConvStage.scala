/*
OutputConvStage.scala: optional post-accumulation output conversion.

Behavior:
  - OutBF16: sanitize BF16, then pass it through
  - OutE4M3: sanitize BF16, scale by an exact power-of-two factor, then cast to OCP E4M3

Sanitization policy (applied once, in BF16):
  - BF16 NaN      -> zero
  - BF16 subnorm  -> zero
  - BF16 zero     -> zero
  - BF16 +/-inf   -> largest finite BF16 with same sign

Scaling factor:
  - scaleExp is a signed 8-bit exponent offset
  - scaled_value = bf16In * 2^(scaleExp)

E4M3 conversion policy:
  - E4M3 subnorm outputs are NOT generated; underflow clamps to zero
  - E4M3 NaNs are NOT generated
  - E4M3 overflow clamps to max finite with same sign

Notes:
  - This stage is purely combinational.
  - External output width remains p.outputFmt.ieeeWidth (BF16 width = 16).
    In OutE4M3 mode, the E4M3 value is placed in bits [7:0] and the upper bits are zero.
*/

package sp26FPUnits

import chisel3._
import chisel3.util._
import atlas.common.InnerProductTreeParams

object OutputFmtSel extends ChiselEnum {
  val OutBF16, OutE4M3 = Value
}

class BF16ScaleToE4M3(p: InnerProductTreeParams) extends Module {
  val io = IO(new Bundle {
    val bf16In   = Input(UInt(p.outputFmt.ieeeWidth.W))  // sanitized BF16
    val scaleExp = Input(SInt(8.W))                      // exact power-of-two scale: multiply by 2^scaleExp
    val e4m3Out  = Output(UInt(p.inputFmt.ieeeWidth.W))  // E4M3
  })

  require(p.outputFmt == AtlasFPType.BF16, "BF16ScaleToE4M3 expects BF16 outputFmt")
  require(p.inputFmt  == AtlasFPType.E4M3, "BF16ScaleToE4M3 expects E4M3 inputFmt")

  def packE4M3(sign: UInt, exp: UInt, mant: UInt): UInt =
    Cat(sign(0), exp(3, 0), mant(2, 0))

  val E4M3_MAX_POS = "h7e".U(8.W) // +max finite
  val E4M3_MAX_NEG = "hfe".U(8.W) // -max finite

  // Round 8-bit significand [1.frac(7)] right by 4 with RNE.
  // Input:  x in [128, 255]
  // Output: rounded in [8, 16]
  def roundRightShift4RNE(x: UInt): UInt = {
    val trunc  = Cat(0.U(1.W), x(7, 4)) // 5 bits
    val guard  = x(3)
    val sticky = x(2, 0).orR
    val lsb    = trunc(0)
    val inc    = guard && (sticky || lsb)
    trunc + inc
  }

  val sign     = io.bf16In(15)
  val expBF16  = io.bf16In(14, 7)
  val fracBF16 = io.bf16In(6, 0)

  val isZero = (expBF16 === 0.U) && (fracBF16 === 0.U)
  val isSub  = (expBF16 === 0.U) && (fracBF16 =/= 0.U)
  val isInf  = (expBF16 === "hff".U) && (fracBF16 === 0.U)
  val isNaN  = (expBF16 === "hff".U) && (fracBF16 =/= 0.U)

  val out = Wire(UInt(8.W))
  out := 0.U

  when(isZero || isSub || isNaN) {
    out := 0.U
  } .elsewhen(isInf) {
    out := Mux(sign.asBool, E4M3_MAX_NEG, E4M3_MAX_POS)
  } .otherwise {
    val unbExp = expBF16.zext - 127.S(10.W)

    val scaleExpWide = Wire(SInt(10.W))
    scaleExpWide := io.scaleExp.pad(10)

    val scaledUnbExp = unbExp + scaleExpWide

    val mant8        = Cat(1.U(1.W), fracBF16)    // 8 bits
    val roundedNorm  = roundRightShift4RNE(mant8) // 5 bits
    val normCarry    = (roundedNorm === 16.U(5.W))

    val finalUnbExp = Wire(SInt(10.W))
    finalUnbExp := scaledUnbExp
    when(normCarry) {
      finalUnbExp := scaledUnbExp + 1.S
    }

    val roundedMinus8 = Wire(UInt(5.W))
    roundedMinus8 := roundedNorm - 8.U(5.W)

    val normMant = Wire(UInt(3.W))
    normMant := roundedMinus8(2, 0)
    when(normCarry) {
      normMant := 0.U
    }

    when(finalUnbExp > 8.S) {
      out := Mux(sign.asBool, E4M3_MAX_NEG, E4M3_MAX_POS)
    } .elsewhen(finalUnbExp >= (-6).S) {
      val expE4Wide = Wire(UInt(5.W))
      expE4Wide := (finalUnbExp + 7.S).asUInt
      out := packE4M3(sign, expE4Wide(3, 0), normMant)
    } .otherwise {
      out := 0.U
    }
  }

  io.e4m3Out := out
}

class OutputConvStage(p: InnerProductTreeParams) extends Module {
  val io = IO(new Bundle {
    val bf16In    = Input(UInt(p.outputFmt.ieeeWidth.W))
    val outFmtSel = Input(OutputFmtSel())
    val scaleExp  = Input(SInt(8.W))

    // BF16-width output container:
    //   OutBF16: full 16-bit sanitized BF16
    //   OutE4M3: low 8 bits hold E4M3, upper bits are zero
    val out       = Output(UInt(p.outputFmt.ieeeWidth.W))
  })

  require(p.outputFmt == AtlasFPType.BF16, "OutputConvStage expects BF16 outputFmt")
  require(p.inputFmt  == AtlasFPType.E4M3, "OutputConvStage expects E4M3 inputFmt")

  val sign     = io.bf16In(15)
  val expBF16  = io.bf16In(14, 7)
  val fracBF16 = io.bf16In(6, 0)

  val isSub = (expBF16 === 0.U) && (fracBF16 =/= 0.U)
  val isInf = (expBF16 === "hff".U) && (fracBF16 === 0.U)
  val isNaN = (expBF16 === "hff".U) && (fracBF16 =/= 0.U)

  val BF16_MAX_POS = "h7f7f".U(16.W)
  val BF16_MAX_NEG = "hff7f".U(16.W)

  val bf16Sanitized = Wire(UInt(p.outputFmt.ieeeWidth.W))
  bf16Sanitized := io.bf16In
  when(isSub || isNaN) {
    bf16Sanitized := 0.U
  } .elsewhen(isInf) {
    bf16Sanitized := Mux(sign.asBool, BF16_MAX_NEG, BF16_MAX_POS)
  }

  val cast = Module(new BF16ScaleToE4M3(p))
  cast.io.bf16In   := bf16Sanitized
  cast.io.scaleExp := io.scaleExp

  io.out := bf16Sanitized
  when(io.outFmtSel === OutputFmtSel.OutE4M3) {
    io.out := Cat(0.U((p.outputFmt.ieeeWidth - p.inputFmt.ieeeWidth).W), cast.io.e4m3Out)
  }
}
