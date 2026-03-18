package sp26FPUnits

import chisel3._
import chisel3.util._
import fpex._

// Tanh block returns data after a 1 cycle latency (takes 2 cycles total to get results). TODO: turn IO into ready/valid and use the AtlasFPType.
// also experiment with pipelining

class TanhSyncBundle(alphaWidth: Int) extends Bundle {
  val sign        = Bool()
  val isZero      = Bool()
  val isSaturated = Bool()
  val alpha       = UInt(alphaWidth.W)
}

class Tanh(fptype: FPType) extends Module {
  val w = fptype.wordWidth
  val sigW = fptype.sigWidth
  val expW = fptype.expWidth
  val bias = fptype.bias
  val addrBits = fptype.lutAddrBits
  val n = fptype.lutValN   
  val n1    = fptype.qmnN     
  val m    = fptype.qmnM  
  
  val io = IO(new Bundle { 
    val x      = Input(UInt(w.W)) // bf16 atm
    val valid  = Input(Bool())
    val result = Output(UInt(w.W))
    val lutAddr = Output(UInt(fptype.lutAddrBits.W))
    val lutEn = Output(Bool())
    val lutVal1 = Input(UInt((m+n1).W))
    val lutVal2 = Input(UInt((m+n1).W))
  })

// Cycle 1: Read and decode

  // split input, converting floating point -> fixed point
  val sign = io.x(w - 1)
  val rawExp = io.x(w - 2, sigW - 1)
  val mantissa = io.x(sigW - 2, 0)

  val isZero = (rawExp === 0.U) && (mantissa === 0.U)
  
  // Subnormals have a hidden bit of 0
  val hiddenBit = Mux(rawExp === 0.U, 0.U(1.W), 1.U(1.W))
  val significand = Cat(hiddenBit, mantissa) // 1.mantissa or 0.mantissa
  
  // Zero-extend before casting to SInt so large exponents aren't treated as negative
  val zextRawExp = Cat(0.U(1.W), rawExp).asSInt
  val trueExp = Mux(rawExp === 0.U, (1 - bias).S, zextRawExp - bias.S) 

  val shiftAmt = trueExp + (n - (sigW - 1)).S
  val shiftAmtAbs = shiftAmt.abs.asUInt
  
  val xFixed = WireInit(0.U((n + 3).W))
  when(shiftAmt >= 0.S) {
    val shifted = significand << shiftAmtAbs
    val padded  = Cat(0.U((n + 3).W), shifted) 
    xFixed := padded(n + 2, 0)
  }
  .otherwise {
    val shifted = significand >> shiftAmtAbs
    val padded  = Cat(0.U((n + 3).W), shifted)
    xFixed := padded(n + 2, 0)
  }

  // LUT covering x = [0.0, 4.0] with 1 cycle latency
  // val lut = Module(new TanhLUT(ports = 1, addrBits = addrBits, m = 1, n = n, min = 0.0, max = 4.0))

  val intBitsToMap = 2
  val addr = xFixed(n + intBitsToMap - 1, n + intBitsToMap - addrBits)
  
  // Saturation check: if x >= 4.0
  val isSaturated = (trueExp >= 2.S) // exp >= 2 means val >= 4.0
  val safeAddr    = Mux(isSaturated, ((1 << addrBits) - 1).U, addr)

  // set addr to safeaddr and enable the read into the LUT
  // lut.io.raddr(0) := safeAddr
  // lut.io.ren(0)   := io.valid
  io.lutAddr := safeAddr
  io.lutEn := io.valid

  // Calculate interpolation fraction
  val fracBits = (n + intBitsToMap) - addrBits
  val alpha    = xFixed(fracBits - 1, 0)

  // Pack Cycle 1 data that skips the LUT
  val stage1Data = Wire(new TanhSyncBundle(fracBits))
  stage1Data.sign        := sign
  stage1Data.isZero      := isZero
  stage1Data.isSaturated := isSaturated
  stage1Data.alpha       := alpha

  // Use Pipe to delay it by 1 cycle, triggered by io.valid. 
  // This automatically provides stage2.valid and stage2.bits.
  val stage2 = Pipe(io.valid, stage1Data, 1)

// Cycle 2: Interpolation and Recode
  
  // 1-cycle later, read y0 (base value) and y1 (next value)
  // val y0 = lut.io.rdata(0)(0) 
  // val y1 = lut.io.rdata(1)(0) 
  val y0 = io.lutVal1.asSInt 
  val y1 = io.lutVal2.asSInt 

  // Prevent accidental unsigned underflow if y1 < y0 due to minor LUT precision rounding
  val yDiff = Mux(y1 >= y0, y1 - y0, 0.S(y0.getWidth.W))
  
  // Use stage2.bits.alpha to interpolate
  val interp = y0 + ((yDiff * stage2.bits.alpha) >> fracBits)
  // val interp = y0 + ((yDiff * stage2.bits.alpha.asSInt) >> fracBits)

  // Magnitude is either the interpolated value or 1.0 (if saturated)
  val oneFixed = (1.U << n)(n, 0) 
  val magFixed = WireInit(0.U((n + 1).W))
  
  // Use stage2.bits.isSaturated to bypass interpolation if out of bounds
  magFixed := Mux(stage2.bits.isSaturated, oneFixed, interp(n, 0)) 

  // fixed point -> floating point 
  val leadingZeros = PriorityEncoder(Reverse(magFixed)) 
  val msbPos       = (n.U) - leadingZeros 
  
  // Zero-extend msbPos before casting to SInt
  val msbPosSInt = Cat(0.U(1.W), msbPos).asSInt
  val expCalc    = msbPosSInt - n.S + bias.S
  val underflow  = expCalc <= 0.S 
  
  // Pad expCalc 
  val paddedExp = Cat(0.U(expW.W), expCalc.asUInt)
  val resRawExp = Mux(underflow || magFixed === 0.U, 0.U, paddedExp(expW - 1, 0))
  
  // Normalize mantissa: shift so MSB is at the hidden bit position
  val normShift  = leadingZeros 
  val normalized = (magFixed << normShift)(n, 0)
  
  // Safely extract mantissa by padding to avoid negative slice indices
  val paddedNormalized = Cat(normalized, 0.U(sigW.W))
  val resMantissa = paddedNormalized(n + sigW - 1, n + sigW - (sigW - 1))

  // Use stage2.bits.isZero to forward zero cases correctly
  val finalRawExp   = Mux(stage2.bits.isZero, 0.U, resRawExp)
  val finalMantissa = Mux(stage2.bits.isZero || underflow || magFixed === 0.U, 0.U, resMantissa)

  // Re-attach the delayed sign bit
  io.result := Cat(stage2.bits.sign, finalRawExp(expW - 1, 0), finalMantissa)
}