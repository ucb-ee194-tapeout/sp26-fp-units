package atlas.vector

import chisel3._
import chisel3.util._
import fpex._

class Log2(fptype: FPType) extends Module {
  val w    = fptype.wordWidth
  val expW = fptype.expWidth
  val sigW = fptype.sigWidth 
  val n    = fptype.qmnN     
  val m    = fptype.qmnM     

  val io = IO(new Bundle {
    val x      = Input(UInt(w.W)) 
    val valid  = Input(Bool())
    val result = Output(UInt(w.W)) 
    val lutAddr = Output(UInt(fptype.lutAddrBits.W))
    val lutEn = Output(Bool())
    val lutVal1 = Input(UInt((m+n).W))
    val lutVal2 = Input(UInt((m+n).W))
  })

  val rawExp   = io.x(w - 2, sigW - 1)
  val mantissa = io.x(sigW - 2, 0)
  val trueExp  = Cat(0.U(1.W), rawExp).asSInt - fptype.bias.S

  // Use 1 port; LogLUT internally provides rdata(0) and rdata(1) for interpolation
  // val lut = Module(new LogLUT(ports = 1, addrBits = fptype.lutAddrBits, m = 1, n = n))
  
  val fracBits = (sigW - 1) - fptype.lutAddrBits
  val addr = mantissa(sigW - 2, fracBits)
  // lut.io.raddr(0) := addr
  // lut.io.ren(0)   := io.valid
  io.lutAddr := addr
  io.lutEn := io.valid

  // We must delay the Exponent and Alpha so they match the 1-cycle LUT delay
  val trueExp_reg = RegEnable(trueExp, io.valid)
  val alpha_wire  = if (fracBits > 0) mantissa(fracBits - 1, 0) else 0.U
  val alpha_reg   = RegEnable(alpha_wire, io.valid)

  // -Interpolation and combining
  // val y0 = lut.io.rdata(0)(0).asSInt 
  // val y1 = lut.io.rdata(1)(0).asSInt 
  val y0 = io.lutVal1.asSInt 
  val y1 = io.lutVal2.asSInt 

  val interp = if (fracBits > 0) {
    y0 + (((y1 - y0) * alpha_reg.asSInt) >> fracBits)
  } else {
    y0
  }

  // logResult (Fixed Point Qm.n)
  val logResult = (trueExp_reg << n) + interp

// Re-Normalization to BF16 ---
  val totalWidth = m + n + 1
  val resSign = logResult < 0.S
  val absVal = Mux(resSign, (-logResult).asUInt, logResult.asUInt)

  val absValFixed = Wire(UInt(totalWidth.W))
  absValFixed := absVal

  // Find MSB for the new Floating Point exponent
  val leadingZeros = PriorityEncoder(Reverse(absValFixed))
  val msbPos = (totalWidth - 1).U - leadingZeros 

  val newTrueExp = Cat(0.U(1.W), msbPos).asSInt - n.S

  val newRawExp  = (newTrueExp + fptype.bias.S).asUInt

  // Shift to align mantissa
  val shiftAmount = Mux(msbPos >= (sigW - 1).U, msbPos - (sigW - 1).U, (sigW - 1).U - msbPos)
  val normalized = Mux(msbPos >= (sigW - 1).U, absVal >> shiftAmount, absVal << shiftAmount)

  val newFraction = normalized(sigW - 2, 0) 
  
  val isZero = absVal === 0.U
  val finalExp = Mux(isZero, 0.U, newRawExp(expW - 1, 0))
  val finalSig = Mux(isZero, 0.U, newFraction)

  io.result := Cat(resSign, finalExp, finalSig)
}