package fpex

import chisel3._
import chisel3.util._

class Exp2(fptype: FPType) extends Module {
  val w = fptype.wordWidth
  val expW = fptype.expWidth
  val sigW = fptype.sigWidth // Total precision (e.g., 24 for Float32)
  val bias = (1 << (expW - 1)) - 1

  val io = IO(new Bundle {
    val x      = Input(UInt(w.W))
    val valid  = Input(Bool())
    val result = Output(UInt(w.W))
  })

  // IEEE format FP 
  val fracBits = sigW - 1 // subtract hidden bit
  val sign     = io.x(w - 1)
  val rawExp   = io.x(w - 2, fracBits) 
  val mantissa = io.x(fracBits - 1, 0) 
  
  // Safe subtraction for exponent
  val adjExp = Cat(0.U(1.W), rawExp).asSInt - bias.S 
  val hasHiddenBit = rawExp.orR 
  val fullMant = Cat(hasHiddenBit, mantissa) // Format: Q1.fracBits (24 bits wide)

  val isPosExp = adjExp >= 0.S
  val absExp = Mux(isPosExp, adjExp.asUInt, (-adjExp).asUInt)
  val x_mag = Mux(isPosExp, fullMant << absExp, fullMant >> absExp)
  
  // Re-add sign 
  val x_val = Mux(sign, -x_mag.asSInt, x_mag.asSInt)
  
  // Integer and fraction
  val x_int  = x_val >> fracBits
  val x_frac = x_val(fracBits - 1, 0) 

  // Instantiate and use LUT values
  val addrBits = 8
  val lut = Module(new ExLUT(ports = 1, addrBits = addrBits, m = 1, n = fracBits))
  
  val lut_idx  = x_frac(fracBits - 1, fracBits - addrBits)
  val frac_rem = x_frac(fracBits - addrBits - 1, 0)

  lut.io.raddr(0) := lut_idx
  lut.io.ren(0)   := io.valid

  val y0 = lut.io.rdata(0)(0)
  val y1 = lut.io.rdata(1)(0)

  // interpolation
  val interpolation = (frac_rem * (y1 - y0)) >> (fracBits - addrBits)
  val res_mant_fixed = y0 + interpolation // Q1.fracBits format (1.0 to 1.999)

  // fixed point -> IEEE
  val res_sign = 0.U(1.W) 
  val res_exp_s = x_int + bias.S 
  val res_mant = res_mant_fixed(fracBits - 1, 0) //removes hidden bit here

  // handling other flows
  val maxExp = ((1 << expW) - 1).S
  val overflow  = res_exp_s >= maxExp
  val underflow = res_exp_s <= 0.S

  io.result := Mux(overflow, "h7F800000".U, // +Inf
               Mux(underflow, 0.U,          // 0.0 
               Cat(res_sign, res_exp_s(expW - 1, 0), res_mant)))
}