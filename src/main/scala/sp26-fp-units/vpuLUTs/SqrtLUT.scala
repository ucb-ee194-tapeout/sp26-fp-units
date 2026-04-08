package sp26FPUnits

import chisel3._
import chisel3.util._

class SqrtLUT(ports: Int, addrBits: Int, m: Int, n: Int, min: Double = 1.0, max: Double = 2.0) extends Module with LUTParams{
  val io = IO(new Bundle {
    val raddr = Input(Vec(ports, UInt(addrBits.W)))
    val exp = Input(Vec(ports, UInt(8.W)))
    val oddExp = Input(Vec(ports, Bool()))
    val ren = Input(Vec(ports, Bool()))
    val rdata = Output(Vec(ports, UInt(16.W)))
  })

  val maxVal = BigInt(Math.round(Math.sqrt(max) * (1 << n)))

  val entries = 1 << addrBits
  val lut = VecInit.tabulate(entries) { i =>
    val r = min + i.toDouble * (max - min) / entries
    val v = Math.sqrt(r)
    val scaled = BigInt(math.round(v * (1 << n)))
    scaled.U((m + n).W)
  }

  val res = Reg(Vec(ports, UInt(16.W)))

  val maxAddr = ((1 << addrBits) - 1).U
  
  res.zip(io.raddr).zip(io.ren).zip(io.oddExp).zip(io.exp).foreach {
    case ((((results, raddr), en), oddExp), exp) => when (en) { 
      val shiftedFull = ((lut(raddr).zext * maxVal.U) >> n).asUInt
      val shifted = shiftedFull((m + n - 1), 0)
      val base = Mux(oddExp, shifted, lut(raddr))
      val out = lutFixedToBf16Sqrt(base, exp, raddr, m, n, false.B).asUInt
      results := out
    }
  }
  io.rdata := res
}