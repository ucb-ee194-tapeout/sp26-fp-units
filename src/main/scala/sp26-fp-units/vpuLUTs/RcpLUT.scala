package sp26FPUnits

import chisel3._
import chisel3.util._

class RcpLUT(ports: Int, addrBits: Int, m: Int, n: Int,
            min: Double = 1.0, max: Double = 2.0)
  extends Module with LUTParams{
  val io = IO(new Bundle {
    val raddr = Input(Vec(ports, UInt(addrBits.W)))
    val exp = Input(Vec(ports, UInt(8.W)))
    val neg = Input(Vec(ports, Bool()))
    val ren = Input(Vec(ports, Bool()))
    val rdata = Output(Vec(ports, UInt(16.W)))
  })

  val entries = 1 << addrBits
  val lut = VecInit.tabulate(entries) { i =>
    val r = min + i.toDouble * (max - min) / entries
    val v = 1 / r
    val scaled = BigInt(math.round(v * (1 << n)))
    scaled.U((m + n).W)
  }

  val res = Reg(Vec(ports, UInt((m + n).W)))

  res.zip(io.raddr).zip(io.ren).zip(io.neg).zip(io.exp).foreach {
    case ((((results, raddr), en), neg), exp) => when (en) { 
      val out = lutFixedToBf16Rcp(lut(raddr), exp, raddr, m, n, neg).asUInt
      results := out
    }
  }
  
  io.rdata := res
}