// package sp26FPUnits

// import chisel3._
// import chisel3.util._

// class Log2LUT(ports: Int, addrBits: Int, m: Int, n: Int,
//             min: Double = 1.0, max: Double = 2.0)
//   extends Module with LUTParams{
//   val io = IO(new Bundle {
//     val raddr = Input(Vec(ports, UInt(addrBits.W)))
//     val exp = Input(Vec(ports, UInt(8.W)))
//     val ren = Input(Vec(ports, Bool()))
//     val rdata = Output(Vec(ports, UInt(16.W)))
//   })

//   val entries = 1 << addrBits
//   val lut = VecInit.tabulate(entries) { i =>
//     val r = min + i.toDouble * (max - min) / entries
//     val v = math.log(r) / math.log(2.0)
//     val scaled = BigInt(math.round(v * (1 << n)))
//     scaled.U((m + n).W)
//   }

//   val bias  = 127.U
//   val res = Reg(Vec(ports, UInt((m + n).W)))

//   res.zip(io.raddr).zip(io.ren).zip(io.exp).foreach {
//     case (((results, raddr), en), exp) => when (en) { 
//       val isNeg = exp < bias
//       val absExp = Mux(isNeg, bias - exp, exp - bias) 
//       val realExpAsQmn = Wire(UInt((m + n).W))
//       realExpAsQmn := absExp << n
//       val base = Mux(isNeg, realExpAsQmn - lut(raddr), realExpAsQmn + lut(raddr))
//       val out = lutFixedToBf16Log(base, exp, raddr, m, n, isNeg).asUInt
//       results := out
//     }
//   }
  
//   io.rdata := res
// }