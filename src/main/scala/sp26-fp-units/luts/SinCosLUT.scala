package atlas.vector

import chisel3._
import chisel3.util._
import atlas.common.VPUParams
// import hardfloat._  //sophia comment: taking this one out bc folder is gone 
import fpex._
import sp26FPUnits.hardfloat._
import sp26FPUnits._

// LUT used for both sin and cos, since we can leverage the identity sin(x) = cos(pi/2 - x)
class SinCosLUT(ports: Int, addrBits: Int, m: Int, n: Int,
    min: Double = 0.0, max: Double = Math.PI / 2) extends Module {
    val io = IO(new Bundle {
        val raddr = Input(Vec(ports, UInt(addrBits.W)))
        val isCos = Input((Vec(ports, Bool()))) // 1 = cos, 0 = sin
        val ren = Input(Vec(ports, Bool()))
        val rdata = Output(Vec(2, Vec(ports, UInt((m + n).W))))
    })

    // Populating the LUT with pre-computed values of sin
    // Note that sin(0) / cos(pi/2) and sin(pi/2) / cos(0) are edge cases that we handle else where
    val entries = 1 << addrBits
    val lut = VecInit.tabulate(entries) { i =>
        val r = min + i.toDouble * (max - min) / entries
        val v = math.sin(r)
        val scaled = BigInt(math.round(v * (1 << n)))
        scaled.U((m + n).W)
    }
    val res = RegInit(VecInit(Seq.fill(2)(VecInit(Seq.fill(ports)(0.U((m + n).W))))))


    /* 
     * Since LUT is computed based on sine function, we need to leverage the identity sin(x) = cos(pi/2 - x) to get cosine values.
     * sin(x) = cos(pi/2 - x) => first element of the LUT is sin(0)  = 0 for sine whereas it is cos(0) = 1 for cosine, 
     *                           last element of the LUT is sin(pi/2) = 1 for sine whereas it is cos(pi/2) = 0 for cosine.
     * Example:
     *  raddr = 1, raddrNxt = 2 
     *  Sin: raddr_sin = 1, raddrNxt_sine = raddr_sin + 1                        => second and third element of the LUT for sine
     *  Cos: raddr_cos = maxAddr - raddr_sin, raddrNxt = maxAddr - raddr_sin - 1 => second and third last element of the LUT for cosine
     */

    // Note: sin(p/2) and cos(0) are not stored in the LUT. They should be handled as special cases in the calling modules.
    // Sin top addr is pi/2 which is not reachable in LUT
    // Cos bottom addr is 0 which is not reachable in LUT
    // Sin: (a, a + 1) where a + 1 might be out of range when a is max
    // Cos: (maxAddr - a + 1, maxAddr - a) where maxAddr - a + 1 might be out of range when a is 0
    val scaled = BigInt(math.round(1 << n)).U((m + n).W) // This is the value for sin(pi/2) and cos(0)
    val maxAddr = ((1 << addrBits) - 1).U
    val is_sin_top_addr = VecInit(io.raddr.zip(io.isCos).map{case (a, isCos) => (a === maxAddr && !isCos)}) // Use on second addr
    val is_cos_bot_addr = VecInit(io.raddr.zip(io.isCos).map{case (a, isCos) => (a === 0.U && isCos)})  // Use on first addr
    val raddrReal = VecInit(io.raddr.zip(io.isCos).map { case (a, isCos) => Mux(isCos, maxAddr - a + 1.U, a)})
    val raddrNxt = VecInit(io.raddr.zip(io.isCos).map { case (a, isCos) => Mux(isCos, maxAddr - a, a + 1.U)})
    // res.zip(Seq(raddrReal, raddrNxt)).foreach {
    //     case (results, raddrs) => results.zip(raddrs).zip(io.ren).foreach {
    //     case ((result, raddr), en) => when (en) { result := lut(raddr) }
    //     }
    // }
    res(0).zip(raddrReal).zip(is_cos_bot_addr).foreach { case ((r, addr), cb) => r := Mux(cb, scaled, lut(addr))}   
    res(1).zip(raddrNxt).zip(is_sin_top_addr).foreach { case ((r, addr), st) => r := Mux(st, scaled, lut(addr))}

    io.rdata := res
}