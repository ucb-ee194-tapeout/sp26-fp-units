package sp26FPUnits

import chisel3._
import chisel3.util._

/* TANH LUT 

Uses Scala math to generate hardwires (inside Vec entries). Returns two entries: The immediate requested and its next entry. 1-cycle latency.

ports       = Number of ports that can read from the LUT (i think)
addrBits    = Size of the LUT 2^addrBits
min         = Minimum domain value
max         = Maximum domain value 
n           = Scaling factor which defines how many bits we shift to the right to convert the fraction into an integer that the hardware can store.
m           = Number of bits reserved for the whole number part of the value
m + n       = Total word-width

*/

class TanhLUT(ports: Int, addrBits: Int, m: Int, n: Int,
    min: Double = 0.0, max: Double = 4.0) extends Module {

    val io = IO(new Bundle {
        val raddr = Input(Vec(ports, UInt(addrBits.W)))
        val ren = Input(Vec(ports, Bool()))
        val rdata = Output(Vec(2, Vec(ports, UInt((m + n).W))))
    })

    // Populating the LUT with Scala Math values of tanh 
    val entries = 1 << addrBits
    val lut = VecInit.tabulate(entries) { i =>
        val r = min + i.toDouble * (max - min) / entries
        val v = math.tanh(r)
        val scaled = BigInt(math.round(v * (1 << n)))
        scaled.U((m + n).W)
    }

    val res = Reg(Vec(2, Vec(ports, UInt((m + n).W))))

    val maxAddr = ((1 << addrBits) - 1).U
    val raddrNxt = io.raddr.map(raddr => Mux(raddr === maxAddr, raddr, raddr + 1.U))
    res.zip(Seq(io.raddr, raddrNxt)).foreach {
        case (results, raddrs) => results.zip(raddrs).zip(io.ren).foreach {
        case ((result, raddr), en) => when (en) { result := lut(raddr) }
        }
    }
    io.rdata := res
}