package atlas.vector

import chisel3._
import chisel3.util._
import fpex._
import sp26FPUnits._
import sp26FPUnits.hardfloat._      
import sp26FPUnits.hardfloat.consts._

// Input bundles
class MaxReduReq(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val tag = UInt(tagWidth.W)
    val whichBank = UInt(5.W)
    val wRow= UInt(7.W)
    val aVec = Vec(numLanes, UInt(wordWidth.W))
}

// Output bundles
class MaxReduResp(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val tag = UInt(tagWidth.W)
    val whichBank = UInt(5.W)
    val wRow= UInt(7.W)
    val result = Vec(numLanes, UInt(wordWidth.W))
}


class MaxRedu(fptype: FPType, numLanes: Int = 16, tagWidth: Int = 8) extends Module {
  val w = fptype.wordWidth
  val expW = fptype.expWidth
  val fracW = w - 1 - expW // Width of the mantissa/fraction (excluding hidden bit)

  val io = IO(new Bundle {
    val req  = Flipped(Decoupled(new MaxReduReq(w, numLanes, tagWidth)))
    val resp = Decoupled(new MaxReduResp(w, numLanes, tagWidth))
  })

  //Helper function: Compares two BF16 and returns the maximum.
  def maxBf16(a: UInt, b: UInt): UInt = {
    val aIsNaN = a(w - 2, fracW) === ((1 << expW) - 1).U && a(fracW - 1, 0) =/= 0.U
    val bIsNaN = b(w - 2, fracW) === ((1 << expW) - 1).U && b(fracW - 1, 0) =/= 0.U

    // split sign and magnitude 
    val aSign = a(w - 1)
    val bSign = b(w - 1)
    val aMag  = a(w - 2, 0)
    val bMag  = b(w - 2, 0)

    // If negative, invert magnitude. If positive, leave as is.
    val aSort = Mux(aSign, Cat(1.U(1.W), ~aMag).asSInt, Cat(0.U(1.W), aMag).asSInt)
    val bSort = Mux(bSign, Cat(1.U(1.W), ~bMag).asSInt, Cat(0.U(1.W), bMag).asSInt)

    // Find greater value and return max
    val aGreater = aSort > bSort
    Mux(aIsNaN, b, Mux(bIsNaN, a, Mux(aGreater, a, b)))
  }

  // Regs to hold inputs  
  val validReg = RegInit(false.B)
  val reqReg = Reg(new MaxReduReq(w, numLanes, tagWidth))

  io.req.ready := !validReg || io.resp.ready
  // populate registers when firing
  when(io.req.fire) { 
    reqReg := io.req.bits
    validReg := true.B
  }.elsewhen(io.resp.fire) {
    validReg := false.B
  }

  val result = reqReg.aVec.reduceTree((a, b) => maxBf16(a, b))
  val zeroVal  = 0.U(w.W)

  // Tree Reduction 
  // reduceTree recursively groups the vector into a balanced binary tree,
  // minimizing the critical path delay from O(N) to O(log2(N)).
  io.resp.valid := validReg
  io.resp.bits.tag := reqReg.tag
  io.resp.bits.whichBank := reqReg.whichBank
  io.resp.bits.wRow := reqReg.wRow
  io.resp.bits.result := VecInit(Seq(result) ++ Seq.fill(numLanes - 1)(zeroVal)) 
}