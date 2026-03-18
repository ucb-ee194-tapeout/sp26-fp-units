package sp26FPUnits

import chisel3._
import chisel3.util._
import fpex._
import sp26FPUnits._
import sp26FPUnits.hardfloat._      
import sp26FPUnits.hardfloat.consts._

// Input bundle for Relu
class ReluReq(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val tag = UInt(tagWidth.W)
    val laneMask = UInt(numLanes.W)
    val whichBank = UInt(4.W) 
    val wRow = UInt(8.W)
    val aVec = Vec(numLanes, UInt(wordWidth.W))
}

// Output bundle for Relu
class ReluResp(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val tag = UInt(tagWidth.W)
    val laneMask = UInt(numLanes.W)
    val whichBank = UInt(4.W) 
    val wRow = UInt(8.W)      
    val result = Vec(numLanes, UInt(wordWidth.W))
}

class Relu(BF16T: FPType, numLanes: Int = 16, tagWidth: Int = 8) extends Module {
    val io = IO(new Bundle {
        val req = Flipped(Decoupled(new ReluReq(BF16T.wordWidth, numLanes, tagWidth)))
        val resp = Decoupled(new ReluResp(BF16T.wordWidth, numLanes, tagWidth))
    })

    // latch inputs
    val validReg = RegInit(false.B)
    val reqReg   = Reg(new ReluReq(BF16T.wordWidth, numLanes, tagWidth))

    // Ready if our register is empty or if the downstream is currently accepting the data
    io.req.ready := !validReg || io.resp.ready

    // Update validReg: 
    // Set to true if a new request comes in. 
    // Set to false if we are sending data and no new request is arriving.
    when (io.req.fire) {
        validReg := true.B
        reqReg   := io.req.bits
    } .elsewhen (io.resp.ready) {
        validReg := false.B
    }

    // LOGIC for relu operation
    for (i <- 0 until numLanes) {
        val inVal = reqReg.aVec(i)
        
        // Check the sign bit - msb
        // If MSB is 1, value is negative -> return 0, otherwise pass value through
        val isNegative = inVal(BF16T.wordWidth - 1)
        io.resp.bits.result(i) := Mux(isNegative, 0.U(BF16T.wordWidth.W), inVal)
    }
    
    // output signals
    io.resp.valid := validReg
    io.resp.bits.tag := reqReg.tag
    io.resp.bits.laneMask := reqReg.laneMask
    io.resp.bits.whichBank := reqReg.whichBank
    io.resp.bits.wRow := reqReg.wRow
}