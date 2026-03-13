package atlas.vector

import chisel3._
import chisel3.util._
import fpex._
import sp26FPUnits._
import sp26FPUnits.hardfloat._      
import sp26FPUnits.hardfloat.consts._

// Input bundles
class SquareCubeReq(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val tag = UInt(tagWidth.W)
    val whichBank = UInt(5.W)
    val wRow= UInt(7.W)
    val roundingMode = UInt(3.W)
    val laneMask = UInt(numLanes.W)
    val aVec = Vec(numLanes, UInt(wordWidth.W))
    val isCube = Bool()
}

// Output bundles
class SquareCubeResp(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val tag = UInt(tagWidth.W)
    val whichBank = UInt(5.W)
    val wRow= UInt(7.W)
    val laneMask = UInt(numLanes.W)
    val result = Vec(numLanes, UInt(wordWidth.W))
}

class SquareCubeRec(BF16T: FPType, numLanes: Int = 16, tagWidth: Int = 16) extends Module with HasPipelineParams {
    val io = IO(new Bundle {
        val req = Flipped(Decoupled(new SquareCubeReq(BF16T.wordWidth, numLanes, tagWidth)))
        val resp = Decoupled(new SquareCubeResp(BF16T.wordWidth, numLanes, tagWidth))
    })
    
    // MulRec instance
    val mul = Module(new MulRec(BF16T, numLanes = numLanes, tagWidth = tagWidth))
    
    // We repeat RUN for cube
     object SCState extends ChiselEnum { 
        val IDLE, RUN, DONE = Value 
    }
    
    val state = RegInit(SCState.IDLE)
    val reqReg = Reg(new SquareCubeReq(BF16T.wordWidth, numLanes, tagWidth))

    // holds a*a for cube only in between Mul1->Mul2
    val tmp = Reg(Vec(numLanes, UInt(BF16T.wordWidth.W))) 
    // phase = 0 => doing a*a, phase = 1 => doing tmp*a
    val phase  = RegInit(0.U(1.W))
    // issued = have we successfully fired mul.req for current phase?
    val issued = RegInit(false.B)


    // Initializing the states
    io.req.ready  := (state === SCState.IDLE)
    io.resp.valid := (state === SCState.DONE)
    io.resp.bits  := 0.U.asTypeOf(io.resp.bits)

    mul.io.req.valid := false.B
    mul.io.req.bits  := 0.U.asTypeOf(mul.io.req.bits)
    mul.io.resp.ready := false.B
    
    // getting actual req 
    when (state === SCState.IDLE) {
        when (io.req.fire) {
            reqReg := io.req.bits
            state := SCState.RUN
        }
    }


    // Output when done -> we have this state instead of going straight back to idle since we don't want to accept a new inst if we are still holding old value
    when (state === SCState.DONE) {
        io.resp.bits.tag       := reqReg.tag
        io.resp.bits.whichBank := reqReg.whichBank
        io.resp.bits.wRow      := reqReg.wRow
        io.resp.bits.laneMask  := reqReg.laneMask
        io.resp.bits.result    := tmp

        when (io.resp.fire) {
            state := SCState.IDLE
        }
    }

    def driveMulReq(a: Vec[UInt], b: Vec[UInt]): Unit = {
        mul.io.req.bits.tag          := reqReg.tag
        mul.io.req.bits.whichBank    := reqReg.whichBank
        mul.io.req.bits.wRow         := reqReg.wRow
        mul.io.req.bits.roundingMode := reqReg.roundingMode
        mul.io.req.bits.laneMask     := reqReg.laneMask
        mul.io.req.bits.aVec         := a
        mul.io.req.bits.bVec         := b
    }

    // RUN: issue request (once) then wait for response
    when (state === SCState.RUN) {
        val aOp = Mux(phase === 0.U, reqReg.aVec, tmp)
        val bOp = reqReg.aVec

        // 1) issue mul request for this phase until it fires
        when (!issued) {
            mul.io.req.valid := true.B
            driveMulReq(aOp, bOp)
            when (mul.io.req.fire) {
                issued := true.B
            }
        }

        // 2) once issued, wait for mul response
        when (issued) {
            mul.io.resp.ready := true.B
            when (mul.io.resp.fire) {
                tmp := mul.io.resp.bits.result

                when (!reqReg.isCube) {
                    // square completes after first response
                    state := SCState.DONE
                }.otherwise {
                    when (phase === 0.U) {
                        // cube needs second multiply
                        phase  := 1.U
                        issued := false.B
                    }.otherwise {
                        // cube completes after second response
                        state := SCState.DONE
                    }
                }
            }
        }
    }
    

}
