package atlas.vector

import chisel3._
import chisel3.util._
import fpex._
import sp26FPUnits._
import sp26FPUnits.hardfloat._      
import sp26FPUnits.hardfloat.consts._

// A BF16-> FP8 (E4M3) Converter: NOTE DOES NOT WORK WITH LANE MASK RN

// Input bundles
class FP8Req(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val tag = UInt(tagWidth.W)
    val whichBank = UInt(5.W)
    val wRow= UInt(7.W)
    val laneMask = UInt(numLanes.W) 

    val expShift = SInt(8.W) // exp shift so it doesn't disappear out of range (underflow)
    val leftAlign = Bool()   // true -> fp8 in the [15:8], false -> fp8 in [7:0]
    val xVec = Vec(numLanes, UInt(wordWidth.W))
}

// Output bundles
class FP8Resp(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val tag = UInt(tagWidth.W)
    val whichBank = UInt(5.W)
    val wRow= UInt(7.W)
    val laneMask = UInt(numLanes.W)
    val result = Vec(numLanes, UInt(wordWidth.W))
}

class FP8(BF16T: FPType, numLanes: Int = 16, tagWidth: Int = 16) extends Module with HasPipelineParams {
    val io = IO(new Bundle {
        val req = Flipped(Decoupled(new FP8Req(BF16T.wordWidth, numLanes, tagWidth)))
        val resp = Decoupled(new FP8Resp(BF16T.wordWidth, numLanes, tagWidth))
    })
    
    // E4M3 constants to reference
    val fp8ExpBits  = 4
    val fp8MantBits = 3
    val fp8Bias     = 7

    // Initialiszing states (right now its all combinational)
    io.req.ready  := io.resp.ready
    io.resp.valid := io.req.valid
    
    // right now just.. doing not much 
    io.resp.bits.tag       := io.req.bits.tag
    io.resp.bits.whichBank := io.req.bits.whichBank
    io.resp.bits.wRow      := io.req.bits.wRow
    io.resp.bits.laneMask  := io.req.bits.laneMask

    val outVec = Wire(Vec(numLanes, UInt(BF16T.wordWidth.W)))
    val laneEnable = VecInit((io.req.bits.laneMask & VecInit.fill(numLanes)(io.req.fire).asUInt).asBools)
    
    for (i <- 0 until numLanes) {
        val bf16 = io.req.bits.xVec(i)

        // unpacking our bits
        val sign = bf16(15)
        val expBF = bf16(14, 7)
        val mantBF = bf16(6, 0)

        // Both of these will flush to 0 as of now, but writing this for more flexibility if needed
        // val isZero = (expBF === 0.U) && (mantBF === 0.U)
        // val isDenormalized = (expBF === 0.U) && (mantBF =/= 0.U)

        // val flushToZero = isZero || isDenormalized

        // BF16: sign[15] exp[14:7] frac[6:0], bias = 127 
        // FP8: E4M3: bias 7 
        // Essentially: (exp_bf16 - 127 + 7) + shift 
        val expAdjusted = expBF.asSInt - 120.S + io.req.bits.expShift

        // Clamp exponent to E4M3 range: (0,15) - convert it to SInt so it doesn't wrap around
        val expFP8 = MuxCase(expAdjusted.asUInt(3, 0), Seq(
            (expBF === 0.U && mantBF === 0.U) -> 0.U(4.W), 
            (expAdjusted <= 0.S) -> 0.U(4.W), 
            (expAdjusted >= 15.S) -> 15.U(4.W) 
        ))

        // Mantisa truncates : 7 bits -> 3 bits
        val mantFP8 = mantBF(6,4) 

        // pack it all back-> sign is the same still
        val fp8Pack = Cat(sign, expFP8, mantFP8)

        // align it properly 
        outVec(i) := Mux(io.req.bits.leftAlign, 
                         Cat(fp8Pack, 0.U(8.W)), 
                         Cat(0.U(8.W), fp8Pack))
    }
    io.resp.bits.result := outVec

}
