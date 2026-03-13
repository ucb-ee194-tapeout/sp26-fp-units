package atlas.vector

import chisel3._
import chisel3.util._
import fpex.hardfloat._
import fpex._
import sp26FPUnits._

trait HasSinCosParams {
    def numFP16Lanes = 16
    def tagWidth = 16

    // Combiantional logic mask: select next value if (lane enabled and not back pressured), otherwise hold current value
    def maskLane[T <: Data](currVec: Vec[T], nxtVec: Vec[T], laneMask: Vec[Bool], bp: Bool): Vec[T] = {
        require(laneMask.length == currVec.length && currVec.length == nxtVec.length)
        VecInit(currVec.zip(nxtVec).zip(laneMask).map{case ((curr, nxt), en) => Mux(en && !bp, nxt, curr)})
    }

    // Sequential logic mask: update register with next value if (lane enabled and not back pressured), otherwise hold current value
    def maskLaneNext[T <: Data](bitsVec: Vec[T], laneMask: Vec[Bool], bp: Bool): Vec[T] = {
        require(laneMask.length == bitsVec.length)
        VecInit(bitsVec.zip(laneMask).map{case (bits, en) => RegEnable(bits, en && !bp)})
    }
}

// Input bundles
class SinCosReq(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val roundingMode = UInt(3.W)
    val tag = UInt(tagWidth.W)
    val whichBank = UInt(5.W)
    val wRow= UInt(7.W)
    val cos = Bool() // 1 = cos, 0 = sin
    val laneMask = UInt(numLanes.W)
    val xVec = Vec(numLanes, UInt(wordWidth.W))
}

// Output bundles
class SinCosResp(wordWidth: Int, numLanes: Int, tagWidth: Int) extends Bundle {
    val tag = UInt(tagWidth.W)
    val whichBank = UInt(5.W)
    val wRow= UInt(7.W)
    val laneMask = UInt(numLanes.W)
    val result = Vec(numLanes, UInt(wordWidth.W))
}

class SinCos(BF16T: FPType, numLanes: Int = 16, tagWidth: Int = 16) extends Module with HasSinCosParams {
    val io = IO(new Bundle {
        val req = Flipped(Decoupled(new SinCosReq(BF16T.wordWidth, numLanes, tagWidth)))
        val resp = Decoupled(new SinCosResp(BF16T.wordWidth, numLanes, tagWidth))
    })


    /* Initial setup:
     *  1. Instantiate the LUT
     *  2. Instatntiate the round to recoding modules
     *  3. Compute the number of bits needed for interpolation multiplication
     *  4. Compute the top endpoint of the LUT for saturation which is 1 shifted by lutValN
     *  5. Allow the input data to pass through if the respective lane is enabled
     *  6. Determine validity of cosine computation
     *  7. Values for determining quadrants
    */ 
    val lut = Module(new SinCosLUT(numLanes, BF16T.lutAddrBits, BF16T.lutValM, BF16T.lutValN))
    val roundToRecFn = Seq.fill(numLanes)(Module(new RoundRawFNToRecFN(BF16T.expWidth, BF16T.sigWidth, 0)))
    val rLowBits = BF16T.qmnN - BF16T.lutAddrBits
    val lutTopEndpoint =(BigInt(1) << BF16T.lutValN).U((BF16T.lutValM + BF16T.lutValN).W)
    val laneEnable = VecInit((io.req.bits.laneMask & VecInit.fill(numLanes)(io.req.fire).asUInt).asBools)
    val maskedXVec = io.req.bits.xVec.zip(laneEnable).map {
        case (x, en) => Mux (en, x, 0.U(BF16T.wordWidth.W))
    }
    val maskedCos = io.req.bits.cos && io.req.fire

    // Storing special angle values in QMN format for quadrant reduction and interpolation
    def bf16BitsFromFloat(f: Float): UInt =(java.lang.Float.floatToIntBits(f) >>> 16).U(16.W)
    def fpTobf(f: Float): Int = {
        val x = java.lang.Float.floatToRawIntBits(f)
        val lsb = (x >>> 16) & 1
        val roundingBias = 0x7FFF + lsb
        ((x + roundingBias) >>> 16) & 0xFFFF
    }

    /* Stage 0: 
     * 1. Raw floaw decomposition
     * 2. Check for special cases (NaN, Inf, zero) and compute early result and early termination signal accordingly
    */ 
    val rawFloatVec = VecInit(maskedXVec.map(x => rawFloatFromFN(BF16T.expWidth, BF16T.sigWidth, x)))
    val earlyResult = VecInit(rawFloatVec.map { x =>
        MuxCase(
            0.U(BF16T.wordWidth.W),
            Seq(
                x.isNaN -> Cat(x.sign, BF16T.nanExp, isSigNaNRawFloat(x), BF16T.nanSig),
                ((x.isZero) && maskedCos) -> BF16T.one,
                ((x.isZero) && !maskedCos) -> BF16T.zero,
                (x.isInf) -> Cat(0.U(1.W), BF16T.infinity)
            )
        )
    })
    val earlyTerminate = VecInit(rawFloatVec.map { x => x.isInf || x.isZero || x.isNaN})


    // States for pipelining
    class CommonStageState extends Bundle {
        val valid = Bool()
        val req = chiselTypeOf(io.req.bits)
        val laneEn = chiselTypeOf(laneEnable)
        val isCos = Bool()
        val earlyTerm = chiselTypeOf(earlyTerminate)
        val earlyRes = chiselTypeOf(earlyResult)
    }

    // Initializing the states
    def numIntermediateStages = 6
    val commonState = RegInit(VecInit(Seq.fill(numIntermediateStages)(0.U.asTypeOf(new CommonStageState))))
    val backPressure = Wire(Vec(numIntermediateStages, Bool()))
    val stateWithBp = commonState.zip(backPressure)
    def st(i: Int) = commonState(i-1)
    def bp(i: Int) = backPressure(i-1)

    // Assigning values to the 0th stage of the pipeline: select between input and holding value based on backpressure.
    stateWithBp.take(1).foreach { case (state, back) =>
        state.valid := Mux(!back, io.req.fire, state.valid)
        state.req := Mux(io.req.fire, io.req.bits, state.req)
        state.laneEn := Mux(!back, laneEnable, state.laneEn)
        state.isCos := Mux(io.req.fire, maskedCos, state.isCos)
        state.earlyTerm := maskLane(state.earlyTerm, earlyTerminate, laneEnable, back)
        state.earlyRes := maskLane(state.earlyRes, earlyResult, laneEnable, back)
    }

    // Assigning values to the rest of the stages: if not back pressured, take value from previous stage; if back pressured, hold current value.
    stateWithBp.zipWithIndex.takeRight(numIntermediateStages - 1).foreach {
        case ((state, back), i) =>
        state.valid := Mux(!back, st(i).valid, state.valid)
        state.req := Mux(st(i).valid && !back, st(i).req, state.req)
        state.laneEn := Mux(!back, st(i).laneEn, state.laneEn)
        state.isCos := Mux(st(i).valid && !back, st(i).isCos, state.isCos)
        state.earlyTerm := maskLane(state.earlyTerm, st(i).earlyTerm, st(i).laneEn, back)
        state.earlyRes := maskLane(state.earlyRes, st(i).earlyRes, st(i).laneEn, back)
    }

    /* Assigning backpressure: 
     * Last stage is back pressured if response is not ready.
     * Current stage is back pressured if it is valid and the next stage is back pressured. 
     */ 
    backPressure(numIntermediateStages-1) := !io.resp.ready
    backPressure.zip(commonState).zipWithIndex.take(numIntermediateStages-1).foreach { case ((bp, st), i) =>
        bp := st.valid && backPressure(i+1)
    }
    
    
    /* Stage 1:
     * 1. Pasing input from stage 0 to stage 1 based on backpressure and lane enable
     * 2. Raw float to qmn conversion
     * 3. Determine the quadrant for each input and the sign of sine and cosine based on the quadrant
     */  
    val stage1RawFloatVec = maskLaneNext(rawFloatVec, laneEnable, bp(1))
    val stage1Qmn = VecInit(stage1RawFloatVec.map(x => BF16T.qmnFromRawFloat(x)))
    

    /* Stage 2: 
     * 1. Passing qmn and quadrant info from stage 1 to stage 2 based on backpressure and lane enable
     * 2. Reducing qmn to the first quadrant based on the quadrant info 
     * 3. Taking the integer bits and the sig bits of the reduced qmn for LUT indexing and interpolation respectively
     * 4. Indexing into the LUT with the (sig - lowBits) bits and requesting the data when valid and not back pressured
     * 5. Determining the sign of the output and cos/sin based on the quadrant information
     * 6. Assigning inputs to the LUT
     */  
    val k2_over_pi_bf16       = fpTobf((2.0 / Math.PI).toFloat).U(16.W)
    def k2verPi = BF16T.qmnFromRawFloat(rawFloatFromFN(BF16T.expWidth, BF16T.sigWidth, k2_over_pi_bf16))
    val stage2Qmn = maskLaneNext(stage1Qmn, st(1).laneEn, bp(2))
    val krVec = stage2Qmn.map(_.mul(k2verPi).getKR).unzip
    val stage2kVec = VecInit(krVec._1)
    val stage2rVec = VecInit(krVec._2)
    val stage2MaskedNeg = VecInit(stage2kVec.map { q => Mux(st(2).isCos, 
            Mux(q(1,0) === 1.U || q(1,0) === 2.U, true.B, false.B), // Cosine is negative in quadrants 2 and 3
            Mux(q(1,0) === 2.U || q(1,0) === 3.U, true.B, false.B)  // Sine is negative in quadrants 3 and 4
        )
    })
    val stage2IsCosVec = VecInit(stage2kVec.map(q => Mux(st(2).isCos, 
        Mux(q(1,0) === 0.U || q(1,0) === 2.U, true.B, false.B), // If cos based, quadrant 1 and 3 behave the same as cos
        Mux(q(1,0) === 1.U || q(1,0) === 3.U, true.B, false.B)  // If sin based, quadrant 2 and 4 behave the same as cos
    )))

    lut.io.raddr := stage2rVec.map(r => r(BF16T.qmnN - 1, rLowBits))
    lut.io.ren := VecInit(st(2).laneEn.map(en => en && st(2).valid && !bp(3)))
    lut.io.isCos := stage2IsCosVec

    /* Stage 3: 
     * 1. Passing necessary signals to stage 3 based on backpressure and lane enable
     * 2. Calculate the address for LUT and the lower bits of r for interpolation
     * 3. Output from LUT is ready
     * 4. Calcualte delta for interpolation
     * 5. Initalize the true base value from LUT output for interpolation which is y0 for sine and y1 for cosine
     */ 
    val stage3IsCosVec = maskLaneNext(stage2IsCosVec, st(2).laneEn, bp(3))
    val stage3MaskedNeg = maskLaneNext(stage2MaskedNeg, st(2).laneEn, bp(3))
    val stage3kVec = maskLaneNext(stage2kVec, st(2).laneEn, bp(3))
    val stage3rVec = maskLaneNext(stage2rVec, st(2).laneEn, bp(3))
    val stage3AddrVec = VecInit(stage3rVec.map(r => r(BF16T.qmnN - 1, rLowBits)))
    val stage3rLowerVec = VecInit(stage3rVec.zip(stage3IsCosVec).map{case(r, c) => Mux(c, (1 << rLowBits).U - r(rLowBits-1, 0), r(rLowBits-1, 0))})  
    val stage3y0 = lut.io.rdata(0)
    val stage3y1 = lut.io.rdata(1)
    val stage3delta = VecInit(stage3y0.zip(stage3y1).zip(stage3IsCosVec).map { 
        case ((y0, y1), c) => Mux(c, y0 - y1, y1 - y0 )})
    val stage3basey = VecInit(stage3y0.zip(stage3y1).zip(stage3IsCosVec).map{case ((y0, y1), c) => Mux(c, y1, y0)})

    // Stage 4: Perform interpolation (i.e. delta * lowbits of qmn)
    val stage4MaskedNeg = maskLaneNext(stage3MaskedNeg, st(3).laneEn, bp(4))
    val stage4kVec = maskLaneNext(stage3kVec, st(3).laneEn, bp(4))
    val stage4y0 = maskLaneNext(stage3basey, st(3).laneEn, bp(4))
    val stage4delta = maskLaneNext(stage3delta, st(3).laneEn, bp(4))
    val stage4frac = maskLaneNext(stage3rLowerVec, st(3).laneEn, bp(4))
    val stage4deltaFrac = VecInit(stage4delta.zip(stage4frac).map {
        case (delta, frac) => (delta * frac) >> rLowBits
    })

    // Stage 5: add delta * frac to y0
    val staged5MaskedNeg = maskLaneNext(stage4MaskedNeg, st(4).laneEn, bp(5))
    val stage5kVec = maskLaneNext(stage4kVec, st(4).laneEn, bp(5))
    val stage5y0 = maskLaneNext(stage4y0, st(4).laneEn, bp(5))
    val stage5deltaFrac = maskLaneNext(stage4deltaFrac, st(4).laneEn, bp(5))
    val stage5OutputVec = VecInit(stage5y0.zip(stage5deltaFrac).map {
        case (y0, deltaFrac) => y0 + deltaFrac
    })

    //stage 6: convert and return result
    val staged6MaskedNeg = maskLaneNext(staged5MaskedNeg, st(5).laneEn, bp(6))
    val stage6kVec = maskLaneNext(stage5kVec, st(5).laneEn, bp(6))
    val stage6OutputVec = maskLaneNext(stage5OutputVec, st(5).laneEn, bp(6))
    val resRawFloat = stage6OutputVec.zip(stage6kVec).zip(staged6MaskedNeg).map{ case ((qmn, k), n) => BF16T.rawFloatFromQmnK(qmn, 0.S).negate(n) }
    // val resRawFloat = stage6OutputVec.map(q => BF16T.rawFloatFromQmnK(q, 0.S))
    roundToRecFn.zip(resRawFloat).zip(st(6).laneEn).foreach {
        case ((round, rawFloat), en) => {
            round.io.invalidExc := false.B
            round.io.infiniteExc := false.B
            round.io.in := rawFloat
            round.io.roundingMode := Mux(en, st(6).req.roundingMode, 0.U)
            round.io.detectTininess := 0.U
        }
    }
    val resValid = st(6).valid
    val resReq = st(6).req
    val resFN = roundToRecFn.map(round => fNFromRecFN(BF16T.expWidth, BF16T.sigWidth, round.io.out))
    // val resFinal = VecInit(resFN.zip(st(6).earlyTerm.zip(st(6).earlyRes)).map {
    //     case (res, (earlyTerminate, earlyRes)) => Mux(earlyTerminate, earlyRes, res)
    // })
    val resFinal = VecInit(resFN.zip(st(6).earlyTerm.zip(st(6).earlyRes)).map {
        case (res, (earlyTerminate, earlyRes)) =>
            val sign = res(15)
            val exp  = res(14,7)
            val frac = res(6,0)

            val doSub1 = (exp === 127.U)
            val isZero = (frac === 0.U)

            val leadingzero  = PriorityEncoder(Reverse(frac)) 
            val shiftNum  = leadingzero + 1.U   

            val expOut = 126.S - leadingzero.zext.asSInt
            val fracOut = (frac << shiftNum)(6,0)

            val subtracted1 =
            Mux(isZero,
                Cat(sign, 0.U(8.W), 0.U(7.W)),        // 1.0 - 1.0 = 0
                Cat(sign, expOut, fracOut)
            )

            val out = Mux(doSub1, subtracted1, res)

            Mux(earlyTerminate, earlyRes, out)
    })

    // Output signals
    io.req.ready := !backPressure(0)
    io.resp.valid := resValid
    io.resp.bits.tag := resReq.tag
    io.resp.bits.whichBank := resReq.whichBank
    io.resp.bits.wRow := resReq.wRow
    io.resp.bits.result := resFinal
    io.resp.bits.laneMask := resReq.laneMask



    // Debugging
    // Debugging
    // Debugging
    // Input Stage
    
    // Stage 0

    // Stage 1

    // // Stage 2
    // val kval = IO(Output(Vec(numLanes, SInt(18.W))))
    // val addr = IO(Output(Vec(numLanes, UInt(18.W))))
    // // Stage 3
    // val d_stage3y0 = IO(Output(Vec(numLanes, UInt(18.W))))
    // val d_stage3y1 = IO(Output(Vec(numLanes, UInt(18.W))))
    // val d_stage3delta = IO(Output(Vec(numLanes, UInt(32.W))))
    

    // // Stage 4 

    // for (i <- 0 until numLanes) {
    //     kval(i) := stage2kVec(i)
    //     addr(i) := lut.io.raddr(i)

    //     d_stage3y0(i) := stage3y0(i)
    //     d_stage3y1(i) := stage3y1(i)
    //     d_stage3delta(i) := stage3delta(i)

    // }
}