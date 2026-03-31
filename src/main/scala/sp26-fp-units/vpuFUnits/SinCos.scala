package sp26FPUnits

import chisel3._
import chisel3.util._
import sp26FPUnits.hardfloat._
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

    def bf16ToQmnTimesTwoOverPi(x: UInt, qmnM: Int = 9, qmnN: Int = 12): SInt = {
        require(x.getWidth == 16, "bf16ToQmnTimesPiOver2 expects a 16-bit BF16 input")

        val sign = x(15)
        val exp  = x(14, 7)
        val frac = x(6, 0)

        val bias = 127
        val outW = qmnM + qmnN + 1   // signed Q(m,n) width

        val isZero = exp === 0.U && frac === 0.U
        val isSub  = exp === 0.U && frac =/= 0.U
        val isInf  = exp === "hff".U && frac === 0.U
        val isNaN  = exp === "hff".U && frac =/= 0.U

        val maxVal = ((BigInt(1) << (qmnM + qmnN)) - 1).S(outW.W)
        val minVal = (-(BigInt(1) << (qmnM + qmnN))).S(outW.W)

        // -------------------------
        // Step 1: BF16 -> Q(m,n)
        // -------------------------
        val mant = Cat(1.U(1.W), frac) // 1.frac, scaled by 2^7

        // Q(m,n) integer = mant * 2^(exp-bias+qmnN-7)
        val shift = (exp.zext - bias.S) + qmnN.S - 7.S

        val shiftedUnsigned = Wire(UInt((qmnM + qmnN + 8).W))
        shiftedUnsigned := 0.U

        when (shift >= 0.S) {
            val sh = shift.asUInt
            shiftedUnsigned := (mant << sh)(qmnM + qmnN + 7, 0)
        } .otherwise {
            val rsh = (-shift).asUInt
            shiftedUnsigned := mant >> rsh
        }

        val shiftedSigned = shiftedUnsigned.asSInt
        val qmnVal = Wire(SInt(outW.W))
        qmnVal := 0.S

        when (isZero || isSub) {
            qmnVal := 0.S
        } .elsewhen (isNaN) {
            qmnVal := 0.S
        } .elsewhen (isInf) {
            qmnVal := Mux(sign, minVal, maxVal)
        } .otherwise {
            val signedVal = Mux(sign, -shiftedSigned, shiftedSigned)

            when (signedVal > maxVal) {
                qmnVal := maxVal
            } .elsewhen (signedVal < minVal) {
                qmnVal := minVal
            } .otherwise {
                qmnVal := signedVal.asTypeOf(SInt(outW.W))
            }
        }

        // -------------------------
        // Step 2: multiply by 2/pi in Q(m,n)
        // -------------------------
        val twoOverPiFixed =((BigDecimal(2 / math.Pi) * BigDecimal(BigInt(1) << qmnN)).setScale(0, BigDecimal.RoundingMode.HALF_UP).toBigInt).S(outW.W)
        val multW = 2 * outW
        val product = Wire(SInt(multW.W))
        product := qmnVal * twoOverPiFixed

        // Back to Q(m,n)
        val scaledProduct = (product >> qmnN).asSInt

        // -------------------------
        // Step 3: saturate result
        // -------------------------
        val result = Wire(SInt(outW.W))
        result := 0.S

        when (scaledProduct > maxVal) {
            result := maxVal
        } .elsewhen (scaledProduct < minVal) {
            result := minVal
        } .otherwise {
            result := scaledProduct(outW - 1, 0).asSInt
        }

        result
    }

    def lutFixedToBf16(x: UInt, lutValM: Int = 1, lutValN: Int = 16, neg: Bool = false.B): UInt = {
        val inW = lutValM + lutValN
        require(x.getWidth == inW, s"lutFixedToBf16 expects ${inW}-bit UInt input")

        val expW  = 8
        val fracW = 7
        val bias  = 127

        val isZero = x === 0.U

        // index of highest 1 bit in x
        val highestIdx = Wire(UInt(log2Ceil(inW).W))
        highestIdx := 0.U
        for (i <- 0 until inW) {
            when(x(i)) { highestIdx := i.U }
        }

        // since x represents real_value * 2^lutValN
        // real exponent = highestIdx - lutValN
        val unbiasedExp = highestIdx.zext - lutValN.S
        val bfExpSigned = unbiasedExp + bias.S

        // shift so leading 1 lands in bit 7 of an 8-bit mantissa: [1].[7 frac bits]
        val shiftAmt = Wire(UInt(log2Ceil(inW + 1).W))
        shiftAmt := Mux(highestIdx > 7.U, highestIdx - 7.U, 0.U)

        val mantPre = x >> shiftAmt
        val mant8   = Wire(UInt(8.W))
        mant8 := mantPre(7, 0)

        val frac = mant8(6, 0)

        val result = Wire(UInt(16.W))
        result := 0.U

        when(isZero) {
            result := Cat(neg, 0.U(expW.W), 0.U(fracW.W))
        } .elsewhen(bfExpSigned <= 0.S) {
            // flush subnormals/underflow to zero
            result := Cat(neg, 0.U(expW.W), 0.U(fracW.W))
        } .elsewhen(bfExpSigned >= 255.S) {
            // overflow to infinity
            result := Cat(neg, Fill(expW, 1.U(1.W)), 0.U(fracW.W))
        } .otherwise {
            result := Cat(neg, bfExpSigned.asUInt(7, 0), frac)
        }

        result
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

class SinCos(BF16T: AtlasFPType, numLanes: Int = 16, tagWidth: Int = 16) extends Module with HasSinCosParams {
    val io = IO(new Bundle {
        val req = Flipped(Decoupled(new SinCosReq(BF16T.wordWidth, numLanes, tagWidth)))
        val resp = Decoupled(new SinCosResp(BF16T.wordWidth, numLanes, tagWidth))
    })

    // Input restriciton
    val zeroBF16  = 0.U(BF16T.wordWidth.W)
    val twoPiBF16 = "h40C9".U(BF16T.wordWidth.W) // BF16 encoding of ~6.28125
    for (i <- 0 until numLanes) {
        when (io.req.fire && io.req.bits.laneMask(i)) {
            assert(
                io.req.bits.xVec(i) >= zeroBF16 && io.req.bits.xVec(i) <= twoPiBF16,
                s"xVec($i) must be in [0, 2pi]"
            )
        }
    }

    // Initial Setup
    val rLowBits = BF16T.qmnN - BF16T.lutAddrBits
    val lut = Module(new SinCosLUT(numLanes, BF16T.lutAddrBits, BF16T.lutValM, BF16T.lutValN))
    val laneEnable = VecInit((io.req.bits.laneMask & VecInit.fill(numLanes)(io.req.fire).asUInt).asBools)
    val maskedCos = io.req.bits.cos && io.req.fire
    val maskedXVec = io.req.bits.xVec.zip(laneEnable).map {
        case (x, en) => Mux (en, x, 0.U(BF16T.wordWidth.W))
    }
    
    /* Stage 0:
     *  1. BF16 -> Q(m,n)
     *  2. Determine sin/cos behavior
     *  3. Determine otuput sign
    */
    val qmnVec = VecInit(maskedXVec.map(x => bf16ToQmnTimesTwoOverPi(x)))
    val nVec   = VecInit(qmnVec.map(x => x(BF16T.qmnN - 1, 0)))
    val quadrantBits = VecInit(qmnVec.map(x => x(BF16T.qmnN + 1, BF16T.qmnN)))
    val isNegVec = VecInit(quadrantBits.map(x => Mux(maskedCos, 
            Mux(x === 1.U || x === 2.U, true.B, false.B), // Cosine is negative in quadrants 2 and 3
            Mux(x === 2.U || x === 3.U, true.B, false.B)  // Sine is negative in quadrants 3 and 4
    ))) 
    val isCosVec = VecInit(quadrantBits.map(x => Mux(maskedCos, 
        Mux(x === 0.U || x === 2.U, true.B, false.B), // If cos based, quadrant 1 and 3 behave the same as cos
        Mux(x === 1.U || x === 3.U, true.B, false.B)  // If sin based, quadrant 2 and 4 behave the same as cos
    )))
    
    // States for pipelining
    class CommonStageState extends Bundle {
        val valid = Bool()
        val req = chiselTypeOf(io.req.bits)
        val laneEn = chiselTypeOf(laneEnable)
        val isCos = Bool()
        val qmnNVec = chiselTypeOf(nVec)
    }

    // Initializing the states
    def numIntermediateStages = 1
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
        state.qmnNVec := maskLane(state.qmnNVec, nVec, laneEnable, back)
    }
    backPressure(numIntermediateStages-1) := !io.resp.ready

    lut.io.raddr := nVec.map(r => r(BF16T.qmnN - 1, rLowBits))
    lut.io.ren := VecInit(laneEnable.map(en => en && io.req.fire && !bp(1)))
    lut.io.isCos := isCosVec

    /* Stage 1: Interpolation */
    val stage1MaskedNeg = maskLaneNext(isNegVec, laneEnable, bp(1))
    val stage1IsCosVec = maskLaneNext(isCosVec, laneEnable, bp(1))
    val stage1rLowerVec = VecInit(st(1).qmnNVec.zip(stage1IsCosVec).map{case(r, c) => Mux(c, (1 << rLowBits).U - r(rLowBits-1, 0), r(rLowBits-1, 0))})  
    val y0Vec = lut.io.rdata(0)
    val y1Vec = lut.io.rdata(1)
    val delta = VecInit(y0Vec.zip(y1Vec).zip(stage1IsCosVec).map { 
        case ((y0, y1), c) => Mux(c, y0 - y1, y1 - y0 )})
    val basey = VecInit(y0Vec.zip(y1Vec).zip(stage1IsCosVec).map{case ((y0, y1), c) => Mux(c, y1, y0)})
    val resultLutFixed = VecInit(delta.zip(stage1rLowerVec).zip(basey).map { case ((d, frac), y) =>
        val interp = ((d * frac) >> rLowBits) + y
        interp(BF16T.lutValM + BF16T.lutValN - 1, 0)
    })
    val resultBF16 = VecInit(resultLutFixed.map(x => lutFixedToBf16(x, BF16T.lutValM, BF16T.lutValN)))
    val resultFinal = VecInit(resultBF16.zip(stage1MaskedNeg).map{case (x, c) => Mux(c, Cat(1.U, x(14,0)), x)})
    val resValid = st(1).valid
    val resReq = st(1).req

    // Output signals
    io.req.ready := !backPressure(0)
    io.resp.valid := resValid
    io.resp.bits.tag := resReq.tag
    io.resp.bits.whichBank := resReq.whichBank
    io.resp.bits.wRow := resReq.wRow
    io.resp.bits.result := resultFinal
    io.resp.bits.laneMask := resReq.laneMask
}