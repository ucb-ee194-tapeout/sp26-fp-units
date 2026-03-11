/*
Add-and-round stage: E4M3Prod (13-bit) + BF16 addend -> BF16 output.

E4M3ProdFmt: S(1) E(5, bias=13) M(7) = 13 bits
BF16 IEEE:   S(1) E(8, bias=127) M(7) = 16 bits

Clean ML contract: no NaN, Inf, subnormals.
Overflow: clamp to max finite BF16.
Underflow: flush to zero.
Rounding: round-to-nearest-even.
*/

package sp26FPUnits

import chisel3._
import chisel3.util._

class E4M3ProdAddBF16 extends Module {
  val io = IO(new Bundle {
    val prod13   = Input(UInt(13.W))  // E4M3Prod (E5M7)
    val addend16 = Input(UInt(16.W))  // BF16
    val out16    = Output(UInt(16.W)) // BF16
  })

  // Constants
  private val SIG8 = 8
  private val GRS = 3
  private val SIG11 = 11
  private val EXP_BIAS_OFFSET = 114

  // 1. Unpack product
  val pSign = io.prod13(12)
  val pExp5 = io.prod13(11, 7)
  val pMan7 = io.prod13(6, 0)
  val pZero = (pExp5 === 0.U)
  val pSig8 = Mux(pZero, 0.U(SIG8.W), Cat(1.U(1.W), pMan7))
  val pExp8 = (pExp5 + EXP_BIAS_OFFSET.U(8.W))(7, 0)

  // 2. Unpack BF16 addend
  val aSign = io.addend16(15)
  val aExp8 = io.addend16(14, 7)
  val aMan7 = io.addend16(6, 0)
  val aZero = (aExp8 === 0.U)
  val aSig8 = Mux(aZero, 0.U(SIG8.W), Cat(1.U(1.W), aMan7))

  // 3. Zero fast paths
  val outWhenPZero = io.addend16
  val outWhenAZero = Cat(pSign, pExp8, pMan7)

  // 4. Extend to 11 bits
  val pSig11 = (pSig8 << GRS)
  val aSig11 = (aSig8 << GRS)

  // 5. Compare and swap
  val pGtA = (pExp8 > aExp8) || ((pExp8 === aExp8) && (pSig11 >= aSig11))
  val eMax = Mux(pGtA, pExp8, aExp8)
  val eMin = Mux(pGtA, aExp8, pExp8)
  val sMax = Mux(pGtA, pSign, aSign)
  val sMin = Mux(pGtA, aSign, pSign)
  val sigMax = Mux(pGtA, pSig11, aSig11)
  val sigMin = Mux(pGtA, aSig11, pSig11)

  val expDiff = eMax - eMin
  val shiftAmt = Mux(expDiff > 11.U, 11.U, expDiff(3, 0))

  // 6. Shift-right-jam (shiftAmt clamped to 11, so sigMin & shiftMask is always valid)
  val shiftedOut = (sigMin >> shiftAmt)(SIG11 - 1, 0)
  val shiftMask = (1.U << shiftAmt) - 1.U
  val sticky = (sigMin & shiftMask).orR
  val sigMinAligned = (shiftedOut | sticky)

  // 7. Add or subtract (12 bits for carry detection)
  val sameSign = (sMax === sMin)
  val sigSum = Mux(sameSign, sigMax +& sigMinAligned, Cat(0.U(1.W), sigMax - sigMinAligned))
  val cancelZero = (sigSum === 0.U)

  // 8. Normalize - addition carry
  val carryOut = sigSum(SIG11)
  val sigAfterAddNorm = Mux(carryOut, sigSum >> 1, sigSum(SIG11 - 1, 0))
  val stickyAfterAddNorm = Mux(carryOut, sticky | sigSum(0), sticky)
  val eAfterAddNorm = Mux(carryOut, eMax + 1.U, eMax)

  // 9. Normalize - subtraction (leading zeros)
  val lzc = PriorityEncoder(Reverse(sigAfterAddNorm))
  val needLeftShift = (!carryOut) && (!sigAfterAddNorm(SIG11 - 1))
  val leftShiftAmt = Mux(needLeftShift, Mux((lzc > eAfterAddNorm), eAfterAddNorm, lzc), 0.U)
  val sigNorm = Mux(needLeftShift, (sigAfterAddNorm << leftShiftAmt)(SIG11 - 1, 0), sigAfterAddNorm)
  val eNorm = eAfterAddNorm - leftShiftAmt

  // 10. Round to nearest even
  val G = sigNorm(2)
  val R = sigNorm(1)
  val S = sigNorm(0) | stickyAfterAddNorm
  val mantLsb = sigNorm(3)
  val mantissa7 = sigNorm(9, 3)
  val roundUp = G && ((R || S) || mantLsb)
  val mantissaRounded = mantissa7 +& roundUp
  val roundOverflow = mantissaRounded(7)
  val mantissaOut = Mux(roundOverflow, 0.U(7.W), mantissaRounded(6, 0))
  val eRounded = Mux(roundOverflow, eNorm + 1.U, eNorm)

  // 11. Overflow/underflow clamp
  val underflow = (eNorm <= 0.U)
  val overflow = (eRounded > 254.U)
  val eOut = Mux(overflow, 254.U, Mux(underflow, 0.U, eRounded(7, 0)))
  val mOut = Mux(overflow, 0x7F.U(7.W), Mux(underflow, 0.U(7.W), mantissaOut))

  // 12. Output mux
  io.out16 := Mux(pZero, outWhenPZero,
    Mux(aZero, outWhenAZero,
      Mux(cancelZero, 0.U(16.W),
        Cat(sMax, eOut, mOut))))
}
