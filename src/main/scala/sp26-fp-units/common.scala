package sp26FPUnits
import chisel3._
import chisel3.util._
import sp26FPUnits.hardfloat._

/**
  * Describes an IEEE-754-style binary format using HardFloat's (expWidth, sigWidth) convention,
  * where sigWidth = mantissa bits + 1 (includes the implicit leading bit).
  */
final case class AtlasFPType(
  exp: Int,
  sig: Int,
  qmnM: Int = 0,            // integer bits in fixed point
  qmnN: Int = 0,            // frac bits in fixed point
  lutAddrBits: Int = 0,     // num LUT addr bits
  lutValM: Int = 1,         // int bits in lut output format
  lutValN: Int = 0,         // frac bits in lut output
  maxXExpVal: BigInt = 0,   // exponent threshold before exp overflow
  maxXSigVal: BigInt = 0,   // mantsisa threshold before overflow
  rln2Val: BigInt = 0       // lit the constant for exp conversion 
) {
  require(exp >= 2, s"expWidth must be >= 2, got $exp")
  require(sig >= 2, s"sigWidth must be >= 2, got $sig")
  

  // Keep both naming styles to reduce churn in downstream code.
  val expWidth: Int = exp
  val sigWidth: Int = sig

  // number of explicit mantissa bits (excludes the implicit leading bit)
  val mantissaBits: Int = sigWidth - 1

  // IEEE-754 exponent bias: 2^(ew-1) - 1
  val ieeeBias: Int = (1 << (expWidth - 1)) - 1

  // Aliasfor ieeeBias
  val bias: Int = ieeeBias

  /** IEEE width: sign + exp + fraction(sig-1) */
  val ieeeWidth: Int = 1 + expWidth + mantissaBits

  // Another alias for IEEE width used by Vector
  val wordWidth: Int = ieeeWidth

  /** HardFloat recoded width: sign + (ew+1) exponent + mantissa */
  val recodedWidth: Int = 1 + (expWidth + 1) + mantissaBits

  // Alias used by a lot of HardFloat-centric code.
  val recFNWidth: Int = recodedWidth

  /* FUNCTIONS FROM FPEX: (needed for Vector) */
  def nanExp: UInt = Fill(expWidth, 1.U(1.W))
  def nanSig: UInt = Fill(sigWidth - 2, 1.U(1.W))
  def zero: UInt = Cat(0.U(1.W), Fill(expWidth, 0.U(1.W)), Fill(sigWidth - 1, 0.U(1.W)))
  def one: UInt = Cat(0.U(1.W), ieeeBias.U(expWidth.W), 0.U((sigWidth - 1).W))
  def infinity: UInt = Cat(Fill(expWidth, 1.U(1.W)), 0.U((sigWidth - 1).W))

  def maxXExp: UInt = maxXExpVal.U(expWidth.W)
  def maxXSig: UInt = maxXSigVal.U((sigWidth - 1).W)

  def qmnCtor = () => new Qmn(qmnM, qmnN)

  def rln2: Qmn = new Qmn(2, qmnN)(rln2Val.S((2 + qmnN).W))

  def recode(x: UInt): UInt = recFNFromFN(expWidth, sigWidth, x)

  def expFPIsInf(in: UInt, neg: Bool): Bool = {
    val sign = in(wordWidth - 1) ^ neg
    val expBits = in(wordWidth - 2, sigWidth - 1)
    val frac = in(sigWidth - 2, 0)
    !sign && (expBits > maxXExp || (expBits === maxXExp && frac > maxXSig))
  }

  // assumes no special cases and no overflow. no rounding
  def qmnFromRawFloat(rawFloat: RawFloat): Qmn = {
    val recodedBias = (1 << expWidth).S((expWidth + 2).W)
    val unbiasedExp = rawFloat.sExp - recodedBias
    val shift = qmnN.asSInt + unbiasedExp - (sigWidth - 1).asSInt
    val qmn = Wire(qmnCtor())
    val sigWide = Wire(UInt((qmnM + qmnN).W))
    sigWide := rawFloat.sig
    val mag = Mux(shift < 0.S, sigWide >> (-shift).asUInt, sigWide << shift.asUInt).asSInt
    qmn.value := Mux(rawFloat.sign, -mag, mag)
    qmn
  }

  // sign is always positive
  def rawFloatFromQmnK(qmn: UInt, k: SInt): RawFloat = {
    val out = Wire(new RawFloat(expWidth, sigWidth + 2))
    out.isNaN := false.B
    out.isInf := false.B
    out.isZero := false.B
    out.sign := false.B

    val expBias = (1 << expWidth).S((expWidth + 2).W)
    out.sExp := k + expBias

    val shiftAmt = lutValN - (sigWidth - 1)
    if (shiftAmt >= 2) {
      val sigWithGR = qmn >> (shiftAmt - 2)
      val sticky = if (shiftAmt > 2) qmn(shiftAmt - 3, 0).orR else false.B
      val preSig = Cat(0.U(1.W), sigWithGR(sigWidth + 1, 0))
      out.sig := Mux(sticky, preSig | 1.U((sigWidth + 3).W), preSig)
    } else {
      val shift = (lutValN - (sigWidth - 1)).asSInt
      val sigMag = Mux(shift < 0.S, qmn << (-shift).asUInt, qmn >> shift.asUInt)
      out.sig := Cat(0.U(1.W), sigMag(sigWidth - 1, 0), 0.U(2.W))
    }
    out
  }
}

object AtlasFPType {
  // Common presets
  val E4M3 = AtlasFPType(exp = 4,  sig = 4)    // FP8 E4M3:   1 + 4 + 3  =  8
  val E5M2 = AtlasFPType(exp = 5,  sig = 3)    // FP8 E5M2:   1 + 5 + 2  =  8
  val E5M3 = AtlasFPType(exp = 5,  sig = 4)    // FP8 E5M3:   1 + 5 + 3  =  9
  val FP16 = AtlasFPType(                      // IEEE half:  1 + 5 + 10 = 16
    exp = 5,
    sig = 11,
    qmnM = 6,
    qmnN = 12,
    lutAddrBits = 6,
    lutValM = 1,
    lutValN = 16,
    maxXExpVal = BigInt("12", 16),
    maxXSigVal = BigInt("18c", 16),
    rln2Val = 5909
  )

  val BF16 = AtlasFPType( // BFloat16:   1 + 8 + 7  = 16
    exp = 8,
    sig = 8,
    qmnM = 9,
    qmnN = 12,
    lutAddrBits = 5,
    lutValM = 1,
    lutValN = 16,
    maxXExpVal = BigInt("85", 16),
    maxXSigVal = BigInt("31", 16),
    rln2Val = 5909
  )

  val FP32 = AtlasFPType( // IEEE single
    exp = 8,
    sig = 24,
    qmnM = 10,
    qmnN = 24,
    lutAddrBits = 9,
    lutValM = 1,
    lutValN = 30,
    maxXExpVal = BigInt("85", 16),
    maxXSigVal = BigInt("317218", 16),
    rln2Val = 24204406
  )   
  val FP64 = AtlasFPType(exp = 11, sig = 53)   // IEEE double
}

/**
  * Standalone constants for the E4M3 product format (the wider intermediate
  * format produced by an E4M3 x E4M3 multiply before accumulation).
  *
  * Layout: S(1) E(5, bias=13) M(7) = 13 bits total.
  * Note: bias is 13 (not the IEEE default of 15 for a 5-bit exponent)
  * because E4M3Mul produces exponents in the range dictated by the
  * E4M3 input operands.
  */
object E4M3ProdFmt {
  val expWidth : Int = 5
  val manWidth : Int = 7
  val sigWidth : Int = 8   // implicit 1 + 7 mantissa
  val bias     : Int = 13
  val width    : Int = 13  // 1 + 5 + 7
}
