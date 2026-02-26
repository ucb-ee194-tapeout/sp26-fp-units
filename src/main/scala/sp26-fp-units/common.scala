package sp26FPUnits

import chisel3._
import sp26FPUnits.hardfloat._

/**
  * Describes an IEEE-754-style binary format using HardFloat's (expWidth, sigWidth) convention,
  * where sigWidth = mantissa bits + 1 (includes the implicit leading bit).
  */
final case class AtlasFPType(exp: Int, sig: Int) {
  require(exp >= 2, s"expWidth must be >= 2, got $exp")
  require(sig >= 2, s"sigWidth must be >= 2, got $sig")

  // Keep both naming styles to reduce churn in downstream code.
  val expWidth: Int = exp
  val sigWidth: Int = sig

  // number of explicit mantissa bits (excludes the implicit leading bit)
  val mantissaBits: Int = sigWidth - 1

  // IEEE-754 exponent bias: 2^(ew-1) - 1
  val ieeeBias: Int = (1 << (expWidth - 1)) - 1

  /** IEEE width: sign + exp + fraction(sig-1) */
  val ieeeWidth: Int = 1 + expWidth + mantissaBits

  /** HardFloat recoded width: sign + (ew+1) exponent + mantissa */
  val recodedWidth: Int = 1 + (expWidth + 1) + mantissaBits

  // Alias used by a lot of HardFloat-centric code.
  val recFNWidth: Int = recodedWidth

  def recode(x: UInt): UInt = recFNFromFN(expWidth, sigWidth, x)
}

object AtlasFPType {
  // Common presets
  val E4M3 = AtlasFPType(exp = 4,  sig = 4)    // FP8 E4M3:  1 + 4 + 3  = 8
  val E5M2 = AtlasFPType(exp = 5,  sig = 3)    // FP8 E5M2:  1 + 5 + 2  = 8
  val E5M3 = AtlasFPType(exp = 5,  sig = 4)    // FP8 E5M3:  1 + 5 + 3  = 9 (if used as custom)
  val FP16 = AtlasFPType(exp = 5,  sig = 11)   // IEEE half: 1 + 5 + 10 = 16
  val BF16 = AtlasFPType(exp = 8,  sig = 8)    // BFloat16:  1 + 8 + 7  = 16
  val FP32 = AtlasFPType(exp = 8,  sig = 24)   // IEEE single
  val FP64 = AtlasFPType(exp = 11, sig = 53)   // IEEE double
}
