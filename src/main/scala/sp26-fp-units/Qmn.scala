package sp26FPUnits

import chisel3._
import chisel3.util._

class Qmn(val m: Int, val n: Int) extends Bundle {
  def apply(value: SInt): Qmn = {
    val q = Wire(new Qmn(m, n))
    q.value := value
    q
  }

  def mul(qkn: Qmn): Qmn = {
    assert(n == qkn.n)
    val prod = ((value * qkn.value) >> n).asSInt
    new Qmn(m + qkn.m, n)(prod)
  }

  def getKR = {
    val k = (value >> n).asSInt
    val r = value(n - 1, 0).asUInt
    (k, r)
  }

  val value = SInt((m + n).W)
}