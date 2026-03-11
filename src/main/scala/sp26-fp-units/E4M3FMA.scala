/*
Full E4M3 FMA: (fp8_a * fp8_b) + bf16_addend -> bf16 output.

Chains E4M3Mul and E4M3ProdAddBF16.
Inputs: two 8-bit E4M3 (IEEE), one 16-bit BF16 (IEEE).
Output: 16-bit BF16 (IEEE).
*/

package sp26FPUnits

import chisel3._
import chisel3.util._

class E4M3FMA extends Module {
  val io = IO(new Bundle {
    val a        = Input(UInt(8.W))   // E4M3
    val b        = Input(UInt(8.W))   // E4M3
    val addend16 = Input(UInt(16.W))  // BF16
    val out16    = Output(UInt(16.W)) // BF16
  })

  val mul = Module(new E4M3Mul)
  mul.io.a := io.a
  mul.io.b := io.b

  val addRound = Module(new E4M3ProdAddBF16)
  addRound.io.prod13 := mul.io.out
  addRound.io.addend16 := io.addend16

  io.out16 := addRound.io.out16
}
