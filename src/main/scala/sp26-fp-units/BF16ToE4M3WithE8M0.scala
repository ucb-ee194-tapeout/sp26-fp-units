package sp26FPUnits

import chisel3._
import chisel3.util._

class BF16ToE4M3WithE8M0 extends Module {
  val io = IO(new Bundle {
    val bf16In    = Input(UInt(16.W))
    val scaleE8M0 = Input(UInt(8.W))
    val e4m3Out   = Output(UInt(8.W))
  })
  val scaleExpWide = io.scaleE8M0.zext -& 127.S(9.W)
  val scaleExp     = Wire(SInt(8.W))
  when(scaleExpWide > 127.S) {
    scaleExp := 127.S
  }.elsewhen(scaleExpWide < (-128).S) {
    scaleExp := (-128).S
  }.otherwise {
    scaleExp := scaleExpWide(7, 0).asSInt
  }
  val conv = Module(new BF16ScaleToE4M3())
  conv.io.bf16In   := io.bf16In
  conv.io.scaleExp := scaleExp
  io.e4m3Out       := conv.io.e4m3Out(7, 0)
}
