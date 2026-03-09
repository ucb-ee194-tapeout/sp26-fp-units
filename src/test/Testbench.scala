package sp26SPUnits

import chisel3._
import chisel3.simulator.EphemeralSimulator._
import org.scalatest.flatspec.AnyFlatSpec
import org.scalatest.Outcome

import chisel3.util.Cat
import scala.math.pow
import java.io._

import sp26FPUnits._

class FPUSpec extends AnyFlatSpec {
  behavior of "GenericMulMulAddFN"
  // Run (until failure, all the way): all the way
  // Generate file (no, yes): yes
  // Take in number of 
  // One test can instantiate multiple DUTs? (see matrix)

  def exhaustiveTest(dut: Module, dutIn: UInt, dutOut: UInt, dutInVld: UInt, dutOutVld: UInt, cycles: Int): Int = {
    println("Starting exhaustive test")
    val fileIn: FileInputStream = new FileInputStream("../../../src/test/resources/GenericMulMulAddRecFN-MulAdd.bin")
    val dataIn: DataInputStream = new DataInputStream(fileIn)
    val fileOut: FileOutputStream = new FileOutputStream("results.bin")
    val dataOut: DataOutputStream = new DataOutputStream(fileOut)

    var failCtr: Int = 0
    var total: Int = 1 << dutIn.getWidth
    var expected: Int = 0
    for (ctr <- 0 until total) {
      dutIn.poke(ctr.U)
      dutInVld.poke(1.U)
      dut.clock.stepUntil(dutOutVld, 1, 20, 1)

      // Check result
      // TODO: Arbitrary number of bytes
      expected = dataIn.readShort()
      dutOut.expect(expected.U)
      val result = dutOut.peek()
      // TODO: Interpret as float
      if (result != expected.U)
        println(s"FAIL test $ctr: got $result but expected $expected")
      dataOut.writeShort(result.litValue.toInt & 0xFFFF)

      dut.clock.step(1)
    }

    // Close files
    fileIn.close()
    fileOut.close()

    println(s"Possibilities: $total")
    println(s"Failures: $failCtr")

    failCtr
  }

  it should "Test every possible input" in {
    println("Test start")
    simulate(new GenericMulMulAddRecFN(AtlasFPType.E4M3, AtlasFPType.E4M3, AtlasFPType.E4M3, AtlasFPType.E4M3, AtlasFPType.E4M3)) { dut =>
      val dutIn = Cat(dut.io.in0, dut.io.in1)
      val dutOut = dut.io.out
      dut.io.in2.poke(0.U)
      dut.io.in3.poke(0.U)
      dut.io.altfmt.poke(false.B)
      exhaustiveTest(dut, dutIn, dutOut, dut.io.validIn.asUInt, dut.io.validOut.asUInt, 0)
    }
  }
}
