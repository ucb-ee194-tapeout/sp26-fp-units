import chisel3._
import chisel3.simulator.EphemeralSimulator._
import org.scalatest.flatspec.AnyFlatSpec
import scala.math.pow
import java.io._

def exhaustiveTest(dutIn: UInt, dutOut: UInt, dutInVld: UInt, dutOutVld: UInt, dutValid cycles: Int) {
  private def openLUT(path: String): Seq[Vec] = {
    val src = Source.fromFile(path)
  }
  FileInputStream fileIn = new FileInputStream("out.bin")
  DataInputStream dataIn = new DataInputStream(fileIn))
  FileOutputStream fileOut = new FileOutputStream("results.bin")
  DataOutputStream dataOut = new DataOutputStream(fileOut)

  var failCtr: Int = 0
  var expected: Int = 0
  for (ctr <- 0 until pow(2, dutIn.getWidth())) {
    dutIn.poke(ctr.U)
    dutInVld.poke(1.U)
    dutIn.clock.stepUntil(dutOutVld === 1.U)

    // Check result
    // TODO: Arbitrary number of bytes
    expected = dataIn.readShort()
    dutOut.expect(expected)
    val result = dutOut.peek()
    // TODO: Interpret as float
    if (result != expected)
      println(s"FAIL test $ctr: got $result but expected $expected")
    dataOut.writeShort(result)
  }

  // Close files
  fileIn.close()
  fileOut.close()

  println(s"Possibilities: $ctr")
  println(s"Failures: $failCtr")
}

class ExhaustiveTest extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "GenericMulMulAddFN"
  // Run (until failure, all the way): all the way
  // Generate file (no, yes): yes
  // Take in number of 
  // One test can instantiate multiple DUTs? (see matrix)

  it should "Test every possible input" in {
    simulate(new GenericMulMulAddFN) { dut =>
      val dutIn = Cat(dut.io.in0, dut.io.in1)
      val dutOut = dut.out
      dut.io.in2.poke(0.U)
      dut.io.in3.poke(0.U)
      dut.io.altfmt.poke(0.U)
      exhaustiveTest(dutIn, dutOut, dut.io.validIn.asUInt, dut.io.validOut.asUInt, 0)
    }
  }
}
