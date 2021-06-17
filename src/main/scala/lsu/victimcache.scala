//******************************************************************************
// Copyright (c) 2017 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// VictimCache
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.lsu

import chisel3._
import chisel3.util._
import chisel3.util.random._
import chisel3.internal.sourceinfo.{SourceInfo}
import chisel3.experimental.{chiselName}

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._
import chisel3.util.experimental.BoringUtils


import boom.common._
import boom.util.{BoomCoreStringPrefix}

/**
 * Victim Cache module
 *
 * @param victimCacheParams parameters for the victim cache
 * @param enableBlackBox use a blackbox icache
 */
class VictimCache(
  val victimCacheParams: DVictimCacheParams,
  val staticIdForMetadataUseOnly: Int)(implicit p: Parameters)
  extends LazyModule
{
  lazy val module = new VictimCacheModule(this)

  val size = victimCacheParams.nSets * victimCacheParams.nWays * victimCacheParams.blockBytes
}

/**
 * IO Signals leaving the VictimCache
 *
 * @param rowBits bits per row
 */
class VictimCacheResp(val rowBits: Int) extends Bundle
{
  val data = UInt(rowBits.W)
  val source  = Output(UInt(4.W)) // to
  val addr = Output(UInt(3.W))
  val refill_done = Output(Bool())
}

/**
 * IO Signals coming from icache to VictimCache
 *
 * @param paddrBits physical address bits
 */
class VictimCacheReq(val paddrBits: Int) extends Bundle
{
  val addr = UInt(paddrBits.W)
  val source  = UInt(4.W) // to
}

/**
 * IO Signals coming from icache to VictimCache
 *
 * @param outer top level VictimCache class
 */
class VictimCacheEvictedData(val outer: VictimCache) extends Bundle
{
  val data = UInt((outer.victimCacheParams.rowBits).W)
  val addr = UInt(width = outer.victimCacheParams.paddrBits.W)
  // val tag = UInt(tagBits.W)
}

/*
 * IO Signals for interacting with the VictimCache
 *
 * @param outer top level VictimCache class
 */
class VictimCacheBundle(val outer: VictimCache) extends BoomBundle()(outer.p)
  with HasL1HellaCacheParameters
{
  val req = Flipped(Decoupled(new VictimCacheReq(outer.victimCacheParams.paddrBits)))
  val evicted_data = Flipped(Decoupled(new VictimCacheEvictedData(outer)))

  val resp = Decoupled(new VictimCacheResp(outer.victimCacheParams.rowBits))
  val invalidate = Input(Bool())
}


/**
 * Main VictimCache module
 *
 * @param outer top level VictimCache class
 */
@chiselName
class VictimCacheModule(outer: VictimCache) extends LazyModuleImp(outer)
  with HasL1HellaCacheParameters
{
  val io = IO(new VictimCacheBundle(outer))

  val enable_vdc = Wire(Bool())
  enable_vdc := true.B
  // BoringUtils.addSink(enable_vdc, "enablevdc")

  val victim_blockOffBits = log2Ceil(outer.victimCacheParams.blockBytes)
  val victim_tagBits = outer.victimCacheParams.paddrBits - victim_blockOffBits
  val victim_untagBits = victim_blockOffBits

  require(isPow2(outer.victimCacheParams.nSets)) // && isPow2(outer.victimCacheParams.nWays))

 
  val src = RegEnable(io.req.bits.source, io.req.fire())
  
  val s0_req_cnt = RegInit((0.U((log2Ceil(refillCycles + 1)).W))) 
  val s1_req_cnt = RegInit((0.U((log2Ceil(refillCycles + 1)).W))) 
  val req_in_progress = RegInit(false.B)  // Is set for 7 cycles after request is received
  val s0_valid = io.req.fire()
  val s0_req_in_progress = s0_valid || req_in_progress  // Is set for 8 cycle while we are satisfying a request
  val s1_req_in_progress = RegNext(s0_req_in_progress)
  val s2_req_in_progress = RegNext(s1_req_in_progress)

  s0_req_cnt := 0.U
  s1_req_cnt := 1.U

  when (io.req.fire()) { req_in_progress := true.B }
  when (s1_req_cnt >= refillCycles.U) { req_in_progress := false.B }

  when (s0_valid || req_in_progress) {
    when (io.resp.ready) {
      s0_req_cnt := s1_req_cnt
      s1_req_cnt := s1_req_cnt + 1.U
    } .otherwise {
      s0_req_cnt := s0_req_cnt
      s1_req_cnt := s1_req_cnt
    }
  }

  // Request signals
  val s0_paddr = RegEnable(io.req.bits.addr, s0_valid)

  val s1_valid = RegNext(s0_valid)
  val s1_tag_hit = Wire(Vec(outer.victimCacheParams.nWays, Bool()))
  val s1_hit = s1_tag_hit.reduce(_||_)
  val s2_valid = RegNext(s1_valid)
  val s2_hit = RegNext(s1_hit)

  // Refill signals
  val s0_refill_valid = io.evicted_data.fire()
  val s0_refill_paddr = io.evicted_data.bits.addr
  val s1_refill_valid = RegNext(s0_refill_valid)
  val s2_refill_valid = RegNext(s1_refill_valid)
  val refill_paddr = RegEnable(s0_refill_paddr, s0_refill_valid && !s1_refill_valid)
  val refill_tag = refill_paddr(victim_tagBits+victim_untagBits-1,victim_untagBits)
  val s0_refill_cnt = RegInit((0.U((log2Ceil(refillCycles + 1)).W))) // Tracks fire()
  val s1_refill_cnt = RegInit((0.U((log2Ceil(refillCycles + 1)).W))) // Tracks fire()
  val s0_refill_valid_cnt = RegInit((0.U((log2Ceil(refillCycles + 1)).W))) // Tracks valid
  val s1_refill_valid_cnt = RegInit((0.U((log2Ceil(refillCycles + 1)).W))) // Tracls valid

  val s1_refill_tag_hit = Wire(Vec(outer.victimCacheParams.nWays, Bool()))
  val s2_refill_tag_hit = RegNext(s1_refill_tag_hit)
  val s2_refill_hit_way = OHToUInt(s2_refill_tag_hit)
  val s1_refill_hit = s1_refill_tag_hit.reduce(_||_)
  val s2_refill_hit = RegNext(s1_refill_hit)

  val refill_in_progress = RegInit(false.B)
  val refill_success = (s0_refill_valid_cnt === s0_refill_cnt)
  val refill_done = !s0_refill_valid && s1_refill_valid && !refill_in_progress

  when (io.evicted_data.valid) { refill_in_progress := true.B }
  when (s1_refill_valid_cnt >= refillCycles.U) { refill_in_progress := false.B }

  s0_refill_cnt := 0.U
  s1_refill_cnt := 1.U

  when (s0_refill_valid) {
    s0_refill_cnt := s1_refill_cnt
    s1_refill_cnt := s1_refill_cnt + 1.U
  } .elsewhen (refill_in_progress) {
    s0_refill_cnt := s0_refill_cnt
    s1_refill_cnt := s1_refill_cnt
  }

  s0_refill_valid_cnt := 0.U
  s1_refill_valid_cnt := 1.U

  when (io.evicted_data.valid) {
    s0_refill_valid_cnt := s1_refill_valid_cnt
    s1_refill_valid_cnt := s1_refill_valid_cnt + 1.U
  } .elsewhen (refill_in_progress) {
    s0_refill_valid_cnt := s0_refill_valid_cnt
    s1_refill_valid_cnt := s1_refill_valid_cnt
  }

  val invalidated = RegInit(false.B)



  val repl = new PseudoLRU(outer.victimCacheParams.nWays)


  io.req.ready := enable_vdc && !s0_refill_valid && !s1_refill_valid && !s2_refill_valid && !req_in_progress && !RegNext(req_in_progress)    // Don't receive a request while refilling (might have small overhead)
  io.evicted_data.ready := enable_vdc && !req_in_progress   // Don't receive an evicted block when satisfying a request

  val tag_array = SyncReadMem(outer.victimCacheParams.nSets, Vec(outer.victimCacheParams.nWays, UInt(victim_tagBits.W)))
  val tag_rdata = tag_array.read(0.U, (s0_valid || s0_refill_valid) && !refill_done)
  when (refill_done) { // tag write when reffiling from l1$
    tag_array.write(0.U, VecInit(Seq.fill(outer.victimCacheParams.nWays)(refill_tag)), Seq.tabulate(outer.victimCacheParams.nWays)(repl.way === _.U))
  }

  val vb_array = RegInit(0.U((outer.victimCacheParams.nSets*outer.victimCacheParams.nWays).W))    // Valid bits
  when (refill_done) {
    when (s2_refill_hit) {
      vb_array := vb_array.bitSet(repl.way, !invalidated && refill_success).bitSet(s2_refill_hit_way, false.B)
    } .otherwise {
      vb_array := vb_array.bitSet(repl.way, !invalidated && refill_success)      
    }
    invalidated := false.B
    repl.access(repl.way)
  }

  when (io.invalidate) {
    vb_array := 0.U
    invalidated := true.B
  }

  val s2_dout   = Wire(Vec(outer.victimCacheParams.nWays, UInt(rowBits.W)))

  for (i <- 0 until outer.victimCacheParams.nWays) {
    val s1_idx = 0.U //s1_paddr(victim_untagBits-1,victim_blockOffBits)
    val s1_tag = s0_paddr(victim_tagBits+victim_untagBits-1,victim_untagBits)
    val s1_vb = vb_array(i.U)
    val tag = tag_rdata(i)
    s1_tag_hit(i) := s1_vb && tag === s1_tag
  }

  for (i <- 0 until outer.victimCacheParams.nWays) {
    val s1_refill_idx = 0.U //s1_paddr(victim_untagBits-1,victim_blockOffBits)
    val s1_refill_tag = s0_refill_paddr(victim_tagBits+victim_untagBits-1,victim_untagBits)
    val s1_refill_vb = vb_array(i.U)
    val tag = tag_rdata(i)
    s1_refill_tag_hit(i) := s1_refill_vb && tag === s1_refill_tag
  }
  assert(PopCount(s1_tag_hit) <= 1.U || !s1_valid, "%d %d", PopCount(s1_tag_hit), !s1_valid)

  val ramDepth = outer.victimCacheParams.nSets

  val dataArrays = {
    // Use unbanked victimcache for narrow accesses.
    (0 until outer.victimCacheParams.nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayWay_${x}",
        desc = "Victim Cache Data Array",
        size = outer.victimCacheParams.nSets * refillCycles,
        data = UInt((rowBits).W)
      )
    }
  }
  // Use unbanked victimcache for narrow accesses.
  for ((dataArray, i) <- dataArrays.map(_._1) zipWithIndex) {
    
    val s0_ren = s0_req_in_progress && io.resp.ready
    val wen = s0_refill_valid && repl.way === i.U && (s0_refill_valid_cnt === s0_refill_cnt)     // Whether we are writing to this block

    s2_dout(i) := RegNext(dataArray.read(s0_paddr | s0_req_cnt, !wen && s0_ren))
    when (wen) {
      dataArray.write(s0_refill_cnt, io.evicted_data.bits.data)
    }
  }
  val s2_tag_hit = RegNext(s1_tag_hit)
  val s2_hit_way = OHToUInt(s2_tag_hit)
  when (s2_hit && (s2_req_in_progress && !s1_req_in_progress)) {
    vb_array := vb_array.bitSet(s2_hit_way, false.B)
    // when (s2_hit_way =/= 0.U && vb_array(0.U)) {
    //   repl.access(0.U)
    // }
    // when (s2_hit_way =/= 1.U && vb_array(1.U)) {
    //   repl.access(1.U)
    // }
    // when (s2_hit_way =/= 2.U && vb_array(2.U)) {
    //   repl.access(2.U)
    // }
    // when (s2_hit_way =/= 3.U && vb_array(3.U)) {
    //   repl.access(3.U)
    // }
  }
  val s2_way_mux = Mux1H(s2_tag_hit, s2_dout)

  val s2_data = s2_way_mux

  io.resp.bits.data := s2_data
  io.resp.valid := enable_vdc && s2_req_in_progress && s2_hit
  io.resp.bits.source := src
  io.resp.bits.addr := RegNext(RegNext(s0_req_cnt))
  io.resp.bits.refill_done := enable_vdc && (s2_req_in_progress && !s1_req_in_progress) && s2_hit

  // when (s2_req_in_progress && s2_hit) {
  //   printf("Victim - %b\n", s2_data)
  // }

}


