//******************************************************************************
// Copyright (c) 2017 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// VictimCache
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.ifu

import chisel3._
import chisel3.util._
import chisel3.util.random._
import chisel3.internal.sourceinfo.{SourceInfo}
import chisel3.experimental.{chiselName}

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.subsystem.{RocketTilesKey}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import freechips.rocketchip.rocket.{HasL1ICacheParameters, ICacheParams, ICacheErrors, ICacheReq}
import freechips.rocketchip.diplomaticobjectmodel.logicaltree.{LogicalTreeNode}
import freechips.rocketchip.diplomaticobjectmodel.DiplomaticObjectModelAddressing
import freechips.rocketchip.diplomaticobjectmodel.model.{OMComponent, OMICache, OMECC}
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
  val victimCacheParams: VictimCacheParams,
  val staticIdForMetadataUseOnly: Int)(implicit p: Parameters)
  extends LazyModule
{
  lazy val module = new VictimCacheModule(this)

  val size = victimCacheParams.nSets * victimCacheParams.nWays * victimCacheParams.blockBytes
  private val wordBytes = victimCacheParams.fetchBytes
}

/**
 * IO Signals leaving the VictimCache
 *
 * @param outer top level VictimCache class
 */
class VictimCacheResp(val outer: VictimCache) extends Bundle
{
  val data = UInt((outer.victimCacheParams.blockBytes*8).W)
}

/**
 * IO Signals coming from icache to VictimCache
 *
 * @param outer top level VictimCache class
 */
class VictimCacheReq(val outer: VictimCache) extends Bundle
{
  val addr = UInt(width = outer.victimCacheParams.paddrBits.W)
}

/**
 * IO Signals coming from icache to VictimCache
 *
 * @param outer top level VictimCache class
 */
class VictimCacheEvictedData(val outer: VictimCache) extends Bundle
{
  val data = UInt((outer.victimCacheParams.blockBytes*8).W)
  val addr = UInt(width = outer.victimCacheParams.paddrBits.W)
  // val tag = UInt(tagBits.W)
}

/*
 * IO Signals for interacting with the VictimCache
 *
 * @param outer top level VictimCache class
 */
class VictimCacheBundle(val outer: VictimCache) extends BoomBundle()(outer.p)
  with HasBoomFrontendParameters
{
  val req = Flipped(Decoupled(new VictimCacheReq(outer)))
  val evicted_data = Flipped(Decoupled(new VictimCacheEvictedData(outer)))

  val resp = Valid(new VictimCacheResp(outer))
  // val invalidate = Input(Bool())
}


/**
 * Main VictimCache module
 *
 * @param outer top level VictimCache class
 */
@chiselName
class VictimCacheModule(outer: VictimCache) extends LazyModuleImp(outer)
  with HasBoomFrontendParameters
{
  val io = IO(new VictimCacheBundle(outer))

  val enable_vic = Wire(Bool())
  enable_vic := true.B
  // BoringUtils.addSink(enable_vic, "enablevic")

  val victim_blockOffBits = log2Ceil(outer.victimCacheParams.blockBytes)
  val victim_tagBits = outer.victimCacheParams.paddrBits - victim_blockOffBits
  val victim_untagBits = victim_blockOffBits

  require(isPow2(outer.victimCacheParams.nSets)) // && isPow2(outer.victimCacheParams.nWays))

  // How many bits do we intend to fetch at most every cycle?
  val wordBits = outer.victimCacheParams.blockBytes*8



  val s0_valid = io.req.fire()
  val s0_paddr = io.req.bits.addr

  val s1_paddr = RegNext(s0_paddr)
  val s1_valid = RegNext(s0_valid)
  val s1_tag_hit = Wire(Vec(outer.victimCacheParams.nWays, Bool()))
  val s1_hit = s1_tag_hit.reduce(_||_)
  val s2_valid = RegNext(s1_valid)
  val s2_hit = RegNext(s1_hit)

  val s0_refill_paddr = io.evicted_data.bits.addr
  val s1_refill_paddr = RegNext(s0_refill_paddr)
  val refill_valid = io.evicted_data.fire()
  val refill_paddr = RegEnable(s1_refill_paddr, s1_valid)
  val refill_tag = refill_paddr(victim_tagBits+victim_untagBits-1,victim_untagBits)
  val refill_idx = 0.U //refill_paddr(victim_untagBits-1,victim_blockOffBits)



  // val repl_way = LFSR(16, refill_valid)(log2Ceil(outer.victimCacheParams.nWays)-1,0) // which block in a set we are replacing
  val repl = new PseudoLRU(outer.victimCacheParams.nWays)


  io.req.ready := enable_vic    // MYTODO: figure out what to do with this
  io.evicted_data.ready := enable_vic

  val tag_array = SyncReadMem(outer.victimCacheParams.nSets, Vec(outer.victimCacheParams.nWays, UInt(victim_tagBits.W)))
  val tag_rdata = tag_array.read(0.U, s0_valid)
  when (RegNext(RegNext(refill_valid))) { // tag write when reffiling from l1$
    tag_array.write(refill_idx, VecInit(Seq.fill(outer.victimCacheParams.nWays)(refill_tag)), Seq.tabulate(outer.victimCacheParams.nWays)(repl.way === _.U))
  }

  // MYTODO: For dcache v$, invalidate blocks when they leave v$
  val vb_array = RegInit(0.U((outer.victimCacheParams.nSets*outer.victimCacheParams.nWays).W))    // Valid bits
  when (RegNext(RegNext(RegNext(refill_valid)))) {
    vb_array := vb_array.bitSet(repl.way, true.B)
    repl.access(repl.way)
  }

  val s2_dout   = Wire(Vec(outer.victimCacheParams.nWays, UInt(wordBits.W)))

  for (i <- 0 until outer.victimCacheParams.nWays) {
    val s1_idx = 0.U //s1_paddr(victim_untagBits-1,victim_blockOffBits)
    val s1_tag = s1_paddr(victim_tagBits+victim_untagBits-1,victim_untagBits)
    val s1_vb = vb_array(i.U)
    val tag = tag_rdata(i)
    s1_tag_hit(i) := s1_vb && tag === s1_tag
  }
  assert(PopCount(s1_tag_hit) <= 1.U || !s1_valid, "%d %d", PopCount(s1_tag_hit), !s1_valid)

  val ramDepth = outer.victimCacheParams.nSets

  val dataArrays = {
    // Use unbanked victimcache for narrow accesses.
    (0 until outer.victimCacheParams.nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayWay_${x}",
        desc = "Victim Cache Data Array",
        size = outer.victimCacheParams.nSets,
        data = UInt((wordBits).W)
      )
    }
  }
  // Use unbanked victimcache for narrow accesses.
  for ((dataArray, i) <- dataArrays.map(_._1) zipWithIndex) {
    
    val s0_ren = s0_valid
    val wen = RegNext(refill_valid) && repl.way === i.U     // Whether we are writing to this block

    val mem_idx = refill_idx    // idx of the read or write

    s2_dout(i) := RegNext(dataArray.read(mem_idx, !wen && s0_ren))
    when (wen) {
      dataArray.write(mem_idx, RegNext(io.evicted_data.bits.data))
    }
  }
  val s2_tag_hit = RegNext(s1_tag_hit)
  val s2_hit_way = OHToUInt(s2_tag_hit)
  val s1_hit_way = OHToUInt(s1_tag_hit)
  when (s2_valid && s2_hit) {
    vb_array := vb_array.bitSet(s2_hit_way, false.B)
  }
  val s2_way_mux = Mux1H(s2_tag_hit, s2_dout)

  val s2_data = s2_way_mux

  io.resp.bits.data := s2_data
  io.resp.valid := enable_vic && s2_valid && s2_hit

  override def toString: String = BoomCoreStringPrefix(
    "==VictimCache-ICache==",
    "Fetch bytes   : " + cacheParams.fetchBytes,
    "Block bytes   : " + (1 << victim_blockOffBits),
    "Row bytes     : " + rowBytes,
    "Word bits     : " + wordBits,
    "Sets          : " + outer.victimCacheParams.nSets,
    "Ways          : " + outer.victimCacheParams.nWays,
    "Refill cycles : " + refillCycles,
    "RAMs          : (" +  wordBits/nBanks + " x " + outer.victimCacheParams.nSets*refillCycles + ") using " + nBanks + " banks",
    "" + (if (nBanks == 2) "Dual-banked" else "Single-banked"),
    "I-TLB ways    : " + cacheParams.nTLBWays + "\n")
}


