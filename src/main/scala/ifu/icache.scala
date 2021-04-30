//******************************************************************************
// Copyright (c) 2017 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// ICache
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

import boom.common._
import boom.util.{BoomCoreStringPrefix}

case class VictimCacheParams(
    nSets: Int = 1,
    nWays: Int = 8,
    rowBits: Int = 128,
    nTLBSets: Int = 1,
    nTLBWays: Int = 32,
    tagECC: Option[String] = None,
    dataECC: Option[String] = None,
    itimAddr: Option[BigInt] = None,
    prefetch: Boolean = false,
    blockBytes: Int = 64,
    latency: Int = 2,
    paddrBits: Int = 56,
    fetchBytes: Int = 4) extends L1CacheParams {
  def tagCode: Code = Code.fromString(tagECC)
  def dataCode: Code = Code.fromString(dataECC)
  def replacement = new RandomReplacement(nWays)
}

/**
 * ICache module
 *
 * @param icacheParams parameters for the icache
 * @param hartId the id of the hardware thread in the cache
 * @param enableBlackBox use a blackbox icache
 */
class ICache(
  val icacheParams: ICacheParams,
  val staticIdForMetadataUseOnly: Int)(implicit p: Parameters)
  extends LazyModule
{
  lazy val module = new ICacheModule(this)
  val masterNode = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
    sourceId = IdRange(0, 1 + icacheParams.prefetch.toInt), // 0=refill, 1=hint
    name = s"Core ${staticIdForMetadataUseOnly} ICache")))))

  val victimCacheParams = if (icacheParams.useVictimCache) {
    VictimCacheParams(rowBits = icacheParams.rowBits, nSets=1, nWays=icacheParams.victimWays, paddrBits=32, fetchBytes=icacheParams.fetchBytes)
  } else {
    VictimCacheParams(rowBits = 1, nSets=1, nWays=1, paddrBits=32, fetchBytes=1)
  }

  val victimCache = LazyModule(new boom.ifu.VictimCache(victimCacheParams, staticIdForMetadataUseOnly))

  val size = icacheParams.nSets * icacheParams.nWays * icacheParams.blockBytes
  private val wordBytes = icacheParams.fetchBytes
}
class BoomICacheLogicalTreeNode(icache: ICache, deviceOpt: Option[SimpleDevice], params: ICacheParams) extends LogicalTreeNode(() => deviceOpt) {
  override def getOMComponents(resourceBindings: ResourceBindings, children: Seq[OMComponent] = Nil): Seq[OMComponent] = {
    Seq(
      OMICache(
        memoryRegions = DiplomaticObjectModelAddressing.getOMMemoryRegions("ITIM", resourceBindings),
        interrupts = Nil,
        nSets = params.nSets,
        nWays = params.nWays,
        blockSizeBytes = params.blockBytes,
        dataMemorySizeBytes = params.nSets * params.nWays * params.blockBytes,
        dataECC = params.dataECC.map(OMECC.fromString),
        tagECC = params.tagECC.map(OMECC.fromString),
        nTLBEntries = params.nTLBSets * params.nTLBWays,
        nTLBSets = params.nTLBSets,
        nTLBWays = params.nTLBWays,
        maxTimSize = params.nSets * (params.nWays-1) * params.blockBytes,
        memories = icache.module.asInstanceOf[ICacheModule].dataArrays.map(_._2)
      )
    )
  }
}

/**
 * IO Signals leaving the ICache
 *
 * @param outer top level ICache class
 */
class ICacheResp(val outer: ICache) extends Bundle
{
  val data = UInt((outer.icacheParams.fetchBytes*8).W)
  val replay = Bool()
  val ae = Bool()
}

/**
 * IO Signals for interacting with the ICache
 *
 * @param outer top level ICache class
 */
class ICacheBundle(val outer: ICache) extends BoomBundle()(outer.p)
  with HasBoomFrontendParameters
{
  val req = Flipped(Decoupled(new ICacheReq))
  val s1_paddr = Input(UInt(paddrBits.W)) // delayed one cycle w.r.t. req

  val s1_kill = Input(Bool()) // delayed one cycle w.r.t. req
  val s2_kill = Input(Bool()) // delayed two cycles; prevents I$ miss emission
  val s2_prefetch = Input(Bool()) // should I$ prefetch next line on a miss?

  val resp = Valid(new ICacheResp(outer))
  val invalidate = Input(Bool())

  val perf = Output(new Bundle {
    val acquire = Bool()
  })
}

/**
 * Get a tile-specific property without breaking deduplication
 */
object GetPropertyByHartId
{
  def apply[T <: Data](tiles: Seq[RocketTileParams], f: RocketTileParams => Option[T], hartId: UInt): T = {
    PriorityMux(tiles.collect { case t if f(t).isDefined => (t.hartId.U === hartId) -> f(t).get })
  }
}


/**
 * Main ICache module
 *
 * @param outer top level ICache class
 */
@chiselName
class ICacheModule(outer: ICache) extends LazyModuleImp(outer)
  with HasBoomFrontendParameters
{
  val wordBits = if (outer.icacheParams.useVictimCache) {
    outer.icacheParams.fetchBytes*8
  } else {
    outer.icacheParams.fetchBytes*8
  }

  val dataArrays = if (nBanks == 1) {
    // Use unbanked icache for narrow accesses.
    if (outer.icacheParams.useVictimCache) {
      (0 until nWays*refillCycles).map { x =>
        DescribedSRAM(
          name = s"dataArrayWay_${x}",
          desc = "ICache Data Array",
          size = nSets,
          data = UInt((wordBits).W)
        )
      }
    } else {
      // Use unbanked icache for narrow accesses.
      (0 until nWays).map { x =>
        DescribedSRAM(
          name = s"dataArrayWay_${x}",
          desc = "ICache Data Array",
          size = nSets * refillCycles,
          data = UInt((wordBits).W)
        )
      }
    }
  } else {
    // Use two banks, interleaved.
    (0 until nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayB0Way_${x}",
        desc = "ICache Data Array",
        size = nSets * refillCycles,
        data = UInt((wordBits/nBanks).W)
      )} ++
    (0 until nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayB1Way_${x}",
        desc = "ICache Data Array",
        size = nSets * refillCycles,
        data = UInt((wordBits/nBanks).W)
      )}
  }

  val io = IO(new ICacheBundle(outer))

  if (outer.icacheParams.useVictimCache) {
    val enableICacheDelay = tileParams.core.asInstanceOf[BoomCoreParams].enableICacheDelay

    val victimCache = outer.victimCache.module

    val (tl_out, edge_out) = outer.masterNode.out(0)

    require(isPow2(nSets) && isPow2(nWays))
    require(usingVM)
    require(pgIdxBits >= untagBits)

    // How many bits do we intend to fetch at most every cycle?
    // Each of these cases require some special-case handling.
    require (tl_out.d.bits.data.getWidth == wordBits || (2*tl_out.d.bits.data.getWidth == wordBits && nBanks == 2))
    // If TL refill is half the wordBits size and we have two banks, then the
    // refill writes to only one bank per cycle (instead of across two banks every
    // cycle).
    val refillsToOneBank = (2*tl_out.d.bits.data.getWidth == wordBits)



    val s0_valid = io.req.fire()
    val s0_vaddr = io.req.bits.addr

    val s1_valid = RegNext(s0_valid)
    val s1_tag_hit = Wire(Vec(nWays, Bool()))
    val s1_hit = s1_tag_hit.reduce(_||_)
    val s2_valid = RegNext(s1_valid && !io.s1_kill)
    val s2_hit = RegNext(s1_hit)


    val invalidated = Reg(Bool())
    val refill_valid = RegInit(false.B)
    val refill_fire = tl_out.a.fire()
    val s2_miss = s2_valid && !s2_hit && !RegNext(refill_valid)
    val refill_paddr = RegEnable(io.s1_paddr, s1_valid && !(refill_valid || s2_miss))
    val refill_tag = refill_paddr(tagBits+untagBits-1,untagBits)
    val refill_idx = refill_paddr(untagBits-1,blockOffBits)
    val refill_one_beat = tl_out.d.fire() && edge_out.hasData(tl_out.d.bits)

    // io.req.ready := !refill_one_beat

    val (_, _, d_done, refill_cnt) = edge_out.count(tl_out.d)
    val refill_done = refill_one_beat && d_done
    tl_out.d.ready := true.B
    require (edge_out.manager.minLatency > 0)

    val early_repl_way = if (isDM) 0.U else LFSR(16, s0_valid)(log2Ceil(nWays)-1,0)
    val repl_way = RegInit(0.U)
    when (refill_fire) {
      repl_way := early_repl_way
    }
    // val repl_way = if (isDM) 0.U else LFSR(16, refill_fire)(log2Ceil(nWays)-1,0)

    val tag_array = SyncReadMem(nSets, Vec(nWays, UInt(tagBits.W)))
    // val tag_rdata = tag_array.read(s0_vaddr(untagBits-1, blockOffBits), !refill_done && s0_valid)
    // when (refill_done) {
    //   tag_array.write(refill_idx, VecInit(Seq.fill(nWays)(refill_tag)), Seq.tabulate(nWays)(repl_way === _.U))
    // }

    val vb_array = RegInit(0.U((nSets*nWays).W))
    // when (refill_one_beat) {
    //   vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), refill_done && !invalidated)
    // }

    when (io.invalidate) {
      vb_array := 0.U
      invalidated := true.B
    }

    val s2_dout   = Wire(Vec(refillCycles, Vec(nWays, UInt(wordBits.W))))
    val s1_bankid = Wire(Bool())

    // ******** Added signals *********
    val s0_word_hit = Wire(Vec(refillCycles, Bool()))
    val s2_victim_dout = Wire(Vec(nWays, Vec(refillCycles, UInt(wordBits.W))))


    val victim_refill_fire = victimCache.io.req.fire()
    val victim_refill_valid = RegInit(false.B)
    when (victim_refill_fire) { victim_refill_valid := true.B }
    val victim_refill_done = victimCache.io.resp.valid

    val victim_updated = Reg(Bool())
    io.req.ready := !(refill_one_beat && !victim_updated) && !victim_refill_done
    val tag_rdata = tag_array.read(s0_vaddr(untagBits-1, blockOffBits), !(refill_done && !victim_updated) && !victim_refill_done && s0_valid)


    when (victim_refill_done) {victim_updated := true.B}  // This just tells us that we don't need to update L1 using L2 repsonse

    
    when ((refill_done && !victim_updated) || victim_refill_done) {
      tag_array.write(refill_idx, VecInit(Seq.fill(nWays)(refill_tag)), Seq.tabulate(nWays)(repl_way === _.U))
    }

    when ((refill_one_beat && !victim_updated) || victim_refill_done) {
      vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), (refill_done || victim_refill_done) && !invalidated)
    }

    val s2_tag_rdata = RegNext(tag_rdata)
    val s2_tag_value = s2_tag_rdata(early_repl_way)
    val s2_paddr = RegNext(io.s1_paddr)

    // ************* end **************
    for (i <- 0 until nWays) {
      val s1_idx = io.s1_paddr(untagBits-1,blockOffBits)
      val s1_tag = io.s1_paddr(tagBits+untagBits-1,untagBits)
      val s1_vb = vb_array(Cat(i.U, s1_idx))
      val tag = tag_rdata(i)
      s1_tag_hit(i) := s1_vb && tag === s1_tag
    }
    assert(PopCount(s1_tag_hit) <= 1.U || !s1_valid)

    val ramDepth = if (refillsToOneBank && nBanks == 2) {
      nSets * refillCycles / 2
    } else {
      nSets
    }
    if (nBanks == 1) {
      // Use unbanked icache for narrow accesses.
      s1_bankid := 0.U
      for ((dataArray, idx) <- dataArrays.map(_._1) zipWithIndex) {
        val i = idx % 2
        val j = idx / 2
        def offset(addr: UInt) = addr(blockOffBits-1, blockOffBits-log2Ceil(refillCycles))
        val selected_word = offset(s0_vaddr) === j.U
        if (i == 0)
          s0_word_hit(j) := selected_word
        def row(addr: UInt) = addr(untagBits-1, blockOffBits)
        val s0_ren = s0_valid

        val wen = (refill_one_beat && !invalidated && !victim_updated) && repl_way === i.U && refill_cnt === j.U

        val mem_idx = Mux((refill_one_beat && !victim_updated) || victim_refill_done, refill_idx,
                      row(s0_vaddr))

        // ********* To evict a block to v$ **********
        s2_victim_dout(i)(j) := s2_dout(j)(i)
        val victim_wen = victim_refill_done && repl_way === i.U
        def beat(block: UInt, index: Int) = block(wordBits*(index+1)-1, wordBits*(index))
        when (victim_wen) {
          dataArray.write(mem_idx, beat(victimCache.io.resp.bits.data, j))
        }
        // ****************** end ********************

        when (wen) {
          dataArray.write(mem_idx, tl_out.d.bits.data)
        }
        if (enableICacheDelay)
          s2_dout(j)(i) := dataArray.read(RegNext(mem_idx), RegNext(!victim_wen && !wen && s0_ren))
        else
          s2_dout(j)(i) := RegNext(dataArray.read(mem_idx, !victim_wen && !wen && s0_ren))
      }
    } else {
      // Use two banks, interleaved.
      val dataArraysB0 = dataArrays.map(_._1).take(nWays)
      val dataArraysB1 = dataArrays.map(_._1).drop(nWays)
      require (nBanks == 2)

      // Bank0 row's id wraps around if Bank1 is the starting bank.
      def b0Row(addr: UInt) =
        if (refillsToOneBank) {
          addr(untagBits-1, blockOffBits-log2Ceil(refillCycles)+1) + bank(addr)
        } else {
          addr(untagBits-1, blockOffBits-log2Ceil(refillCycles)) + bank(addr)
        }
      // Bank1 row's id stays the same regardless of which Bank has the fetch address.
      def b1Row(addr: UInt) =
        if (refillsToOneBank) {
          addr(untagBits-1, blockOffBits-log2Ceil(refillCycles)+1)
        } else {
          addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
        }

      s1_bankid := RegNext(bank(s0_vaddr))

      for (i <- 0 until nWays) {
        val s0_ren = s0_valid
        val wen = (refill_one_beat && !invalidated)&& repl_way === i.U

        var mem_idx0: UInt = null
        var mem_idx1: UInt = null

        if (refillsToOneBank) {
          // write a refill beat across only one beat.
          mem_idx0 =
            Mux(refill_one_beat, (refill_idx << (log2Ceil(refillCycles)-1)) | (refill_cnt >> 1.U),
            b0Row(s0_vaddr))
          mem_idx1 =
            Mux(refill_one_beat, (refill_idx << (log2Ceil(refillCycles)-1)) | (refill_cnt >> 1.U),
            b1Row(s0_vaddr))

          when (wen && refill_cnt(0) === 0.U) {
            dataArraysB0(i).write(mem_idx0, tl_out.d.bits.data)
          }
          when (wen && refill_cnt(0) === 1.U) {
            dataArraysB1(i).write(mem_idx1, tl_out.d.bits.data)
          }
        } else {
          // write a refill beat across both banks.
          mem_idx0 =
            Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
            b0Row(s0_vaddr))
          mem_idx1 =
            Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
            b1Row(s0_vaddr))

          when (wen) {
            val data = tl_out.d.bits.data
            dataArraysB0(i).write(mem_idx0, data(wordBits/2-1, 0))
            dataArraysB1(i).write(mem_idx1, data(wordBits-1, wordBits/2))
          }
        }
        if (enableICacheDelay) {
          s2_dout(i) := Cat(dataArraysB1(i).read(RegNext(mem_idx1), RegNext(!wen && s0_ren)),
                            dataArraysB0(i).read(RegNext(mem_idx0), RegNext(!wen && s0_ren)))
        } else {
          s2_dout(i) := RegNext(Cat(dataArraysB1(i).read(mem_idx1, !wen && s0_ren),
                                    dataArraysB0(i).read(mem_idx0, !wen && s0_ren)))
        }
      }
    }
    // ******** Added signals *********
    val s2_word_hit = RegNext(RegNext(s0_word_hit))
    assert(PopCount(s0_word_hit) <= 1.U || !s0_valid)
    val s2_victim_data = s2_victim_dout(early_repl_way).asUInt
    val s2_vb = vb_array(Cat(early_repl_way, s2_paddr(untagBits-1,blockOffBits)))
    victimCache.io.evicted_data.valid := s2_vb && s2_miss && !victim_refill_valid && !refill_valid && !io.s2_kill
    victimCache.io.evicted_data.bits.data := s2_victim_data
    victimCache.io.evicted_data.bits.addr := Cat(s2_tag_value, s2_paddr(untagBits-1,0))
    // ************* end **************
    val s2_tag_hit = RegNext(s1_tag_hit)
    val s2_hit_way = OHToUInt(s2_tag_hit)
    val s2_bankid = RegNext(s1_bankid)
    val s2_way_mux = Mux1H(s2_tag_hit, Mux1H(s2_word_hit, s2_dout))

    val s2_unbanked_data = s2_way_mux
    val sz = s2_way_mux.getWidth
    val s2_bank0_data = s2_way_mux(sz/2-1,0)
    val s2_bank1_data = s2_way_mux(sz-1,sz/2)

    val s2_data =
      if (nBanks == 2) {
        Mux(s2_bankid,
          Cat(s2_bank0_data, s2_bank1_data),
          Cat(s2_bank1_data, s2_bank0_data))
      } else {
        s2_unbanked_data
      }

    io.resp.bits.data := s2_data
    io.resp.valid := s2_valid && s2_hit

    tl_out.a.valid := s2_miss && !refill_valid && !io.s2_kill
    tl_out.a.bits := edge_out.Get(
      fromSource = 0.U,
      toAddress = (refill_paddr >> blockOffBits) << blockOffBits,
      lgSize = lgCacheBlockBytes.U)._2
    tl_out.b.ready := true.B
    tl_out.c.valid := false.B
    tl_out.e.valid := false.B

    io.perf.acquire := tl_out.a.fire()

    when (!refill_valid) { 
      victim_updated := false.B
      invalidated := false.B }
    when (refill_fire) { refill_valid := true.B }
    when (refill_done) { refill_valid := false.B
      victim_refill_valid := false.B 
    }

    // ******** Added signals *********
    victimCache.io.req.valid := s2_miss && !victim_refill_valid && !refill_valid && !io.s2_kill // request victim cache access if we missed and haven't already requested it yet
    victimCache.io.req.bits.addr := (refill_paddr >> blockOffBits) << blockOffBits  // the physical address requested
    // ************* end **************
  } else {
    val enableICacheDelay = tileParams.core.asInstanceOf[BoomCoreParams].enableICacheDelay

    val (tl_out, edge_out) = outer.masterNode.out(0)

    require(isPow2(nSets) && isPow2(nWays))
    require(usingVM)
    require(pgIdxBits >= untagBits)

    // How many bits do we intend to fetch at most every cycle?
    // Each of these cases require some special-case handling.
    require (tl_out.d.bits.data.getWidth == wordBits || (2*tl_out.d.bits.data.getWidth == wordBits && nBanks == 2))
    // If TL refill is half the wordBits size and we have two banks, then the
    // refill writes to only one bank per cycle (instead of across two banks every
    // cycle).
    val refillsToOneBank = (2*tl_out.d.bits.data.getWidth == wordBits)



    val s0_valid = io.req.fire()
    val s0_vaddr = io.req.bits.addr

    val s1_valid = RegNext(s0_valid)
    val s1_tag_hit = Wire(Vec(nWays, Bool()))
    val s1_hit = s1_tag_hit.reduce(_||_)
    val s2_valid = RegNext(s1_valid && !io.s1_kill)
    val s2_hit = RegNext(s1_hit)


    val invalidated = Reg(Bool())
    val refill_valid = RegInit(false.B)
    val refill_fire = tl_out.a.fire()
    val s2_miss = s2_valid && !s2_hit && !RegNext(refill_valid)
    val refill_paddr = RegEnable(io.s1_paddr, s1_valid && !(refill_valid || s2_miss))
    val refill_tag = refill_paddr(tagBits+untagBits-1,untagBits)
    val refill_idx = refill_paddr(untagBits-1,blockOffBits)
    val refill_one_beat = tl_out.d.fire() && edge_out.hasData(tl_out.d.bits)

    io.req.ready := !refill_one_beat

    val (_, _, d_done, refill_cnt) = edge_out.count(tl_out.d)
    val refill_done = refill_one_beat && d_done
    tl_out.d.ready := true.B
    require (edge_out.manager.minLatency > 0)

    val repl_way = if (isDM) 0.U else LFSR(16, refill_fire)(log2Ceil(nWays)-1,0)

    val tag_array = SyncReadMem(nSets, Vec(nWays, UInt(tagBits.W)))
    val tag_rdata = tag_array.read(s0_vaddr(untagBits-1, blockOffBits), !refill_done && s0_valid)
    when (refill_done) {
      tag_array.write(refill_idx, VecInit(Seq.fill(nWays)(refill_tag)), Seq.tabulate(nWays)(repl_way === _.U))
    }

    val vb_array = RegInit(0.U((nSets*nWays).W))
    when (refill_one_beat) {
      vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), refill_done && !invalidated)
    }

    when (io.invalidate) {
      vb_array := 0.U
      invalidated := true.B
    }

    val s2_dout   = Wire(Vec(nWays, UInt(wordBits.W)))
    val s1_bankid = Wire(Bool())

    for (i <- 0 until nWays) {
      val s1_idx = io.s1_paddr(untagBits-1,blockOffBits)
      val s1_tag = io.s1_paddr(tagBits+untagBits-1,untagBits)
      val s1_vb = vb_array(Cat(i.U, s1_idx))
      val tag = tag_rdata(i)
      s1_tag_hit(i) := s1_vb && tag === s1_tag
    }
    assert(PopCount(s1_tag_hit) <= 1.U || !s1_valid)

    val ramDepth = if (refillsToOneBank && nBanks == 2) {
      nSets * refillCycles / 2
    } else {
      nSets * refillCycles
    }

    if (nBanks == 1) {
      // Use unbanked icache for narrow accesses.
      s1_bankid := 0.U
      for ((dataArray, i) <- dataArrays.map(_._1) zipWithIndex) {
        def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
        val s0_ren = s0_valid

        val wen = (refill_one_beat && !invalidated) && repl_way === i.U

        val mem_idx = Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
                      row(s0_vaddr))
        when (wen) {
          dataArray.write(mem_idx, tl_out.d.bits.data)
        }
        if (enableICacheDelay)
          s2_dout(i) := dataArray.read(RegNext(mem_idx), RegNext(!wen && s0_ren))
        else
          s2_dout(i) := RegNext(dataArray.read(mem_idx, !wen && s0_ren))
      }
    } else {
      // Use two banks, interleaved.
      val dataArraysB0 = dataArrays.map(_._1).take(nWays)
      val dataArraysB1 = dataArrays.map(_._1).drop(nWays)
      require (nBanks == 2)

      // Bank0 row's id wraps around if Bank1 is the starting bank.
      def b0Row(addr: UInt) =
        if (refillsToOneBank) {
          addr(untagBits-1, blockOffBits-log2Ceil(refillCycles)+1) + bank(addr)
        } else {
          addr(untagBits-1, blockOffBits-log2Ceil(refillCycles)) + bank(addr)
        }
      // Bank1 row's id stays the same regardless of which Bank has the fetch address.
      def b1Row(addr: UInt) =
        if (refillsToOneBank) {
          addr(untagBits-1, blockOffBits-log2Ceil(refillCycles)+1)
        } else {
          addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
        }

      s1_bankid := RegNext(bank(s0_vaddr))

      for (i <- 0 until nWays) {
        val s0_ren = s0_valid
        val wen = (refill_one_beat && !invalidated)&& repl_way === i.U

        var mem_idx0: UInt = null
        var mem_idx1: UInt = null

        if (refillsToOneBank) {
          // write a refill beat across only one beat.
          mem_idx0 =
            Mux(refill_one_beat, (refill_idx << (log2Ceil(refillCycles)-1)) | (refill_cnt >> 1.U),
            b0Row(s0_vaddr))
          mem_idx1 =
            Mux(refill_one_beat, (refill_idx << (log2Ceil(refillCycles)-1)) | (refill_cnt >> 1.U),
            b1Row(s0_vaddr))

          when (wen && refill_cnt(0) === 0.U) {
            dataArraysB0(i).write(mem_idx0, tl_out.d.bits.data)
          }
          when (wen && refill_cnt(0) === 1.U) {
            dataArraysB1(i).write(mem_idx1, tl_out.d.bits.data)
          }
        } else {
          // write a refill beat across both banks.
          mem_idx0 =
            Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
            b0Row(s0_vaddr))
          mem_idx1 =
            Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
            b1Row(s0_vaddr))

          when (wen) {
            val data = tl_out.d.bits.data
            dataArraysB0(i).write(mem_idx0, data(wordBits/2-1, 0))
            dataArraysB1(i).write(mem_idx1, data(wordBits-1, wordBits/2))
          }
        }
        if (enableICacheDelay) {
          s2_dout(i) := Cat(dataArraysB1(i).read(RegNext(mem_idx1), RegNext(!wen && s0_ren)),
                            dataArraysB0(i).read(RegNext(mem_idx0), RegNext(!wen && s0_ren)))
        } else {
          s2_dout(i) := RegNext(Cat(dataArraysB1(i).read(mem_idx1, !wen && s0_ren),
                                    dataArraysB0(i).read(mem_idx0, !wen && s0_ren)))
        }
      }
    }
    val s2_tag_hit = RegNext(s1_tag_hit)
    val s2_hit_way = OHToUInt(s2_tag_hit)
    val s2_bankid = RegNext(s1_bankid)
    val s2_way_mux = Mux1H(s2_tag_hit, s2_dout)

    val s2_unbanked_data = s2_way_mux
    val sz = s2_way_mux.getWidth
    val s2_bank0_data = s2_way_mux(sz/2-1,0)
    val s2_bank1_data = s2_way_mux(sz-1,sz/2)

    val s2_data =
      if (nBanks == 2) {
        Mux(s2_bankid,
          Cat(s2_bank0_data, s2_bank1_data),
          Cat(s2_bank1_data, s2_bank0_data))
      } else {
        s2_unbanked_data
      }

    io.resp.bits.data := s2_data
    io.resp.valid := s2_valid && s2_hit

    tl_out.a.valid := s2_miss && !refill_valid && !io.s2_kill
    tl_out.a.bits := edge_out.Get(
      fromSource = 0.U,
      toAddress = (refill_paddr >> blockOffBits) << blockOffBits,
      lgSize = lgCacheBlockBytes.U)._2
    tl_out.b.ready := true.B
    tl_out.c.valid := false.B
    tl_out.e.valid := false.B

    io.perf.acquire := tl_out.a.fire()

    when (!refill_valid) { invalidated := false.B }
    when (refill_fire) { refill_valid := true.B }
    when (refill_done) { refill_valid := false.B }
  }


  override def toString: String = BoomCoreStringPrefix(
    "==L1-ICache==",
    "Fetch bytes   : " + cacheParams.fetchBytes,
    "Block bytes   : " + (1 << blockOffBits),
    "Row bytes     : " + rowBytes,
    "Word bits     : " + wordBits,
    "Sets          : " + nSets,
    "Ways          : " + nWays,
    "Refill cycles : " + refillCycles,
    "RAMs          : (" +  wordBits/nBanks + " x " + nSets*refillCycles + ") using " + nBanks + " banks",
    "" + (if (nBanks == 2) "Dual-banked" else "Single-banked"),
    "I-TLB ways    : " + cacheParams.nTLBWays + "\n")
}


