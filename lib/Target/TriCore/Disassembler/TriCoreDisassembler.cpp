//===- TriCoreDisassembler.cpp - Disassembler for TriCore -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///
/// \file
/// \brief This file is part of the TriCore Disassembler.
///
//===----------------------------------------------------------------------===//

#include "TriCore.h"
#include "TriCoreRegisterInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "tricore-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {

/// \brief A disassembler class for TriCore.
class TriCoreDisassembler : public MCDisassembler {
public:
  TriCoreDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx) :
    MCDisassembler(STI, Ctx) {}

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &VStream,
                              raw_ostream &CStream) const override;
};
}

static bool readInstruction16(ArrayRef<uint8_t> Bytes, uint64_t Address,
                              uint64_t &Size, uint16_t &Insn) {
  // We want to read exactly 2 Bytes of data.
  if (Bytes.size() < 2) {
    Size = 0;
    return false;
  }
  // Encoded as a little-endian 16-bit word in the stream.
  Insn = (Bytes[0] << 0) | (Bytes[1] << 8);
  return true;
}

static bool readInstruction32(ArrayRef<uint8_t> Bytes, uint64_t Address,
                              uint64_t &Size, uint32_t &Insn) {
  // We want to read exactly 4 Bytes of data.
  if (Bytes.size() < 4) {
    Size = 0;
    return false;
  }
  // Encoded as a little-endian 32-bit word in the stream.
  Insn =
      (Bytes[0] << 0) | (Bytes[1] << 8) | (Bytes[2] << 16) | (Bytes[3] << 24);
  return true;
}

static unsigned getReg(const void *D, unsigned RC, unsigned RegNo) {
  const TriCoreDisassembler *Dis = static_cast<const TriCoreDisassembler*>(D);
  const MCRegisterInfo *RegInfo = Dis->getContext().getRegisterInfo();
  return *(RegInfo->getRegClass(RC).begin() + RegNo);
}

static DecodeStatus DecodeDataRegsRegisterClass(MCInst &Inst,
                                              unsigned RegNo,
                                              uint64_t Address,
                                              const void *Decoder);

static DecodeStatus DecodeAddrRegsRegisterClass(MCInst &Inst,
                                             unsigned RegNo,
                                             uint64_t Address,
                                             const void *Decoder);

static DecodeStatus DecodeExtRegsRegisterClass(MCInst &Inst,
                                             unsigned RegNo,
                                             uint64_t Address,
                                             const void *Decoder);

static DecodeStatus DecodeSBInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSBCInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSBRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSBRNInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSCInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSLRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSLROInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSRCInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSROInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSRRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSRRSInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSSRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSSROInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeABSInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeABSBInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeBInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeBITInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeBOInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeBOLInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeBRCInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeBRNInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeBRRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRCInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRCPWInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRCRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRCRRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRCRWInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRLCInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRR1Instruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRR2Instruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRRPWInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRRRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRRR1Instruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRRR2Instruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRRRRInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeRRRWInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeSYSInstruction(MCInst &Inst,
                                               unsigned Insn,
                                               uint64_t Address,
                                               const void *Decoder);


#include "TriCoreGenDisassemblerTables.inc"

static DecodeStatus DecodeDataRegsRegisterClass(MCInst &Inst,
                                              unsigned RegNo,
                                              uint64_t Address,
                                              const void *Decoder)
{
  if (RegNo > 15)
    return MCDisassembler::Fail;
  unsigned Reg = getReg(Decoder, TriCore::DataRegsRegClassID, RegNo);
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeAddrRegsRegisterClass(MCInst &Inst,
                                             unsigned RegNo,
                                             uint64_t Address,
                                             const void *Decoder)
{
  if (RegNo > 15)
    return MCDisassembler::Fail;
  unsigned Reg = getReg(Decoder, TriCore::AddrRegsRegClassID, RegNo);
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeExtRegsRegisterClass(MCInst &Inst,
                                             unsigned RegNo,
                                             uint64_t Address,
                                             const void *Decoder)
{
  if (RegNo > 15)
    return MCDisassembler::Fail;
  unsigned Reg = getReg(Decoder, TriCore::ExtRegsRegClassID, RegNo);
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSBInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned disp8 = fieldFromInstruction(Insn, 8, 8);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode disp8.
  Inst.addOperand(MCOperand::createImm(disp8));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSBCInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned const4 = fieldFromInstruction(Insn, 12, 4);
  unsigned disp4 = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode const4.
  Inst.addOperand(MCOperand::createImm(const4));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSBRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned disp4 = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode s2.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode disp4.
  Inst.addOperand(MCOperand::createImm(disp4));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSBRNInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned n = fieldFromInstruction(Insn, 12, 4);
  unsigned disp4 = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode n.
  Inst.addOperand(MCOperand::createImm(n));

  // Decode disp4.
  Inst.addOperand(MCOperand::createImm(disp4));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSCInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned const8 = fieldFromInstruction(Insn, 8, 8);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode const8.
  Inst.addOperand(MCOperand::createImm(const8));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSLRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned d = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode d.
  status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSLROInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned off4 = fieldFromInstruction(Insn, 12, 4);
  unsigned d = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode d.
  status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode off4.
  Inst.addOperand(MCOperand::createImm(off4));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned s1_d = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode s1/d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, s1_d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSRCInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned const4 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1_d = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode s1/d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, s1_d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode const4.
  Inst.addOperand(MCOperand::createImm(const4));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSROInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned off4 = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode s2.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode off4.
  Inst.addOperand(MCOperand::createImm(off4));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSRRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1_d = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode s1/d.
  switch (Inst.getOpcode()) {
    case TriCore::MOVAAsrr:
      status = DecodeAddrRegsRegisterClass(Inst, s1_d, Address, Decoder);
      break;
    default:
      status = DecodeDataRegsRegisterClass(Inst, s1_d, Address, Decoder);
      break;
  }
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  switch (Inst.getOpcode()) {
    case TriCore::MOVAAsrr:
      status = DecodeAddrRegsRegisterClass(Inst, s2, Address, Decoder);
      break;
    default:
      status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
      break;
  }
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSRRSInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1_d = fieldFromInstruction(Insn, 8, 4);
  unsigned n = fieldFromInstruction(Insn, 6, 2);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode s1/d.
  status = DecodeDataRegsRegisterClass(Inst, s1_d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode n.
  Inst.addOperand(MCOperand::createImm(n));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSSRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSSROInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned off4 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(is32Bit) // This instruction is 16-bit
    return MCDisassembler::Fail;

  // Decode off4.
  Inst.addOperand(MCOperand::createImm(off4));

  // Decode s1.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeABSInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned off18_0 = fieldFromInstruction(Insn, 16, 6);
  unsigned off18_1 = fieldFromInstruction(Insn, 28, 4);
  unsigned off18_2 = fieldFromInstruction(Insn, 22, 4);
  unsigned off18_3 = fieldFromInstruction(Insn, 12, 4);
  unsigned off18 = (off18_0 << 0) | (off18_1 << 6) |
    (off18_2 << 10) | (off18_3 << 14);

  unsigned s1_d = fieldFromInstruction(Insn, 8, 4);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode s1/d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, s1_d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode off18.
  Inst.addOperand(MCOperand::createImm(off18));

  return MCDisassembler::Success;
}


static DecodeStatus
DecodeABSBInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned off18_0 = fieldFromInstruction(Insn, 16, 6);
  unsigned off18_1 = fieldFromInstruction(Insn, 28, 4);
  unsigned off18_2 = fieldFromInstruction(Insn, 22, 4);
  unsigned off18_3 = fieldFromInstruction(Insn, 12, 4);
  unsigned off18 = (off18_0 << 0) | (off18_1 << 6) |
    (off18_2 << 10) | (off18_3 << 14);

  unsigned b = fieldFromInstruction(Insn, 11, 1);
  unsigned bpos3 = fieldFromInstruction(Insn, 8, 3);
  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode off18.
  Inst.addOperand(MCOperand::createImm(off18));

  // Decode bpos3.
  Inst.addOperand(MCOperand::createImm(bpos3));

  // Decode b.
  Inst.addOperand(MCOperand::createImm(b));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeBInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned disp24_0 = fieldFromInstruction(Insn, 16, 16);
  unsigned disp24_1 = fieldFromInstruction(Insn, 8, 8);
  unsigned disp24 = (disp24_0 << 0) | (disp24_1 << 16);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode disp24.
  Inst.addOperand(MCOperand::createImm(disp24));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeBITInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned pos2 = fieldFromInstruction(Insn, 23, 5);
  unsigned pos1 = fieldFromInstruction(Insn, 16, 5);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode pos1.
  Inst.addOperand(MCOperand::createImm(pos1));

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode pos2.
  Inst.addOperand(MCOperand::createImm(pos2));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeBOInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned off10_0 = fieldFromInstruction(Insn, 16, 6);
  unsigned off10_1 = fieldFromInstruction(Insn, 28, 4);
  unsigned off10 = (off10_0 << 0) | (off10_1 << 6);

  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1_d = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode s1_d.
  switch (Inst.getOpcode()) {
    case TriCore::STAbo:
      status = DecodeAddrRegsRegisterClass(Inst, s1_d, Address, Decoder);
      break;
    case TriCore::LDDbo:
    case TriCore::STDbo:
      status = DecodeExtRegsRegisterClass(Inst, s1_d, Address, Decoder);
      break;
    default:
      status = DecodeDataRegsRegisterClass(Inst, s1_d, Address, Decoder);
      break;
  }
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeAddrRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode off10.
  Inst.addOperand(MCOperand::createImm(off10));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeBOLInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned off16_0 = fieldFromInstruction(Insn, 16, 6);
  unsigned off16_1 = fieldFromInstruction(Insn, 22, 6);
  unsigned off16_2 = fieldFromInstruction(Insn, 28, 4);
  unsigned off16 = (off16_0 << 0) | (off16_1 << 6) | (off16_2 << 6);

  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1_d = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode s1_d.
  status = DecodeDataRegsRegisterClass(Inst, s1_d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeAddrRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode off16.
  Inst.addOperand(MCOperand::createImm(off16));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeBRCInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned disp15 = fieldFromInstruction(Insn, 16, 15);
  unsigned const4 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode s1.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode const4.
  Inst.addOperand(MCOperand::createImm(const4));

  // Decode disp15.
  Inst.addOperand(MCOperand::createImm(disp15));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeBRNInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned disp15 = fieldFromInstruction(Insn, 16, 15);
  unsigned n_0 = fieldFromInstruction(Insn, 12, 4);
  unsigned n_1 = fieldFromInstruction(Insn, 7, 1);
  unsigned n = (n_0 << 0) | (n_1 << 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode s1.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode n.
  Inst.addOperand(MCOperand::createImm(n));

  // Decode disp15.
  Inst.addOperand(MCOperand::createImm(disp15));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeBRRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned disp15 = fieldFromInstruction(Insn, 16, 15);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode disp15.
  Inst.addOperand(MCOperand::createImm(disp15));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRCInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned const9 = fieldFromInstruction(Insn, 12, 9);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode const9.
  Inst.addOperand(MCOperand::createImm(const9));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRCPWInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned pos = fieldFromInstruction(Insn, 23, 5);
  unsigned width = fieldFromInstruction(Insn, 16, 5);
  unsigned const4 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  status = DecodeExtRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode const4.
  Inst.addOperand(MCOperand::createImm(const4));

  // Decode pos.
  Inst.addOperand(MCOperand::createImm(pos));

  // Decode width.
  Inst.addOperand(MCOperand::createImm(width));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRCRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned s3 = fieldFromInstruction(Insn, 24, 4);
  unsigned const9 = fieldFromInstruction(Insn, 12, 9);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s3.
  status = DecodeDataRegsRegisterClass(Inst, s3, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode const9.
  Inst.addOperand(MCOperand::createImm(const9));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRCRRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned s3 = fieldFromInstruction(Insn, 24, 4);
  unsigned const4 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode const4.
  Inst.addOperand(MCOperand::createImm(const4));

  // Decode s3.
  status = DecodeExtRegsRegisterClass(Inst, s3, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRCRWInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned s3 = fieldFromInstruction(Insn, 24, 4);
  unsigned width = fieldFromInstruction(Insn, 16, 5);
  unsigned const4 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode const4.
  Inst.addOperand(MCOperand::createImm(const4));

  // Decode s3.
  status = DecodeDataRegsRegisterClass(Inst, s3, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode width.
  Inst.addOperand(MCOperand::createImm(width));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRLCInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned const16 = fieldFromInstruction(Insn, 12, 16);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode const16.
  Inst.addOperand(MCOperand::createImm(const16));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {
  DecodeStatus status;
  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned n = fieldFromInstruction(Insn, 16, 2);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  switch (Inst.getOpcode()) {
    case TriCore::ADDArr:
    case TriCore::SUBArr:
    case TriCore::MOVArr:
    case TriCore::MOVAArr:
      status = DecodeAddrRegsRegisterClass(Inst, d, Address, Decoder);
      break;
    default:
      status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
      break;
  }
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  switch (Inst.getOpcode()) {
    case TriCore::ADDArr:
    case TriCore::SUBArr:
      status = DecodeAddrRegsRegisterClass(Inst, s1, Address, Decoder);
      break;
    default:
      status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
      break;
  }
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  switch (Inst.getOpcode()) {
    case TriCore::ADDArr:
    case TriCore::SUBArr:
    case TriCore::MOVDrr:
    case TriCore::MOVAArr:
      status = DecodeAddrRegsRegisterClass(Inst, s2, Address, Decoder);
      break;
    default:
      status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
      break;
  }
  if (status != MCDisassembler::Success)
    return status;

  // Decode n.
  Inst.addOperand(MCOperand::createImm(n));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRR1Instruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned n = fieldFromInstruction(Insn, 16, 2);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode n.
  Inst.addOperand(MCOperand::createImm(n));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRR2Instruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRRPWInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  DecodeStatus status;
  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned pos = fieldFromInstruction(Insn, 23, 5);
  unsigned width = fieldFromInstruction(Insn, 16, 5);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode pos.
  Inst.addOperand(MCOperand::createImm(pos));

  // Decode width.
  Inst.addOperand(MCOperand::createImm(width));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRRRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned s3 = fieldFromInstruction(Insn, 24, 4);
  unsigned n = fieldFromInstruction(Insn, 16, 2);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s3.
  status = DecodeDataRegsRegisterClass(Inst, s3, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode n.
  Inst.addOperand(MCOperand::createImm(n));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRRR1Instruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned s3 = fieldFromInstruction(Insn, 24, 4);
  unsigned n = fieldFromInstruction(Insn, 16, 2);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s3.
  status = DecodeDataRegsRegisterClass(Inst, s3, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode n.
  Inst.addOperand(MCOperand::createImm(n));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRRR2Instruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned s3 = fieldFromInstruction(Insn, 24, 4);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s3.
  status = DecodeDataRegsRegisterClass(Inst, s3, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRRRRInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned s3 = fieldFromInstruction(Insn, 24, 4);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s3.
  status = DecodeDataRegsRegisterClass(Inst, s3, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeRRRWInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned d = fieldFromInstruction(Insn, 28, 4);
  unsigned s3 = fieldFromInstruction(Insn, 24, 4);
  unsigned width = fieldFromInstruction(Insn, 16, 5);
  unsigned s2 = fieldFromInstruction(Insn, 12, 4);
  unsigned s1 = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s1.
  status = DecodeDataRegsRegisterClass(Inst, s1, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s2.
  status = DecodeDataRegsRegisterClass(Inst, s2, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode s3.
  status = DecodeDataRegsRegisterClass(Inst, s3, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  // Decode width.
  Inst.addOperand(MCOperand::createImm(width));

  return MCDisassembler::Success;
}

static DecodeStatus
DecodeSYSInstruction(MCInst &Inst, unsigned Insn, uint64_t Address,
                    const void *Decoder) {

  unsigned s1_d = fieldFromInstruction(Insn, 8, 4);

  unsigned is32Bit = fieldFromInstruction(Insn, 0, 1);

  if(!is32Bit) // This instruction is 32-bit
    return MCDisassembler::Fail;

  // Decode s1/d.
  DecodeStatus status = DecodeDataRegsRegisterClass(Inst, s1_d, Address, Decoder);
  if (status != MCDisassembler::Success)
    return status;

  return MCDisassembler::Success;
}

MCDisassembler::DecodeStatus TriCoreDisassembler::getInstruction(
    MCInst &instr, uint64_t &Size, ArrayRef<uint8_t> Bytes, uint64_t Address,
    raw_ostream &vStream, raw_ostream &cStream) const {
  uint16_t insn16;

  if (!readInstruction16(Bytes, Address, Size, insn16)) {
    return Fail;
  }

  // Calling the auto-generated decoder function.
  DecodeStatus Result = decodeInstruction(DecoderTable16, instr, insn16,
                                          Address, this, STI);
  if (Result != Fail) {
    Size = 2;
    return Result;
  }

  uint32_t insn32;

  if (!readInstruction32(Bytes, Address, Size, insn32)) {
    return Fail;
  }

  // Calling the auto-generated decoder function.
  Result = decodeInstruction(DecoderTable32, instr, insn32, Address, this, STI);
  if (Result != Fail) {
    Size = 4;
    return Result;
  }

  return Fail;
}

namespace llvm {
  extern Target TheTriCoreTarget;
}

static MCDisassembler *createTriCoreDisassembler(const Target &T,
                                               const MCSubtargetInfo &STI,
                                               MCContext &Ctx) {
  return new TriCoreDisassembler(STI, Ctx);
}

extern "C" void LLVMInitializeTriCoreDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(TheTriCoreTarget,
                                         createTriCoreDisassembler);
}
