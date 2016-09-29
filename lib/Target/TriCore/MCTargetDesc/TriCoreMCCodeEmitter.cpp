//== TriCore/TriCoreMCCodeEmitter.cpp - Convert TriCore code to machine code =//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the TriCoreMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "mccodeemitter"
#include "MCTargetDesc/TriCoreMCTargetDesc.h"
#include "MCTargetDesc/TriCoreFixupKinds.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

STATISTIC(MCNumEmitted, "Number of MC instructions emitted.");

namespace {
class TriCoreMCCodeEmitter : public MCCodeEmitter {
  TriCoreMCCodeEmitter(const TriCoreMCCodeEmitter &) = delete;
  void operator=(const TriCoreMCCodeEmitter &) = delete;
  const MCInstrInfo &MCII;
  const MCContext &CTX;

public:
  TriCoreMCCodeEmitter(const MCInstrInfo &mcii, MCContext &ctx)
      : MCII(mcii), CTX(ctx) {}

  ~TriCoreMCCodeEmitter() {}

  // getBinaryCodeForInstr - TableGen'erated function for getting the
  // binary encoding for an instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  /// getMachineOpValue - Return binary encoding of operand. If the machine
  /// operand requires relocation, record the relocation and return zero.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getMemSrcValue(const MCInst &MI, unsigned OpIdx,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;

  unsigned encodeCallTarget(const MCInst &MI, unsigned OpNo,
                            SmallVectorImpl<MCFixup> &Fixups,
                            const MCSubtargetInfo &STI) const;

  void EmitByte(unsigned char C, raw_ostream &OS) const { OS << (char)C; }

  void EmitConstant(uint64_t Val, unsigned Size, raw_ostream &OS) const {
    // Output the constant in little endian byte order.
    for (unsigned i = 0; i != Size; ++i) {
      EmitByte((Val >> (i * 8)) & 0xff, OS);
    }
  }
  
  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;
};

} // end anonymous namespace

MCCodeEmitter *llvm::createTriCoreMCCodeEmitter(const MCInstrInfo &MCII,
                                            const MCRegisterInfo &MRI,
                                            MCContext &Ctx) {
  return new TriCoreMCCodeEmitter(MCII, Ctx);
}

unsigned TriCoreMCCodeEmitter::encodeCallTarget(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    MCFixupKind FixupKind = static_cast<MCFixupKind>(TriCore::fixup_call);
    Fixups.push_back(MCFixup::create(0, MO.getExpr(), FixupKind, MI.getLoc()));
    return 0;
  }

  assert(MO.isImm());

  auto target = MO.getImm();
  return target;
}

/// getMachineOpValue - Return binary encoding of operand. If the machine
/// operand requires relocation, record the relocation and return zero.
unsigned TriCoreMCCodeEmitter::getMachineOpValue(const MCInst &MI,
                                             const MCOperand &MO,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI) const {
  if (MO.isReg()) {
    switch(MO.getReg()) {
      default:
        return CTX.getRegisterInfo()->getEncodingValue(MO.getReg());
      case TriCore::E0:  return 0;
      case TriCore::E2:  return 2;
      case TriCore::E4:  return 4;
      case TriCore::E6:  return 6;
      case TriCore::E8:  return 8;
      case TriCore::E10: return 10;
      case TriCore::E12: return 12;
      case TriCore::E14: return 14;
    }
  }

  if (MO.isImm()) {
    return static_cast<unsigned>(MO.getImm());
  }

  assert(MO.isExpr() && "unknown operand kind in printOperand");

  const MCExpr *Expr = MO.getExpr();
  MCExpr::ExprKind Kind = Expr->getKind();

  if (Kind == MCExpr::Binary) {
    Expr = static_cast<const MCBinaryExpr*>(Expr)->getLHS();
    Kind = Expr->getKind();
  }

  assert (Kind == MCExpr::SymbolRef);

  cast<MCSymbolRefExpr>(Expr)->printVariantKind(outs());
  unsigned FixupKind;
  switch (cast<MCSymbolRefExpr>(Expr)->getKind()) {
  default:
    llvm_unreachable("Unknown fixup kind!");
  case MCSymbolRefExpr::VK_TRICORE_LO_OFFSET:
  case MCSymbolRefExpr::VK_TRICORE_HI_OFFSET:
    return 0;
  case MCSymbolRefExpr::VK_TRICORE_LO: {
    FixupKind = TriCore::fixup_leg_mov_lo16_pcrel;
    break;
  }
  case MCSymbolRefExpr::VK_TRICORE_HI: {
    FixupKind = TriCore::fixup_leg_mov_hi16_pcrel;
    break;
  }
  }

  Fixups.push_back(MCFixup::create(0, MO.getExpr(), MCFixupKind(FixupKind)));
  return 0;
}

unsigned TriCoreMCCodeEmitter::getMemSrcValue(const MCInst &MI, unsigned OpIdx,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  const MCOperand &RegMO = MI.getOperand(OpIdx);
  const MCOperand &ImmMO = MI.getOperand(OpIdx + 1);
  //assert(ImmMO.getImm() >= 0);
  unsigned Reg = getMachineOpValue(MI, RegMO, Fixups, STI);
  int32_t offset = Reg | (ImmMO.getImm() << 4);
  return offset;
}

void TriCoreMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  if (Desc.getSize() != 2 && Desc.getSize() != 4) {
    llvm_unreachable("Unexpected instruction size!");
  }

  uint32_t Binary = getBinaryCodeForInstr(MI, Fixups, STI);

  if (Desc.getSize() == 4) {
      EmitConstant(Binary & 0xffff, 2, OS);
      EmitConstant(Binary >> 16, 2, OS);
    }
  else
      EmitConstant(Binary, 2, OS);

  ++MCNumEmitted;
}

#include "TriCoreGenMCCodeEmitter.inc"
