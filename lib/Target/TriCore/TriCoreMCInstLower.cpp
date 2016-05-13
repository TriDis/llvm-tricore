//===-- TriCoreMCInstLower.cpp - Convert TriCore MachineInstr to MCInst ---===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///
/// \file
/// \brief This file contains code to lower TriCore MachineInstrs to their
/// corresponding MCInst records.
///
//===----------------------------------------------------------------------===//
#include "TriCoreMCInstLower.h"
#include "MCTargetDesc/TriCoreBaseInfo.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"

using namespace llvm;

TriCoreMCInstLower::TriCoreMCInstLower(class AsmPrinter &asmprinter)
    : Printer(asmprinter) {}

void TriCoreMCInstLower::Initialize(Mangler *M, MCContext *C) {
  Mang = M;
  Ctx = C;
}

MCOperand TriCoreMCInstLower::LowerSymbolOperand(const MachineOperand &MO,
                                             MachineOperandType MOTy,
                                             unsigned Offset) const {
  const MCSymbol *Symbol;

  switch (MOTy) {
  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    break;
  case MachineOperand::MO_GlobalAddress: {
  	//outs()<<"MachineOperand::MO_GlobalAddress\n";
    Symbol = Printer.getSymbol(MO.getGlobal());
    //Offset += MO.getOffset();
    //Offset = 0;
//    outs()<<"offset->MCInstr: " << Offset <<"\n";
//    const MCExpr *Expr = MCSymbolRefExpr::create(Symbol, *Ctx);
//    Expr = MCBinaryExpr::createAdd(Expr,MCConstantExpr::create(Offset, *Ctx),*Ctx);
//    return MCOperand::createExpr(Expr);
    break;
  }
  case MachineOperand::MO_BlockAddress:
    Symbol = Printer.GetBlockAddressSymbol(MO.getBlockAddress());
    Offset += MO.getOffset();
    break;
  case MachineOperand::MO_ExternalSymbol:
    Symbol = Printer.GetExternalSymbolSymbol(MO.getSymbolName());
    Offset += MO.getOffset();
    break;
  case MachineOperand::MO_JumpTableIndex:
    Symbol = Printer.GetJTISymbol(MO.getIndex());
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    Symbol = Printer.GetCPISymbol(MO.getIndex());
    Offset += MO.getOffset();
    break;
  default:
    llvm_unreachable("<unknown operand type>");
  }

  const unsigned Option = MO.getTargetFlags() & TriCoreII::MO_OPTION_MASK;
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_None;

  switch (Option) {
    default:
      break;
    case TriCoreII::MO_LO_OFFSET:
      Kind = MCSymbolRefExpr::VK_TRICORE_LO_OFFSET;
      break;
    case TriCoreII::MO_HI_OFFSET:
      Kind = MCSymbolRefExpr::VK_TRICORE_HI_OFFSET;
      break;
  }
  const MCSymbolRefExpr *MCSym = MCSymbolRefExpr::create(Symbol, Kind, *Ctx);

  if (!Offset) {
    return MCOperand::createExpr(MCSym);
  }

  // Assume offset is never negative.
  //assert(Offset > 0);
//  outs().changeColor(raw_ostream::BLUE,1)<<"MO_GlobalAddress->offset: "
//    																				<< Offset <<"\n";


  //const MCConstantExpr *OffsetExpr = MCConstantExpr::create(Offset, *Ctx);
  //const MCBinaryExpr *Add = MCBinaryExpr::createAdd(MCSym, OffsetExpr, *Ctx);
  //Add->dump();
  //MCOperand::createExpr(Add).dump();
  //outs().changeColor(raw_ostream::WHITE,0);
  return MCOperand::createExpr(MCSym);

  //return MCOperand::createExpr(Add);

}

MCOperand TriCoreMCInstLower::LowerOperand(const MachineOperand &MO,
                                       unsigned offset) const {
  MachineOperandType MOTy = MO.getType();

  switch (MOTy) {
  default:
    llvm_unreachable("unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit()) {
      break;
    }
    return MCOperand::createReg(MO.getReg());
  case MachineOperand::MO_Immediate:
    return MCOperand::createImm(MO.getImm() + offset);
  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_JumpTableIndex:
  case MachineOperand::MO_ConstantPoolIndex:
  case MachineOperand::MO_BlockAddress:
    return LowerSymbolOperand(MO, MOTy, offset);
  case MachineOperand::MO_RegisterMask:
    break;
  }

  return MCOperand();
}

void TriCoreMCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(MI->getOpcode());

  for (auto &MO : MI->operands()) {
    const MCOperand MCOp = LowerOperand(MO);

    if (MCOp.isValid()) {
      OutMI.addOperand(MCOp);
    }
  }
}
