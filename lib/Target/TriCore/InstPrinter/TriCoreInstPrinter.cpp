//==-- TriCoreInstPrinter.cpp - Convert TriCore MCInst to assembly syntax -===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This class prints an TriCore MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "asm-printer"
#include "TriCoreInstPrinter.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "../TriCore.h"
using namespace llvm;

#include "TriCoreGenAsmWriter.inc"

void TriCoreInstPrinter::printRegName(raw_ostream &OS, unsigned RegNo) const {
  OS << "%" <<StringRef(getRegisterName(RegNo)).lower();

}

void TriCoreInstPrinter::printInst(const MCInst *MI, raw_ostream &O,
                               StringRef Annot, const MCSubtargetInfo &STI) {
  printInstruction(MI, O);
  printAnnotation(O, Annot);
}

static void printExpr(const MCExpr *Expr, raw_ostream &OS) {
  int Offset = 0;
  const MCSymbolRefExpr *SRE;

  if (const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(Expr)) {
    SRE = dyn_cast<MCSymbolRefExpr>(BE->getLHS());
    const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(BE->getRHS());
    assert(SRE && CE && "Binary expression must be sym+const.");
    Offset = CE->getValue();
  } else {
    SRE = dyn_cast<MCSymbolRefExpr>(Expr);
    assert(SRE && "Unexpected MCExpr type.");
  }
  const MCSymbolRefExpr::VariantKind Kind = SRE->getKind();
//  assert(Kind == MCSymbolRefExpr::VK_None ||
//         Kind == MCSymbolRefExpr::VK_TRICORE_LO ||
//         Kind == MCSymbolRefExpr::VK_TRICORE_HI);
//

  if (Kind ==  MCSymbolRefExpr::VK_TRICORE_HI_OFFSET)
  	OS << "hi:";
  else if (Kind ==  MCSymbolRefExpr::VK_TRICORE_LO_OFFSET)
  	OS << "lo:";
  OS << SRE->getSymbol();
  if (Offset) {
    if (Offset > 0) {
      OS << '+';
    }
    OS << Offset;
  }
}

void TriCoreInstPrinter::printPCRelImmOperand(const MCInst *MI, unsigned OpNo,
                                             raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isImm())
    O << Op.getImm();
  else {
    assert(Op.isExpr() && "unknown pcrel immediate operand");
    Op.getExpr()->print(O, &MAI);
  }
}


//===----------------------------------------------------------------------===//
// PrintSExtImm<unsigned bits>
//===----------------------------------------------------------------------===//
template <unsigned bits>
void TriCoreInstPrinter::printSExtImm(const MCInst *MI, unsigned OpNo,
                                       raw_ostream &O) {
  int64_t Value = MI->getOperand(OpNo).getImm();
 Value = SignExtend32<bits>(Value);
  //outs()<< "Value: "<< Value <<"\n";
  assert(isInt<bits>(Value) && "Invalid simm argument");

  O << Value;
}

template <unsigned bits>
void TriCoreInstPrinter::printZExtImm(const MCInst *MI, int OpNo,
                                       raw_ostream &O) {
  unsigned int Value = MI->getOperand(OpNo).getImm();
//  unsigned int maxval = pow((uint)2,(uint)bits);
  //outs()<< "Power value: " << maxval <<"\n";
  assert(Value <= ((unsigned int)pow(2,bits) -1 )  && "Invalid uimm argument!");
  //Value =  (unsigned char)(Value);
  O << (unsigned int)Value;
}


// Print a 'memsrc' operand which is a (Register, Offset) pair.
void TriCoreInstPrinter::printAddrModeMemSrc(const MCInst *MI, unsigned OpNum,
                                         raw_ostream &O) {

	const MCOperand &Base = MI->getOperand(OpNum);
	const MCOperand &Disp = MI->getOperand(OpNum+1);

	// Print register base field
	if (Base.getReg())
			O << "[%" << StringRef(getRegisterName(Base.getReg())).lower() << ']';

	if (Disp.isExpr())
		Disp.getExpr()->print(O, &MAI);
	else {
		assert(Disp.isImm() && "Expected immediate in displacement field");
		O << " " << Disp.getImm();
	}

}

void TriCoreInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {

	const MCOperand &Op = MI->getOperand(OpNo);

  if (Op.isReg()) {
    printRegName(O, Op.getReg());
    return;
  }

  if (Op.isImm()) {
    O << Op.getImm();
    return;
  }

  assert(Op.isExpr() && "unknown operand kind in printOperand");
  printExpr(Op.getExpr(), O);
}

void TriCoreInstPrinter::printCCOperand(const MCInst *MI, unsigned OpNo,
                                       raw_ostream &O) {
  unsigned CC = MI->getOperand(OpNo).getImm();

  switch (CC) {
  default:
   llvm_unreachable("Unsupported CC code");
  case 0:
   O << "eq";
   break;
  case 1:
   O << "ne";
   break;
  case 3:
   O << "lt";
   break;
  case 2:
   O << "ge";
   break;
  }
}
