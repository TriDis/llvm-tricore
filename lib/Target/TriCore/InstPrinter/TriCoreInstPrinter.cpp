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

#define DEBUG_TYPE "asm-printer"

#include "TriCoreGenAsmWriter.inc"

void TriCoreInstPrinter::printRegName(raw_ostream &OS, unsigned RegNo) const {
  OS << "%" << StringRef(getRegisterName(RegNo)).lower();
}

void TriCoreInstPrinter::printInst(const MCInst *MI, raw_ostream &O,
                               StringRef Annot, const MCSubtargetInfo &STI) {
  printInstruction(MI, O);
  printAnnotation(O, Annot);
}

static void printExpr(const MCExpr *Expr, const MCAsmInfo *MAI,
                      raw_ostream &OS) {
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

  MCSymbolRefExpr::VariantKind Kind = SRE->getKind();

  switch (Kind) {
    default:                                 llvm_unreachable("Invalid kind!");
    case MCSymbolRefExpr::VK_None:           break;
    case MCSymbolRefExpr::VK_TRICORE_HI_OFFSET:    OS << "hi:";     break;
    case MCSymbolRefExpr::VK_TRICORE_LO_OFFSET:    OS << "lo:";     break;
  }

  SRE->getSymbol().print(OS, MAI);

  if (Offset) {
    if (Offset > 0)
      OS << '+';
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

void TriCoreInstPrinter::printDoubleAddrRegs(const MCInst *MI, unsigned OpNo,
                                             raw_ostream &O) {
  const MCOperand &Addr = MI->getOperand(OpNo);

  if (Addr.isReg()) {
    O << "[";
    printRegName(O, Addr.getReg());
    O << "/";
    printRegName(O, Addr.getReg()+1);
    O << "]";
  }
}


//===----------------------------------------------------------------------===//
// PrintSExtImm<unsigned bits>
//===----------------------------------------------------------------------===//
template <unsigned bits>
void TriCoreInstPrinter::printSExtImm(const MCInst *MI, unsigned OpNo,
                                       raw_ostream &O) {
  if (MI->getOperand(OpNo).isImm()) {
    int64_t Value = MI->getOperand(OpNo).getImm();
    Value = SignExtend32<bits>(Value);
    assert(isInt<bits>(Value) && "Invalid simm argument");
    O << Value;
  }
  else
    printOperand(MI, OpNo, O);
}

template <unsigned bits>
void TriCoreInstPrinter::printZExtImm(const MCInst *MI, int OpNo,
                                       raw_ostream &O) {
  if (MI->getOperand(OpNo).isImm()) {
    unsigned int Value = MI->getOperand(OpNo).getImm();
    assert(Value <= ((unsigned int)pow(2,bits) -1 )  && "Invalid uimm argument!");
    O << (unsigned int)Value;
  }
  else
    printOperand(MI, OpNo, O);
}

// Print a 'abs' operand which is an addressing mode
// Absolute
//void TriCoreInstPrinter::printAddrABS(const MCInst *MI, unsigned OpNum,
                                         //raw_ostream &O) {

  //const MCOperand &Op = MI->getOperand(OpNo);
  //if (Op.isImm())
    //O << Op.getImm();
  //else {
    //assert(Op.isExpr() && "unknown absolute address");
    //Op.getExpr()->print(O, &MAI);
  //}
//}

// Print a 'bo' operand which is an addressing mode
// Base+Offset
void TriCoreInstPrinter::printAddrBO(const MCInst *MI, unsigned OpNum,
                                         raw_ostream &O) {

  const MCOperand &Base = MI->getOperand(OpNum);
  const MCOperand &Offset = MI->getOperand(OpNum+1);

  unsigned Opcode = MI->getOpcode();

  switch (Opcode) {
    default:
      // Print register base field
      if (Base.isReg())
          O << "[%" << StringRef(getRegisterName(Base.getReg())).lower() << "]";

      if (Offset.isExpr())
        Offset.getExpr()->print(O, &MAI);
      else {
        assert(Offset.isImm() && "Expected immediate in displacement field");
        O << " " << Offset.getImm();
      }
      break;
}

// Print a 'preincbo' operand which is an addressing mode
// Pre-increment Base+Offset
void TriCoreInstPrinter::printAddrPreIncBO(const MCInst *MI, unsigned OpNum,
                                         raw_ostream &O) {

  const MCOperand &Base = MI->getOperand(OpNum);
  const MCOperand &Offset = MI->getOperand(OpNum+1);

  // Print register base field
  if (Base.isReg())
      O << "[+%" << StringRef(getRegisterName(Base.getReg())).lower() << "]";

  if (Offset.isExpr())
    Offset.getExpr()->print(O, &MAI);
  else {
    assert(Offset.isImm() && "Expected immediate in displacement field");
    O << " " << Offset.getImm();
  }
}

// Print a 'postincbo' operand which is an addressing mode
// Post-increment Base+Offset
void TriCoreInstPrinter::printAddrPostIncBO(const MCInst *MI, unsigned OpNum,
                                         raw_ostream &O) {

  const MCOperand &Base = MI->getOperand(OpNum);
  const MCOperand &Offset = MI->getOperand(OpNum+1);

  // Print register base field
  if (Base.isReg())
      O << "[%" << StringRef(getRegisterName(Base.getReg())).lower() << "+]";

  if (Offset.isExpr())
    Offset.getExpr()->print(O, &MAI);
  else {
    assert(Offset.isImm() && "Expected immediate in displacement field");
    O << " " << Offset.getImm();
  }
}

// Print a 'circbo' operand which is an addressing mode
// Circular Base+Offset
void TriCoreInstPrinter::printAddrCircBO(const MCInst *MI, unsigned OpNum,
                                         raw_ostream &O) {

  const MCOperand &Base = MI->getOperand(OpNum);
  const MCOperand &Offset = MI->getOperand(OpNum+1);

  // Print register base field
  if (Base.isReg())
      O << "[%" << StringRef(getRegisterName(Base.getReg())).lower() << "/%" << StringRef(getRegisterName(Base.getReg()+1)).lower() << "+c]";

  if (Offset.isExpr())
    Offset.getExpr()->print(O, &MAI);
  else {
    assert(Offset.isImm() && "Expected immediate in displacement field");
    O << " " << Offset.getImm();
  }
}

// Print a 'bitrevbo' operand which is an addressing mode
// Bit-Reverse Base+Offset
void TriCoreInstPrinter::printAddrBitRevBO(const MCInst *MI, unsigned OpNum,
                                         raw_ostream &O) {

  const MCOperand &Base = MI->getOperand(OpNum);
  const MCOperand &Offset = MI->getOperand(OpNum+1);

  // Print register base field
  if (Base.isReg())
      O << "[%" << StringRef(getRegisterName(Base.getReg())).lower() << "/%" << StringRef(getRegisterName(Base.getReg()+1)).lower() << "+r]";
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
  printExpr(Op.getExpr(), &MAI, O);
}
