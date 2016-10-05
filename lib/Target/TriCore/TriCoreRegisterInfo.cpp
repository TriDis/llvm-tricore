//===-- TriCoreRegisterInfo.cpp - TriCore Register Information ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the TriCore implementation of the MRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "TriCoreRegisterInfo.h"
#include "TriCore.h"
#include "TriCoreFrameLowering.h"
#include "TriCoreInstrInfo.h"
#include "TriCoreMachineFunctionInfo.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

#define DEBUG_TYPE "tricore-reg-info"

#define GET_REGINFO_TARGET_DESC
#include "TriCoreGenRegisterInfo.inc"

TriCoreRegisterInfo::TriCoreRegisterInfo() 
  : TriCoreGenRegisterInfo(TriCore::A11) {
}

const MCPhysReg *
TriCoreRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {   
  static const MCPhysReg CalleeSavedRegs[] = { 0 };  
  return CalleeSavedRegs;
}

BitVector TriCoreRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  Reserved.set(TriCore::PC);
  Reserved.set(TriCore::PCXI);
  Reserved.set(TriCore::A10);
  Reserved.set(TriCore::A11);
  Reserved.set(TriCore::PSW);
  Reserved.set(TriCore::FCX);
  return Reserved;
}

//const uint32_t *
//TriCoreRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                          //CallingConv::ID CC) const {
  //return CSR_TriCore_RegMask;
//}

bool
TriCoreRegisterInfo::requiresRegisterScavenging(const MachineFunction &MF) const {
  return true;
}

bool
TriCoreRegisterInfo::trackLivenessAfterRegAlloc(const MachineFunction &MF) const {
  return true;
}

bool TriCoreRegisterInfo::useFPForScavengingIndex(const MachineFunction &MF) const {
  return false;
}

void
TriCoreRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                       int SPAdj, unsigned FIOperandNum,
                                       RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected");
  MachineInstr &MI = *II;
  const MachineFunction &MF = *MI.getParent()->getParent();
  DebugLoc dl = MI.getDebugLoc();
  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  MachineOperand &FIOp = MI.getOperand(FIOperandNum);
  unsigned FI = FIOp.getIndex();
  const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
  unsigned BasePtr = (TFI->hasFP(MF) ? TriCore::A14 : TriCore::A10);
  // Determine if we can eliminate the index from this kind of instruction.
  unsigned ImmOpIdx = 0;

  if (MI.getOpcode() == TriCore::ADDrc) {    
    int Offset = MFI->getObjectOffset(FI);    
    Offset = -Offset;
    const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
    MI.setDesc(TII.get(TriCore::MOV_Drr));
    MI.getOperand(FIOperandNum).ChangeToRegister(BasePtr, false);

    if (Offset == 0)
      return;

    // We need to materialize the offset via add instruction.
    unsigned DstReg = MI.getOperand(0).getReg();
    if (Offset < 0) {
      BuildMI(MBB, std::next(II), dl, TII.get(TriCore::ADDrc), DstReg).addReg(
          DstReg).addImm(Offset);
    } else
      BuildMI(MBB, std::next(II), dl, TII.get(TriCore::ADDrc), DstReg).addReg(
          DstReg).addImm(-Offset);

    return;
  }

  ImmOpIdx = FIOperandNum + 1;

  // FIXME: check the size of offset.
  MachineOperand &ImmOp = MI.getOperand(ImmOpIdx);
  int Offset = MFI->getObjectOffset(FI);
  FIOp.ChangeToRegister(BasePtr, false);
  ImmOp.setImm(Offset);
}


unsigned TriCoreRegisterInfo::getFrameRegister(const MachineFunction &MF) const {  
  const TriCoreFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? TriCore::A14 : TriCore::A10;  
}
