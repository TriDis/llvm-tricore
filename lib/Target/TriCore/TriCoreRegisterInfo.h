//=-- TriCoreRegisterInfo.h - TriCore Register Information Impl ---*- C++ -*-=//
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

#ifndef LLVM_LIB_TARGET_TRICORE_TRICOREREGISTERINFO_H
#define LLVM_LIB_TARGET_TRICORE_TRICOREREGISTERINFO_H

#include "llvm/Target/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "TriCoreGenRegisterInfo.inc"

namespace llvm {

class TargetInstrInfo;

struct TriCoreRegisterInfo : public TriCoreGenRegisterInfo {
public:
  TriCoreRegisterInfo();

  /// Code Generation virtual methods...

  const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  //const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       //CallingConv::ID CC) const override;

  bool requiresRegisterScavenging(const MachineFunction &MF) const override;

  bool trackLivenessAfterRegAlloc(const MachineFunction &MF) const override;

  bool useFPForScavengingIndex(const MachineFunction &MF) const override;

  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;

  // Debug information queries.
  unsigned getFrameRegister(const MachineFunction &MF) const override;
};

} // end namespace llvm

#endif
