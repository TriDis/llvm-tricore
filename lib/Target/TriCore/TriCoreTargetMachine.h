//===-- TriCoreTargetMachine.h - Define TargetMachine for Tricore-*- C++-*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the TriCore specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_TRICORE_TRICORETARGETMACHINE_H
#define LLVM_LIB_TARGET_TRICORE_TRICORETARGETMACHINE_H

#include "TriCore.h"
#include "TriCoreFrameLowering.h"
#include "TriCoreISelLowering.h"
#include "TriCoreInstrInfo.h"
#include "TriCoreSelectionDAGInfo.h"
#include "TriCoreSubtarget.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class TriCoreTargetMachine : public LLVMTargetMachine {
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  TriCoreSubtarget Subtarget;

public:
  TriCoreTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options, Reloc::Model RM,
                   CodeModel::Model CM, CodeGenOpt::Level OL);
  ~TriCoreTargetMachine() override;

  const TriCoreSubtarget *getSubtargetImpl() const { return &Subtarget; }
  const TriCoreSubtarget *getSubtargetImpl(const Function &) const override {
    return &Subtarget;
  }

  // Pass Pipeline Configuration
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return TLOF.get();
  }
};

} // end namespace llvm

#endif
