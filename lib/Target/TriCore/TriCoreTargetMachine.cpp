//===-- TriCoreTargetMachine.cpp - Define TargetMachine for TriCore -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

#include "TriCoreTargetMachine.h"
#include "TriCore.h"
#include "TriCoreFrameLowering.h"
#include "TriCoreInstrInfo.h"
#include "TriCoreISelLowering.h"
#include "TriCoreSelectionDAGInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

/*
*  @brief This function calculates the data layout of TriCore architecture.
*/
static std::string computeDataLayout() {
  return "e-m:e-p:32:32-i64:32-a:0:32-n32";
  //"e-m:e-p:32:32-i8:8:8-i16:16:16-i64:32-f32:32-f64:32-a:8:16-n32:64-S32";
}

/** 
@brief Creates a TriCore machine architecture.
*
* The data layout is described as below:
* 
*  Meaning of symbols:
*  Symbol    | Definition                                       | 
*  :-------- | :--:                                             |
*   e        | little endian                                    |
*   p:32:32  | pointer size: pointer ABI alignment              |
*   i1:8:32  | integer data type : size of type : ABI alignment |
*   f:64:64  | float data type : size of type : ABI alignment   |
*   n32      | all 32 bit registers are available               |
* 
*
*/
TriCoreTargetMachine::TriCoreTargetMachine(const Target &T, const Triple &TT,
                                           StringRef CPU, StringRef FS,
                                           const TargetOptions &Options,
                                           Reloc::Model RM, CodeModel::Model CM,
                                           CodeGenOpt::Level OL)
    : LLVMTargetMachine(T, computeDataLayout(), 
                        TT, CPU, FS, Options, RM, CM, OL),
      TLOF(make_unique<TargetLoweringObjectFileELF>()),
      Subtarget(TT, CPU, FS, *this) {
  initAsmInfo();
}

TriCoreTargetMachine::~TriCoreTargetMachine() {}

namespace {
/// TriCore Code Generator Pass Configuration Options.
class TriCorePassConfig : public TargetPassConfig {
public:
  TriCorePassConfig(TriCoreTargetMachine *TM, legacy::PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  TriCoreTargetMachine &getTriCoreTargetMachine() const {
    return getTM<TriCoreTargetMachine>();
  }

  virtual bool addPreISel() override;
  virtual bool addInstSelector() override;
  virtual void addPreEmitPass() override;
};
} // namespace

TargetPassConfig *TriCoreTargetMachine::createPassConfig(legacy::PassManagerBase &PM) {
  return new TriCorePassConfig(this, PM);
}

bool TriCorePassConfig::addPreISel() { return false; }

bool TriCorePassConfig::addInstSelector() {
  addPass(createTriCoreISelDag(getTriCoreTargetMachine(), getOptLevel()));
  return false;
}

void TriCorePassConfig::addPreEmitPass() {}

// Force static initialization.
extern "C" void LLVMInitializeTriCoreTarget() {
  RegisterTargetMachine<TriCoreTargetMachine> X(TheTriCoreTarget);
}
