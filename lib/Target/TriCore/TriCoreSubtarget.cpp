//==-- TriCoreSubtarget.cpp - TriCore Subtarget Information ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the TriCore specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "TriCoreSubtarget.h"
#include "TriCore.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "tricore-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "TriCoreGenSubtargetInfo.inc"

void TriCoreSubtarget::anchor() { }

TriCoreSubtarget::TriCoreSubtarget(const Triple &TT, const std::string &CPU,
                               const std::string &FS, const TargetMachine &TM)
    : TriCoreGenSubtargetInfo(TT, CPU, FS),
      DL("e-m:e-p:32:32-i64:32-a:0:32-n32"),
      InstrInfo(), FrameLowering(*this), TLInfo(TM, *this), TSInfo() {}
