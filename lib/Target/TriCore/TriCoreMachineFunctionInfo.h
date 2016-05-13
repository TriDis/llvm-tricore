//=-- TriCoreMachineFuctionInfo.h - TriCore machine function info -*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares TriCore-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#ifndef TriCoreMACHINEFUNCTIONINFO_H
#define TriCoreMACHINEFUNCTIONINFO_H

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

// Forward declarations
class Function;

/// TriCoreFunctionInfo - This class is derived from MachineFunction private
/// TriCore target-specific information for each MachineFunction.
class TriCoreFunctionInfo : public MachineFunctionInfo {
public:
  TriCoreFunctionInfo() {}

  ~TriCoreFunctionInfo() {}
};
} // End llvm namespace

#endif // TriCoreMACHINEFUNCTIONINFO_H

