//===-- TriCoreMCAsmInfo.h - TriCore asm properties ------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the TriCoreMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef TriCoreTARGETASMINFO_H
#define TriCoreTARGETASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class StringRef;
class Target;
class Triple;

class TriCoreMCAsmInfo : public MCAsmInfoELF {
  virtual void anchor();

public:
  explicit TriCoreMCAsmInfo(const Triple &TT);
};

} // namespace llvm

#endif
