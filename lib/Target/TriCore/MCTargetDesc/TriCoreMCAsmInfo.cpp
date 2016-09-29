//===-- TriCoreMCAsmInfo.cpp - TriCore asm properties ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "TriCoreMCAsmInfo.h"
#include "llvm/ADT/StringRef.h"
using namespace llvm;

void TriCoreMCAsmInfo::anchor() {}

TriCoreMCAsmInfo::TriCoreMCAsmInfo(const Triple &TT) {
  SupportsDebugInformation = true;
  Data8bitsDirective  = "\t.byte\t";
  Data16bitsDirective = "\t.short\t";
  Data32bitsDirective = "\t.word\t";
  Data64bitsDirective = nullptr;
  ZeroDirective = "\t.zero\t";
  CommentString = "#";
  UsesELFSectionDirectiveForBSS = true;
  AlignmentIsInBytes = false;

  AscizDirective = "\t.string ";

  HiddenVisibilityAttr = MCSA_Invalid;
  HiddenDeclarationVisibilityAttr = MCSA_Invalid;
  ProtectedVisibilityAttr = MCSA_Invalid;
}

