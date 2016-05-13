//===-- TriCoreInstrInfo.h - TriCore Instruction Information ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the TriCore implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef TriCoreINSTRUCTIONINFO_H
#define TriCoreINSTRUCTIONINFO_H

#include "TriCoreRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "TriCoreGenInstrInfo.inc"

namespace llvm {

namespace TriCoreCC {
  enum CondCodes {
    COND_EQ, // Equal
    COND_NE, // Not equal
    COND_GE, // Greater than or equal
    COND_LT, // Less than
    COND_INVALID
  };

  enum LogicCodes {
      LOGIC_AND, // AND
      LOGIC_OR,  // OR
      LOGIC_INVALID
    };
}

class TriCoreInstrInfo : public TriCoreGenInstrInfo {
  const TriCoreRegisterInfo RI;
  virtual void anchor();

public:
  TriCoreInstrInfo();

  /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
  /// such, whenever a client has an instance of instruction info, it should
  /// always be able to get register info as well (through this method).
  ///
  const TriCoreRegisterInfo &getRegisterInfo() const { return RI; }

  /// isLoadFromStackSlot - If the specified machine instruction is a direct
  /// load from a stack slot, return the virtual or physical register number of
  /// the destination along with the FrameIndex of the loaded stack slot.  If
  /// not, return 0.  This predicate must return 0 if the instruction has
  /// any side effects other than loading from the stack slot.
  virtual unsigned isLoadFromStackSlot(const MachineInstr *MI,
                                       int &FrameIndex) const override;
//
//  /// isStoreToStackSlot - If the specified machine instruction is a direct
//  /// store to a stack slot, return the virtual or physical register number of
//  /// the source reg along with the FrameIndex of the loaded stack slot.  If
//  /// not, return 0.  This predicate must return 0 if the instruction has
//  /// any side effects other than storing to the stack slot.
  virtual unsigned isStoreToStackSlot(const MachineInstr *MI,
                                      int &FrameIndex) const override;


  virtual void copyPhysReg(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator I, DebugLoc DL,
                           unsigned DestReg, unsigned SrcReg,
                           bool KillSrc) const override;
//
//  virtual void storeRegToStackSlot(MachineBasicBlock &MBB,
//                                   MachineBasicBlock::iterator MI,
//                                   unsigned SrcReg, bool isKill, int FrameIndex,
//                                   const TargetRegisterClass *RC,
//                                   const TargetRegisterInfo *TRI) const
//                                   override;
//
//  virtual void loadRegFromStackSlot(MachineBasicBlock &MBB,
//                                    MachineBasicBlock::iterator MI,
//                                    unsigned DestReg, int FrameIndex,
//                                    const TargetRegisterClass *RC,
//                                    const TargetRegisterInfo *TRI) const
//                                    override;

  void splitRegs(unsigned Reg, unsigned &LoReg, unsigned &HiReg) const;

   virtual bool expandPostRAPseudo(MachineBasicBlock::iterator MI) const
     override;

//  TriCoreCC::CondCodes getCondFromBranchOpc(unsigned Opc) const;
//  TriCoreCC::CondCodes getOppositeCondition(TriCoreCC::CondCodes CC) const;
//  const MCInstrDesc& getBrCond(TriCoreCC::CondCodes CC) const;
//  bool AnalyzeBranch(MachineBasicBlock &MBB,
//                       MachineBasicBlock *&TBB, MachineBasicBlock *&FBB,
//                       SmallVectorImpl<MachineOperand> &Cond,
//                       std::vector<unsigned> &s1,
//                       std::vector<unsigned> &s2,
//                       bool AllowModify) const;
//
//  unsigned RemoveBranch(MachineBasicBlock &MBB) const override;
//  unsigned InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
//                          MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
//                          std::vector<unsigned> &s1,
//                          std::vector<unsigned> &s2,
//                          DebugLoc DL) const ;
};
}

#endif
