//==-- TriCoreISelLowering.h - TriCore DAG Lowering Interface -----*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that TriCore uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_TRICORE_TRICOREISELLOWERING_H
#define LLVM_LIB_TARGET_TRICORE_TRICOREISELLOWERING_H

#include "TriCore.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/Target/TargetLowering.h"

namespace llvm {

  // Forward declarations
  class TriCoreSubtarget;
  class TriCoreTargetMachine;

  namespace TriCoreISD {
    enum NodeType : unsigned {
      // Start the numbering where the builtin ops and target ops leave off.
      FIRST_NUMBER = ISD::BUILTIN_OP_END,
      RET_FLAG,
      // This loads the symbol (e.g. global address) into a register.
      LOAD_SYM,
      // This loads a 32-bit immediate into a register.
      MOVEi32,
      CALL,
      // TriCore has a different way of lowering branch conditions.
      BR_CC,
      // This loads the comparison type, as Tricore doesn't support all
      // sorts of comparisons, some have to be created.
      CMP,
      // This load the addressing information
      Wrapper,
      // This loads the Shift instructions operands. Right and left shift
      // depends on the signed-ness on the shift value. A negytive value is
      // a right shift, and vice versa.
      SH,
      // Arithmetic Shift
      SHA,
      // Loads ternary operators
      SELECT_CC,
      LOGICCMP,
      IMASK,
      EXTR,
      ABS
    };
  }

  //===--------------------------------------------------------------------===//
  // TargetLowering Implementation
  //===--------------------------------------------------------------------===//
  class TriCoreTargetLowering : public TargetLowering
  {
  public:
    explicit TriCoreTargetLowering(const TargetMachine &TM,
                                 const TriCoreSubtarget &Subtarget);

    /// LowerOperation - Provide custom lowering hooks for some operations.
    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

    /// getTargetNodeName - This method returns the name of a target specific
    //  DAG node.
    const char *getTargetNodeName(unsigned Opcode) const override;

  private:
    const TargetMachine &TM;
    const TriCoreSubtarget &Subtarget;

    SDValue
      LowerFormalArguments(SDValue Chain,
                           CallingConv::ID CallConv,
                           bool isVarArg,
                           const SmallVectorImpl<ISD::InputArg> &Ins,
                           SDLoc dl, SelectionDAG &DAG,
                           SmallVectorImpl<SDValue> &InVals) const override;

    SDValue LowerCall(TargetLowering::CallLoweringInfo &CLI,
                      SmallVectorImpl<SDValue> &InVals) const override;

    SDValue LowerReturn(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
                        const SmallVectorImpl<ISD::OutputArg> &Outs,
                        const SmallVectorImpl<SDValue> &OutVals, SDLoc dl,
                        SelectionDAG &DAG) const override;

    SDValue LowerCallResult(SDValue Chain, SDValue InGlue,
                            CallingConv::ID CallConv, bool isVarArg,
                            const SmallVectorImpl<ISD::InputArg> &Ins, SDLoc dl,
                            SelectionDAG &DAG,
                            SmallVectorImpl<SDValue> &InVals) const;

    bool CanLowerReturn(CallingConv::ID CallConv, MachineFunction &MF,
                        bool isVarArg,
                        const SmallVectorImpl<ISD::OutputArg> &ArgsFlags,
                        LLVMContext &Context) const;

    // LowerGlobalAddress - Emit a constant load to the global address.
    SDValue LowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const;

    MachineBasicBlock* EmitInstrWithCustomInserter(MachineInstr *MI,
                                                    MachineBasicBlock *BB) const;

    // Lower Branch
    SDValue LowerBR_CC(SDValue Op, SelectionDAG &DAG) const;

    // Lower SELECT_CC
    SDValue LowerSELECT_CC(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerSETCC(SDValue Op, SelectionDAG &DAG) const;

    // Lower Shift Instruction
    SDValue LowerShifts(SDValue Op, SelectionDAG &DAG) const;
  };
}

#endif

