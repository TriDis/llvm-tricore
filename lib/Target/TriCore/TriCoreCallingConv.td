//=- TriCoreCallingConv.td - Calling Conventions for TriCore -*- tablegen -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for TriCore architecture.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// TriCore Return Value Calling Convention
//===----------------------------------------------------------------------===//
def RetCC_TriCore : CallingConv<[
  // Promote i8/i16 arguments to i32.
  CCIfType<[i8, i16], CCPromoteToType<i32>>,

  // i32 are returned in registers D2
  CCIfType<[i32], CCAssignToReg<[D2]>>,
  CCIfType<[i64], CCAssignToReg<[E2]>>

  // Integer values get stored in stack slots that are 4 bytes in
  // size and 4-byte aligned.
  //CCIfType<[i32], CCAssignToStack<4, 4>>
]>;

//===----------------------------------------------------------------------===//
// TriCore Argument Calling Conventions
//===----------------------------------------------------------------------===//
def CC_TriCore : CallingConv<[
  // Promote i8/i16 arguments to i32.
  CCIfType<[i8, i16], CCPromoteToType<i32>>,


  // The first 4 integer arguments are passed in integer registers.
  //  CCIfType<[i32], CCAssignToReg<[D4, D5, D6, D7]>>,

  CCIfType<[i64], CCAssignToReg<[E4, E6]>>,

  // Pointer arguments are handled inside TriCoreIselLowering, because
  // LLVM lowers i32** type into i32, hence there is no way to distinguish
  // between a pointer type and an integer type.


  // Integer values get stored in stack slots that are 4 bytes in
  // size and 4-byte aligned.
  CCIfType<[i32], CCAssignToStack<4, 4>>,
  CCIfType<[i64], CCAssignToStack<8, 4>>
]>;

def CC_Save : CalleeSavedRegs<(add A2, A3, A4, A5, A6, A7,
                                   D0, D1, D2, D3, D4, D5, D6, D7,
                                   A11)>;
