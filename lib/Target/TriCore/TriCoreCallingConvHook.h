/*
 * TriCoreCallingConvHook.h
 *
 *  Created on: Dec 2, 2015
 *      Author: kumail ahmed
 */

#ifndef TRICORECALLINGCONVHOOK_H_
#define TRICORECALLINGCONVHOOK_H_
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/Function.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/raw_ostream.h"

#include "TriCore.h"
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>

#define UNKNOWN_REG 5555

using namespace llvm;

struct regInfo{
  StringRef fName;
  unsigned reg;
  bool isPointer;
};

class TriCoreCallingConvHook {


private:
  uint32_t curPos = 0;
  uint32_t curArg = 0;


  std::vector< regInfo > regRecord;

public:
  TriCoreCallingConvHook();
  bool isRegValPtrType (MachineFunction& _mf);
  bool isRegValid64Type (MachineFunction& _mf);
  void init();
  void setCurPos(uint32_t curPos = 0) {this->curPos = curPos;}
  void setArgPos(uint32_t curArg = 0) {this->curArg = curArg;}

  void incrArgPos() {this->curArg++;}
  TriCoreCallingConvHook operator++(int);
  uint32_t operator()() { return this->curPos;}



  int32_t findInRegRecord(StringRef funString);
  int32_t findInRegLastRecord(StringRef funString);

  uint32_t getCurPos() const {  return this->curPos;}
  uint32_t getArgPos() const {  return curArg;}
  unsigned getNextAddrRegs(StringRef fName);
  unsigned getNextDataRegs(StringRef fName);
  unsigned getNextExtRegs(StringRef fName);
  bool getRegRecordisPointer(uint32_t pos);
  StringRef getFunctionName(uint32_t pos);
  unsigned getRegRecordRegister(uint32_t pos);
  uint32_t getNumOfArgs(StringRef fName);

  void saveRegRecord(StringRef funName, unsigned reg, bool isPointer);
  void printRegRecord();
};

extern TriCoreCallingConvHook TCCH;

#endif /* TRICORECALLINGCONVHOOK_H_ */
