/*
 * TriCoreCallingConvHook.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: adminuser
 */

#include "TriCoreCallingConvHook.h"

TriCoreCallingConvHook::TriCoreCallingConvHook() : curPos(0), curArg(0) {}

void TriCoreCallingConvHook::init(){
  curPos = 0;
  curArg = 0;
  //regRecord.clear();
}

TriCoreCallingConvHook TriCoreCallingConvHook::
                  operator++(int) {
  this->curPos++;
  return *this;
}

bool TriCoreCallingConvHook::isRegValPtrType (MachineFunction& _mf) {
  Function::const_arg_iterator FI;
  FI = _mf.getFunction()->arg_begin();
  std::advance(FI,curArg);
  return FI->getType()->isPointerTy()? true : false;
}

bool TriCoreCallingConvHook::isRegValid64Type (MachineFunction& _mf) {
  Function::const_arg_iterator FI;
  FI = _mf.getFunction()->arg_begin();
  std::advance(FI,curArg);
  outs() << "size: " << FI->getType()->getScalarSizeInBits() << "\n";
  return (FI->getType()->getScalarSizeInBits() == 64) ? true : false;
}

void TriCoreCallingConvHook::saveRegRecord(StringRef funName, unsigned reg, bool isPointer){
  struct regInfo tmp;
  tmp.fName = funName;
  tmp.reg = reg;
  tmp.isPointer = isPointer;
  regRecord.push_back(tmp);

}

StringRef TriCoreCallingConvHook::getFunctionName(uint32_t pos) {
  return regRecord[pos].fName;
}

unsigned TriCoreCallingConvHook::getRegRecordRegister(uint32_t pos) {
  return regRecord[pos].reg;
}

bool TriCoreCallingConvHook::getRegRecordisPointer(uint32_t pos) {
  return regRecord[pos].isPointer;
}

void TriCoreCallingConvHook::printRegRecord() {
  for (auto& rec : regRecord)
    outs()<< rec.fName <<"\t"<<rec.reg<<"\t"<<rec.isPointer<<"\n";
}

int32_t TriCoreCallingConvHook::findInRegRecord(StringRef funString) {

  uint32_t index = 0;
  for (auto& rec : regRecord) {
    if(rec.fName.compare(funString) == 0){
      return index;
    }
    index++;
  }
  return -1;
}

int32_t TriCoreCallingConvHook::findInRegLastRecord(StringRef funString) {
  uint32_t index = regRecord.size();
  for (int32_t i = index-1; i>=0; i--) {
    if(regRecord[i].fName.compare(funString) == 0)
      return i;
  }
  return -1;
}

unsigned TriCoreCallingConvHook::getNextAddrRegs(StringRef fName) {

  std::vector<unsigned> regList;

  for (auto& rec : regRecord) {
    if (rec.fName==fName && rec.reg >= TriCore::A4 && rec.reg <= TriCore::A7)
      regList.push_back(rec.reg);
  }
  if (regList.size() == 0)
    return TriCore::A4;

  unsigned lastReg = regList.back();

  if (lastReg == TriCore::A7)
    return UNKNOWN_REG;

  return lastReg + 1;
}

unsigned TriCoreCallingConvHook::getNextDataRegs(StringRef fName) {

  std::vector<unsigned> regList;

  for (auto& rec : regRecord) {
    if (rec.fName==fName && rec.reg >= TriCore::D4 && rec.reg <= TriCore::D7)
      regList.push_back(rec.reg);
  }
  if (regList.size() == 0)
    return TriCore::D4;

  unsigned lastReg = regList.back();

  if (lastReg == TriCore::D7)
    return UNKNOWN_REG;
  return lastReg + 1;
}

unsigned TriCoreCallingConvHook::getNextExtRegs(StringRef fName) {

  std::vector<unsigned> regList;

  for (auto& rec : regRecord) {
    if (rec.fName==fName && rec.reg >= TriCore::E4 && rec.reg <= TriCore::E6)
      regList.push_back(rec.reg);
  }
  if (regList.size() == 0)
    return TriCore::E4;

  unsigned lastReg = regList.back();

  if (lastReg == TriCore::E6)
    return UNKNOWN_REG;
  return lastReg + 1;
}

uint32_t TriCoreCallingConvHook::getNumOfArgs(StringRef fName) {

  uint32_t count = 0;
  for (auto& rec : regRecord) {
    if(rec.fName.compare(fName)==0)
      count++;
  }

  return count;

}


TriCoreCallingConvHook TCCH;
