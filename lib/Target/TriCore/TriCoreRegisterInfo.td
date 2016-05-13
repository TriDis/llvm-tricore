//==-- TriCoreRegisterInfo.td - TriCore Register defs ------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the TriCore register file
//===----------------------------------------------------------------------===//

class TriCoreReg<string n> : Register<n> {
  field bits<16> Num;
  let Namespace = "TriCore";
  let HWEncoding = Num;
}

// General Purpose Data Registers
class TriCoreDataReg<bits<16> num, string n> : TriCoreReg<n> {
  let Num = num;
}

// General Purpose Address Registers
class TriCoreAddrReg<bits<16> num, string n> : TriCoreReg<n> {
  let Num = num;
}

// Program Status Register
class TriCorePSReg<bits<16> num, string n> : TriCoreReg<n> {
  let Num = num;
}

class TriCoreRegWithSubregs<bits<16> num, string n, list<Register> subregs>
: RegisterWithSubRegs<n, subregs> {
  field bits<16> Num;

  let Num = num;
  let Namespace = "TriCore";
}

//===----------------------------------------------------------------------===//
//@Registers
//===----------------------------------------------------------------------===//
// The register string, such as "D0" or "D13" will show on "llvm-objdump -d"

def D0 : TriCoreDataReg<0, "D0">, DwarfRegNum<[0]>;
def D1 : TriCoreDataReg<1, "D1">, DwarfRegNum<[1]>;
def D2 : TriCoreDataReg<2, "D2">, DwarfRegNum<[2]>;
def D3 : TriCoreDataReg<3, "D3">, DwarfRegNum<[3]>;
def D4 : TriCoreDataReg<4, "D4">, DwarfRegNum<[4]>;
def D5 : TriCoreDataReg<5, "D5">, DwarfRegNum<[5]>;
def D6 : TriCoreDataReg<6, "D6">, DwarfRegNum<[6]>;
def D7 : TriCoreDataReg<7, "D7">, DwarfRegNum<[7]>;
def D8 : TriCoreDataReg<8, "D8">, DwarfRegNum<[8]>;
def D9 : TriCoreDataReg<9, "D9">, DwarfRegNum<[9]>;
def D10 : TriCoreDataReg<10, "D10">, DwarfRegNum<[10]>;
def D11 : TriCoreDataReg<11, "D11">, DwarfRegNum<[11]>;
def D12 : TriCoreDataReg<12, "D12">, DwarfRegNum<[12]>;
def D13 : TriCoreDataReg<13, "D13">, DwarfRegNum<[13]>;
def D14 : TriCoreDataReg<14, "D14">, DwarfRegNum<[14]>;
def D15 : TriCoreDataReg<15, "D15">, DwarfRegNum<[15]>;


def A0 : TriCoreAddrReg<0, "A0">, DwarfRegNum<[16]>;
def A1 : TriCoreAddrReg<1, "A1">, DwarfRegNum<[17]>;
def A2 : TriCoreAddrReg<2, "A2">, DwarfRegNum<[18]>;
def A3 : TriCoreAddrReg<3, "A3">, DwarfRegNum<[19]>;
def A4 : TriCoreAddrReg<4, "A4">, DwarfRegNum<[20]>;
def A5 : TriCoreAddrReg<5, "A5">, DwarfRegNum<[21]>;
def A6 : TriCoreAddrReg<6, "A6">, DwarfRegNum<[22]>;
def A7 : TriCoreAddrReg<7, "A7">, DwarfRegNum<[23]>;
def A8 : TriCoreAddrReg<8, "A8">, DwarfRegNum<[24]>;
def A9 : TriCoreAddrReg<9, "A9">, DwarfRegNum<[25]>;
def A10 : TriCoreAddrReg<10, "A10">, DwarfRegNum<[26]>;
def A11 : TriCoreAddrReg<11, "A11">, DwarfRegNum<[27]>;
def A12 : TriCoreAddrReg<12, "A12">, DwarfRegNum<[28]>;
def A13 : TriCoreAddrReg<13, "A13">, DwarfRegNum<[29]>;
def A14 : TriCoreAddrReg<14, "A14">, DwarfRegNum<[30]>;
def A15 : TriCoreAddrReg<15, "A15">, DwarfRegNum<[31]>;

let Namespace = "TriCore" in {
def subreg_even : SubRegIndex<32>;
def subreg_odd  : SubRegIndex<32, 32>;
}

//Extended 64-bit registers
let SubRegIndices = [subreg_even, subreg_odd] in {
def E0 : TriCoreRegWithSubregs<0,   "E0",  [D0,D1] >, DwarfRegNum<[32]>;
def E2 : TriCoreRegWithSubregs<2,   "E2",  [D2,D3] >, DwarfRegNum<[33]>;
def E4 : TriCoreRegWithSubregs<4,   "E4",  [D4,D5] >, DwarfRegNum<[34]>;
def E6 : TriCoreRegWithSubregs<6,   "E6",  [D6,D7] >, DwarfRegNum<[35]>;
def E8 : TriCoreRegWithSubregs<8,   "E8",  [D8,D9] >, DwarfRegNum<[36]>;
def E10 : TriCoreRegWithSubregs<10, "E10", [D10,D11] >, DwarfRegNum<[37]>;
def E12 : TriCoreRegWithSubregs<12, "E12", [D12,D13] >, DwarfRegNum<[38]>;
def E14 : TriCoreRegWithSubregs<14, "E14", [D14,D15] >, DwarfRegNum<[39]>;
}

//Program Status Information Registers
def PSW  : TriCorePSReg<0, "PSW">, DwarfRegNum<[40]>;
def PCXI : TriCorePSReg<1, "PCXI">, DwarfRegNum<[41]>;
def PC   : TriCorePSReg<2, "PC">, DwarfRegNum<[42]>;
def FCX  : TriCorePSReg<3, "FCX">, DwarfRegNum<[43]>;

//===----------------------------------------------------------------------===//
//@Register Classes
//===----------------------------------------------------------------------===//

def DataRegs : RegisterClass<"TriCore", [i32], 32, (add
    // Implict Data
    D15,
    // Mostly Used
    D2, D3,
    D4, D5, D6, D7,
    D8, D9, D10, D11,
    D12, D13, D14,
    // Others - Compiler Specific
    D0, D1)>;

def AddrRegs : RegisterClass<"TriCore", [i32], 32, (add
    // Implicit Address
    A15,
    // Others
    A2, A3, A4, A5, A6,
    A7, A12, A13, A14,
    // Stack Pointer
    A10,
    // Return Address
    A11,
    // Global Address
    A0, A1, A8, A9)>;

def ExtRegs : RegisterClass<"TriCore", [i64], 32, (add
    E2, E4,
    E6, E8, E10,
    E12, E14, E0)>;

def PSRegs : RegisterClass<"TriCore", [i32], 32, (add
    PSW, PCXI, PC, FCX)>;
