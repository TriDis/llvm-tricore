set(LLVM_TARGET_DEFINITIONS TriCore.td)

tablegen(LLVM TriCoreGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM TriCoreGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM TriCoreGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM TriCoreGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM TriCoreGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM TriCoreGenCallingConv.inc -gen-callingconv)
tablegen(LLVM TriCoreGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM TriCoreGenMCCodeEmitter.inc -gen-emitter)
add_public_tablegen_target(TriCoreCommonTableGen)

add_llvm_target(TriCoreCodeGen
  TriCoreSubtarget.cpp
  TriCoreTargetMachine.cpp
  TriCoreRegisterInfo.cpp
  TriCoreInstrInfo.cpp
  TriCoreFrameLowering.cpp
  TriCoreISelLowering.cpp
  TriCoreSelectionDAGInfo.cpp
  TriCoreISelDAGToDAG.cpp
  TriCoreAsmPrinter.cpp
  TriCoreMCInstLower.cpp
  TriCoreCallingConvHook.cpp
  )

add_subdirectory(Disassembler)
add_subdirectory(InstPrinter)
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
