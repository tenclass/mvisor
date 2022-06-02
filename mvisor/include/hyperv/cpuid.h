/* 
 * MVisor
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MVISOR_CPUID_H
#define MVISOR_CPUID_H

#define CPUID_TOPOLOGY_LEVEL_SMT      (1U << 8)
#define CPUID_TOPOLOGY_LEVEL_CORE     (2U << 8)
#define CPUID_TOPOLOGY_LEVEL_INVALID  (0U << 8)

#define MSR_IA32_TSC                  0x10
#define MSR_IA32_UCODE_REV            0x8B
#define MSR_IA32_PERF_CAPABILITIES    0x345


/* cpuid_features bits */
#define CPUID_FP87 (1U << 0)
#define CPUID_VME  (1U << 1)
#define CPUID_DE   (1U << 2)
#define CPUID_PSE  (1U << 3)
#define CPUID_TSC  (1U << 4)
#define CPUID_MSR  (1U << 5)
#define CPUID_PAE  (1U << 6)
#define CPUID_MCE  (1U << 7)
#define CPUID_CX8  (1U << 8)
#define CPUID_APIC (1U << 9)
#define CPUID_SEP  (1U << 11) /* sysenter/sysexit */
#define CPUID_MTRR (1U << 12)
#define CPUID_PGE  (1U << 13)
#define CPUID_MCA  (1U << 14)
#define CPUID_CMOV (1U << 15)
#define CPUID_PAT  (1U << 16)
#define CPUID_PSE36   (1U << 17)
#define CPUID_PN   (1U << 18)
#define CPUID_CLFLUSH (1U << 19)
#define CPUID_DTS (1U << 21)
#define CPUID_ACPI (1U << 22)
#define CPUID_MMX  (1U << 23)
#define CPUID_FXSR (1U << 24)
#define CPUID_SSE  (1U << 25)
#define CPUID_SSE2 (1U << 26)
#define CPUID_SS (1U << 27)
#define CPUID_HT (1U << 28)
#define CPUID_TM (1U << 29)
#define CPUID_IA64 (1U << 30)
#define CPUID_PBE (1U << 31)

#define CPUID_EXT_SSE3     (1U << 0)
#define CPUID_EXT_PCLMULQDQ (1U << 1)
#define CPUID_EXT_DTES64   (1U << 2)
#define CPUID_EXT_MONITOR  (1U << 3)
#define CPUID_EXT_DSCPL    (1U << 4)
#define CPUID_EXT_VMX      (1U << 5)
#define CPUID_EXT_SMX      (1U << 6)
#define CPUID_EXT_EST      (1U << 7)
#define CPUID_EXT_TM2      (1U << 8)
#define CPUID_EXT_SSSE3    (1U << 9)
#define CPUID_EXT_CID      (1U << 10)
#define CPUID_EXT_FMA      (1U << 12)
#define CPUID_EXT_CX16     (1U << 13)
#define CPUID_EXT_XTPR     (1U << 14)
#define CPUID_EXT_PDCM     (1U << 15)
#define CPUID_EXT_PCID     (1U << 17)
#define CPUID_EXT_DCA      (1U << 18)
#define CPUID_EXT_SSE41    (1U << 19)
#define CPUID_EXT_SSE42    (1U << 20)
#define CPUID_EXT_X2APIC   (1U << 21)
#define CPUID_EXT_MOVBE    (1U << 22)
#define CPUID_EXT_POPCNT   (1U << 23)
#define CPUID_EXT_TSC_DEADLINE_TIMER (1U << 24)
#define CPUID_EXT_AES      (1U << 25)
#define CPUID_EXT_XSAVE    (1U << 26)
#define CPUID_EXT_OSXSAVE  (1U << 27)
#define CPUID_EXT_AVX      (1U << 28)
#define CPUID_EXT_F16C     (1U << 29)
#define CPUID_EXT_RDRAND   (1U << 30)
#define CPUID_EXT_HYPERVISOR  (1U << 31)

#define CPUID_EXT2_FPU     (1U << 0)
#define CPUID_EXT2_VME     (1U << 1)
#define CPUID_EXT2_DE      (1U << 2)
#define CPUID_EXT2_PSE     (1U << 3)
#define CPUID_EXT2_TSC     (1U << 4)
#define CPUID_EXT2_MSR     (1U << 5)
#define CPUID_EXT2_PAE     (1U << 6)
#define CPUID_EXT2_MCE     (1U << 7)
#define CPUID_EXT2_CX8     (1U << 8)
#define CPUID_EXT2_APIC    (1U << 9)
#define CPUID_EXT2_SYSCALL (1U << 11)
#define CPUID_EXT2_MTRR    (1U << 12)
#define CPUID_EXT2_PGE     (1U << 13)
#define CPUID_EXT2_MCA     (1U << 14)
#define CPUID_EXT2_CMOV    (1U << 15)
#define CPUID_EXT2_PAT     (1U << 16)
#define CPUID_EXT2_PSE36   (1U << 17)
#define CPUID_EXT2_MP      (1U << 19)
#define CPUID_EXT2_NX      (1U << 20)
#define CPUID_EXT2_MMXEXT  (1U << 22)
#define CPUID_EXT2_MMX     (1U << 23)
#define CPUID_EXT2_FXSR    (1U << 24)
#define CPUID_EXT2_FFXSR   (1U << 25)
#define CPUID_EXT2_PDPE1GB (1U << 26)
#define CPUID_EXT2_RDTSCP  (1U << 27)
#define CPUID_EXT2_LM      (1U << 29)
#define CPUID_EXT2_3DNOWEXT (1U << 30)
#define CPUID_EXT2_3DNOW   (1U << 31)


/* Support RDFSBASE/RDGSBASE/WRFSBASE/WRGSBASE */
#define CPUID_7_0_EBX_FSGSBASE          (1U << 0)
/* Support SGX */
#define CPUID_7_0_EBX_TSC_ADJUST_MSR    (1U << 1)
/* Support SGX */
#define CPUID_7_0_EBX_SGX               (1U << 2)
/* 1st Group of Advanced Bit Manipulation Extensions */
#define CPUID_7_0_EBX_BMI1              (1U << 3)
/* Hardware Lock Elision */
#define CPUID_7_0_EBX_HLE               (1U << 4)
/* Intel Advanced Vector Extensions 2 */
#define CPUID_7_0_EBX_AVX2              (1U << 5)
/* Supervisor-mode Execution Prevention */
#define CPUID_7_0_EBX_SMEP              (1U << 7)
/* 2nd Group of Advanced Bit Manipulation Extensions */
#define CPUID_7_0_EBX_BMI2              (1U << 8)
/* Enhanced REP MOVSB/STOSB */
#define CPUID_7_0_EBX_ERMS              (1U << 9)
/* Invalidate Process-Context Identifier */
#define CPUID_7_0_EBX_INVPCID           (1U << 10)
/* Restricted Transactional Memory */
#define CPUID_7_0_EBX_RTM               (1U << 11)
/* Memory Protection Extension */
#define CPUID_7_0_EBX_MPX               (1U << 14)
/* AVX-512 Foundation */
#define CPUID_7_0_EBX_AVX512F           (1U << 16)
/* AVX-512 Doubleword & Quadword Instruction */
#define CPUID_7_0_EBX_AVX512DQ          (1U << 17)
/* Read Random SEED */
#define CPUID_7_0_EBX_RDSEED            (1U << 18)
/* ADCX and ADOX instructions */
#define CPUID_7_0_EBX_ADX               (1U << 19)
/* Supervisor Mode Access Prevention */
#define CPUID_7_0_EBX_SMAP              (1U << 20)
/* AVX-512 Integer Fused Multiply Add */
#define CPUID_7_0_EBX_AVX512IFMA        (1U << 21)
/* Persistent Commit */
#define CPUID_7_0_EBX_PCOMMIT           (1U << 22)
/* Flush a Cache Line Optimized */
#define CPUID_7_0_EBX_CLFLUSHOPT        (1U << 23)
/* Cache Line Write Back */
#define CPUID_7_0_EBX_CLWB              (1U << 24)
/* Intel Processor Trace */
#define CPUID_7_0_EBX_INTEL_PT          (1U << 25)
/* AVX-512 Prefetch */
#define CPUID_7_0_EBX_AVX512PF          (1U << 26)
/* AVX-512 Exponential and Reciprocal */
#define CPUID_7_0_EBX_AVX512ER          (1U << 27)
/* AVX-512 Conflict Detection */
#define CPUID_7_0_EBX_AVX512CD          (1U << 28)
/* SHA1/SHA256 Instruction Extensions */
#define CPUID_7_0_EBX_SHA_NI            (1U << 29)
/* AVX-512 Byte and Word Instructions */
#define CPUID_7_0_EBX_AVX512BW          (1U << 30)
/* AVX-512 Vector Length Extensions */
#define CPUID_7_0_EBX_AVX512VL          (1U << 31)

/* AVX-512 Vector Byte Manipulation Instruction */
#define CPUID_7_0_ECX_AVX512_VBMI       (1U << 1)
/* User-Mode Instruction Prevention */
#define CPUID_7_0_ECX_UMIP              (1U << 2)
/* Protection Keys for User-mode Pages */
#define CPUID_7_0_ECX_PKU               (1U << 3)
/* OS Enable Protection Keys */
#define CPUID_7_0_ECX_OSPKE             (1U << 4)
/* UMONITOR/UMWAIT/TPAUSE Instructions */
#define CPUID_7_0_ECX_WAITPKG           (1U << 5)
/* Additional AVX-512 Vector Byte Manipulation Instruction */
#define CPUID_7_0_ECX_AVX512_VBMI2      (1U << 6)
/* Galois Field New Instructions */
#define CPUID_7_0_ECX_GFNI              (1U << 8)
/* Vector AES Instructions */
#define CPUID_7_0_ECX_VAES              (1U << 9)
/* Carry-Less Multiplication Quadword */
#define CPUID_7_0_ECX_VPCLMULQDQ        (1U << 10)
/* Vector Neural Network Instructions */
#define CPUID_7_0_ECX_AVX512VNNI        (1U << 11)
/* Support for VPOPCNT[B,W] and VPSHUFBITQMB */
#define CPUID_7_0_ECX_AVX512BITALG      (1U << 12)
/* POPCNT for vectors of DW/QW */
#define CPUID_7_0_ECX_AVX512_VPOPCNTDQ  (1U << 14)
/* 5-level Page Tables */
#define CPUID_7_0_ECX_LA57              (1U << 16)
/* Read Processor ID */
#define CPUID_7_0_ECX_RDPID             (1U << 22)
/* Bus Lock Debug Exception */
#define CPUID_7_0_ECX_BUS_LOCK_DETECT   (1U << 24)
/* Cache Line Demote Instruction */
#define CPUID_7_0_ECX_CLDEMOTE          (1U << 25)
/* Move Doubleword as Direct Store Instruction */
#define CPUID_7_0_ECX_MOVDIRI           (1U << 27)
/* Move 64 Bytes as Direct Store Instruction */
#define CPUID_7_0_ECX_MOVDIR64B         (1U << 28)
/* Support SGX Launch Control */
#define CPUID_7_0_ECX_SGX_LC            (1U << 30)
/* Protection Keys for Supervisor-mode Pages */
#define CPUID_7_0_ECX_PKS               (1U << 31)

/* AVX512 Neural Network Instructions */
#define CPUID_7_0_EDX_AVX512_4VNNIW     (1U << 2)
/* AVX512 Multiply Accumulation Single Precision */
#define CPUID_7_0_EDX_AVX512_4FMAPS     (1U << 3)
/* Fast Short Rep Mov */
#define CPUID_7_0_EDX_FSRM              (1U << 4)
/* AVX512 Vector Pair Intersection to a Pair of Mask Registers */
#define CPUID_7_0_EDX_AVX512_VP2INTERSECT (1U << 8)
/* SERIALIZE instruction */
#define CPUID_7_0_EDX_SERIALIZE         (1U << 14)
/* TSX Suspend Load Address Tracking instruction */
#define CPUID_7_0_EDX_TSX_LDTRK         (1U << 16)
/* AVX512_FP16 instruction */
#define CPUID_7_0_EDX_AVX512_FP16       (1U << 23)
/* Speculation Control */
#define CPUID_7_0_EDX_SPEC_CTRL         (1U << 26)
/* Single Thread Indirect Branch Predictors */
#define CPUID_7_0_EDX_STIBP             (1U << 27)
/* Arch Capabilities */
#define CPUID_7_0_EDX_ARCH_CAPABILITIES (1U << 29)
/* Core Capability */
#define CPUID_7_0_EDX_CORE_CAPABILITY   (1U << 30)
/* Speculative Store Bypass Disable */
#define CPUID_7_0_EDX_SPEC_CTRL_SSBD    (1U << 31)

/* AVX VNNI Instruction */
#define CPUID_7_1_EAX_AVX_VNNI          (1U << 4)
/* AVX512 BFloat16 Instruction */
#define CPUID_7_1_EAX_AVX512_BF16       (1U << 5)

/* XCR0 */
#define XSTATE_FP_BIT                   0
#define XSTATE_SSE_BIT                  1
#define XSTATE_YMM_BIT                  2
#define XSTATE_BNDREGS_BIT              3
#define XSTATE_BNDCSR_BIT               4
#define XSTATE_OPMASK_BIT               5
#define XSTATE_ZMM_Hi256_BIT            6
#define XSTATE_Hi16_ZMM_BIT             7
#define XSTATE_PKRU_BIT                 9

#define XSTATE_FP_MASK                  (1ULL << XSTATE_FP_BIT)
#define XSTATE_SSE_MASK                 (1ULL << XSTATE_SSE_BIT)
#define XSTATE_YMM_MASK                 (1ULL << XSTATE_YMM_BIT)
#define XSTATE_BNDREGS_MASK             (1ULL << XSTATE_BNDREGS_BIT)
#define XSTATE_BNDCSR_MASK              (1ULL << XSTATE_BNDCSR_BIT)
#define XSTATE_OPMASK_MASK              (1ULL << XSTATE_OPMASK_BIT)
#define XSTATE_ZMM_Hi256_MASK           (1ULL << XSTATE_ZMM_Hi256_BIT)
#define XSTATE_Hi16_ZMM_MASK            (1ULL << XSTATE_Hi16_ZMM_BIT)
#define XSTATE_PKRU_MASK                (1ULL << XSTATE_PKRU_BIT)


#define ALTER_FEATURE(ecx, bit, on) \
  if (on) { \
    ecx |= bit; \
  } else { \
    ecx &= ~bit; \
  }

#endif // MVISOR_CPUID_H
