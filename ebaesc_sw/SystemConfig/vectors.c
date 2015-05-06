/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2013 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
********************************************************************************
*
* $File Name: vectors_845xx.c$
*
* $Date:      Nov-2-2012$
*
* $Version:   2.5.3.0$
*
* Description: 56F824x/5x Interrupt Vector Table (66 interrupts, INTC_VERSION_5)
*
*****************************************************************************/

#include "qs.h"

#ifndef INTC_VERSION_6
#error Bad INTC version defined in arch.h, or wrong vectors.c used
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* reference to startup code */

void Start(void);

/* for each interrupt vector, we (re)define HERE its correct prototype -
   - So, in the case the user defines BAD prototype himself e.g. in appconfig.h,
     the compiler warns him here about function prototype redefinition */
     
#ifdef INT_VECTOR_ADDR_1
void INT_VECTOR_ADDR_1(void);
#endif
#ifdef INT_VECTOR_ADDR_2
void INT_VECTOR_ADDR_2(void);
#endif
#ifdef INT_VECTOR_ADDR_3
void INT_VECTOR_ADDR_3(void);
#endif
#ifdef INT_VECTOR_ADDR_4
void INT_VECTOR_ADDR_4(void);
#endif
#ifdef INT_VECTOR_ADDR_5
void INT_VECTOR_ADDR_5(void);
#endif
#ifdef INT_VECTOR_ADDR_6
void INT_VECTOR_ADDR_6(void);
#endif
#ifdef INT_VECTOR_ADDR_7
void INT_VECTOR_ADDR_7(void);
#endif
#ifdef INT_VECTOR_ADDR_8
void INT_VECTOR_ADDR_8(void);
#endif
#ifdef INT_VECTOR_ADDR_9
void INT_VECTOR_ADDR_9(void);
#endif
#ifdef INT_VECTOR_ADDR_10
void INT_VECTOR_ADDR_10(void);
#endif
#ifdef INT_VECTOR_ADDR_11
void INT_VECTOR_ADDR_11(void);
#endif
#ifdef INT_VECTOR_ADDR_12
void INT_VECTOR_ADDR_12(void);
#endif
#ifdef INT_VECTOR_ADDR_13
void INT_VECTOR_ADDR_13(void);
#endif
#ifdef INT_VECTOR_ADDR_14
void INT_VECTOR_ADDR_14(void);
#endif
#ifdef INT_VECTOR_ADDR_15
void INT_VECTOR_ADDR_15(void);
#endif
#ifdef INT_VECTOR_ADDR_16
void INT_VECTOR_ADDR_16(void);
#endif
#ifdef INT_VECTOR_ADDR_17
void INT_VECTOR_ADDR_17(void);
#endif
#ifdef INT_VECTOR_ADDR_18
void INT_VECTOR_ADDR_18(void);
#endif
#ifdef INT_VECTOR_ADDR_19
void INT_VECTOR_ADDR_19(void);
#endif
#ifdef INT_VECTOR_ADDR_20
void INT_VECTOR_ADDR_20(void);
#endif
#ifdef INT_VECTOR_ADDR_21
void INT_VECTOR_ADDR_21(void);
#endif
#ifdef INT_VECTOR_ADDR_22
void INT_VECTOR_ADDR_22(void);
#endif
#ifdef INT_VECTOR_ADDR_23
void INT_VECTOR_ADDR_23(void);
#endif
#ifdef INT_VECTOR_ADDR_24
void INT_VECTOR_ADDR_24(void);
#endif
#ifdef INT_VECTOR_ADDR_25
void INT_VECTOR_ADDR_25(void);
#endif
#ifdef INT_VECTOR_ADDR_26
void INT_VECTOR_ADDR_26(void);
#endif
#ifdef INT_VECTOR_ADDR_27
void INT_VECTOR_ADDR_27(void);
#endif
#ifdef INT_VECTOR_ADDR_28
void INT_VECTOR_ADDR_28(void);
#endif
#ifdef INT_VECTOR_ADDR_29
void INT_VECTOR_ADDR_29(void);
#endif
#ifdef INT_VECTOR_ADDR_30
void INT_VECTOR_ADDR_30(void);
#endif
#ifdef INT_VECTOR_ADDR_31
void INT_VECTOR_ADDR_31(void);
#endif
#ifdef INT_VECTOR_ADDR_32
void INT_VECTOR_ADDR_32(void);
#endif
#ifdef INT_VECTOR_ADDR_33
void INT_VECTOR_ADDR_33(void);
#endif
#ifdef INT_VECTOR_ADDR_34
void INT_VECTOR_ADDR_34(void);
#endif
#ifdef INT_VECTOR_ADDR_35
void INT_VECTOR_ADDR_35(void);
#endif
#ifdef INT_VECTOR_ADDR_36
void INT_VECTOR_ADDR_36(void);
#endif
#ifdef INT_VECTOR_ADDR_37
void INT_VECTOR_ADDR_37(void);
#endif
#ifdef INT_VECTOR_ADDR_38
void INT_VECTOR_ADDR_38(void);
#endif
#ifdef INT_VECTOR_ADDR_39
void INT_VECTOR_ADDR_39(void);
#endif
#ifdef INT_VECTOR_ADDR_40
void INT_VECTOR_ADDR_40(void);
#endif
#ifdef INT_VECTOR_ADDR_41
void INT_VECTOR_ADDR_41(void);
#endif
#ifdef INT_VECTOR_ADDR_42
void INT_VECTOR_ADDR_42(void);
#endif
#ifdef INT_VECTOR_ADDR_43
void INT_VECTOR_ADDR_43(void);
#endif
#ifdef INT_VECTOR_ADDR_44
void INT_VECTOR_ADDR_44(void);
#endif
#ifdef INT_VECTOR_ADDR_45
void INT_VECTOR_ADDR_45(void);
#endif
#ifdef INT_VECTOR_ADDR_46
void INT_VECTOR_ADDR_46(void);
#endif
#ifdef INT_VECTOR_ADDR_47
void INT_VECTOR_ADDR_47(void);
#endif
#ifdef INT_VECTOR_ADDR_48
void INT_VECTOR_ADDR_48(void);
#endif
#ifdef INT_VECTOR_ADDR_49
void INT_VECTOR_ADDR_49(void);
#endif
#ifdef INT_VECTOR_ADDR_50
void INT_VECTOR_ADDR_50(void);
#endif
#ifdef INT_VECTOR_ADDR_51
void INT_VECTOR_ADDR_51(void);
#endif
#ifdef INT_VECTOR_ADDR_52
void INT_VECTOR_ADDR_52(void);
#endif
#ifdef INT_VECTOR_ADDR_53
void INT_VECTOR_ADDR_53(void);
#endif
#ifdef INT_VECTOR_ADDR_54
void INT_VECTOR_ADDR_54(void);
#endif
#ifdef INT_VECTOR_ADDR_55
void INT_VECTOR_ADDR_55(void);
#endif
#ifdef INT_VECTOR_ADDR_56
void INT_VECTOR_ADDR_56(void);
#endif
#ifdef INT_VECTOR_ADDR_57
void INT_VECTOR_ADDR_57(void);
#endif
#ifdef INT_VECTOR_ADDR_58
void INT_VECTOR_ADDR_58(void);
#endif
#ifdef INT_VECTOR_ADDR_59
void INT_VECTOR_ADDR_59(void);
#endif
#ifdef INT_VECTOR_ADDR_60
void INT_VECTOR_ADDR_60(void);
#endif
#ifdef INT_VECTOR_ADDR_61
void INT_VECTOR_ADDR_61(void);
#endif
#ifdef INT_VECTOR_ADDR_62
void INT_VECTOR_ADDR_62(void);
#endif
#ifdef INT_VECTOR_ADDR_63
void INT_VECTOR_ADDR_63(void);
#endif
#ifdef INT_VECTOR_ADDR_64
void INT_VECTOR_ADDR_64(void);
#endif
#ifdef INT_VECTOR_ADDR_65
void INT_VECTOR_ADDR_65(void);
#endif
#ifdef INT_VECTOR_ADDR_66
void INT_VECTOR_ADDR_66(void);
#endif
#ifdef INT_VECTOR_ADDR_67
void INT_VECTOR_ADDR_67(void);
#endif
#ifdef INT_VECTOR_ADDR_68
void INT_VECTOR_ADDR_68(void);
#endif
#ifdef INT_VECTOR_ADDR_69
void INT_VECTOR_ADDR_69(void);
#endif
#ifdef INT_VECTOR_ADDR_70
void INT_VECTOR_ADDR_70(void);
#endif
#ifdef INT_VECTOR_ADDR_71
void INT_VECTOR_ADDR_71(void);
#endif
#ifdef INT_VECTOR_ADDR_72
void INT_VECTOR_ADDR_72(void);
#endif
#ifdef INT_VECTOR_ADDR_73
void INT_VECTOR_ADDR_73(void);
#endif
#ifdef INT_VECTOR_ADDR_74
void INT_VECTOR_ADDR_74(void);
#endif
#ifdef INT_VECTOR_ADDR_75
void INT_VECTOR_ADDR_75(void);
#endif
#ifdef INT_VECTOR_ADDR_76
void INT_VECTOR_ADDR_76(void);
#endif
#ifdef INT_VECTOR_ADDR_77
void INT_VECTOR_ADDR_77(void);
#endif
#ifdef INT_VECTOR_ADDR_78
void INT_VECTOR_ADDR_78(void);
#endif
#ifdef INT_VECTOR_ADDR_79
void INT_VECTOR_ADDR_79(void);
#endif
#ifdef INT_VECTOR_ADDR_80
void INT_VECTOR_ADDR_80(void);
#endif
#ifdef INT_VECTOR_ADDR_81
void INT_VECTOR_ADDR_81(void);
#endif
#ifdef INT_VECTOR_ADDR_82
void INT_VECTOR_ADDR_82(void);
#endif
#ifdef INT_VECTOR_ADDR_83
void INT_VECTOR_ADDR_83(void);
#endif
#ifdef INT_VECTOR_ADDR_84
void INT_VECTOR_ADDR_84(void);
#endif
#ifdef INT_VECTOR_ADDR_85
void INT_VECTOR_ADDR_85(void);
#endif
#ifdef INT_VECTOR_ADDR_86
void INT_VECTOR_ADDR_86(void);
#endif
#ifdef INT_VECTOR_ADDR_87
void INT_VECTOR_ADDR_87(void);
#endif
#ifdef INT_VECTOR_ADDR_88
void INT_VECTOR_ADDR_88(void);
#endif
#ifdef INT_VECTOR_ADDR_89
void INT_VECTOR_ADDR_89(void);
#endif
#ifdef INT_VECTOR_ADDR_90
void INT_VECTOR_ADDR_90(void);
#endif
#ifdef INT_VECTOR_ADDR_91
void INT_VECTOR_ADDR_91(void);
#endif
#ifdef INT_VECTOR_ADDR_92
void INT_VECTOR_ADDR_92(void);
#endif
#ifdef INT_VECTOR_ADDR_93
void INT_VECTOR_ADDR_93(void);
#endif
#ifdef INT_VECTOR_ADDR_94
void INT_VECTOR_ADDR_94(void);
#endif
#ifdef INT_VECTOR_ADDR_95
void INT_VECTOR_ADDR_95(void);
#endif
#ifdef INT_VECTOR_ADDR_96
void INT_VECTOR_ADDR_96(void);
#endif
#ifdef INT_VECTOR_ADDR_97
void INT_VECTOR_ADDR_97(void);
#endif
#ifdef INT_VECTOR_ADDR_98
void INT_VECTOR_ADDR_98(void);
#endif
#ifdef INT_VECTOR_ADDR_99
void INT_VECTOR_ADDR_99(void);
#endif
#ifdef INT_VECTOR_ADDR_100
void INT_VECTOR_ADDR_100(void);
#endif
#ifdef INT_VECTOR_ADDR_101
void INT_VECTOR_ADDR_101(void);
#endif
#ifdef INT_VECTOR_ADDR_102
void INT_VECTOR_ADDR_102(void);
#endif
#ifdef INT_VECTOR_ADDR_103
void INT_VECTOR_ADDR_103(void);
#endif
#ifdef INT_VECTOR_ADDR_104
void INT_VECTOR_ADDR_104(void);
#endif
#ifdef INT_VECTOR_ADDR_105
void INT_VECTOR_ADDR_105(void);
#endif
#ifdef INT_VECTOR_ADDR_106
void INT_VECTOR_ADDR_106(void);
#endif
#ifdef INT_VECTOR_ADDR_107
void INT_VECTOR_ADDR_107(void);
#endif
#ifdef INT_VECTOR_ADDR_108
void INT_VECTOR_ADDR_108(void);
#endif
#ifdef INT_VECTOR_ADDR_109
void INT_VECTOR_ADDR_109(void);
#endif
#ifdef INT_VECTOR_ADDR_110
void INT_VECTOR_ADDR_110(void);
#endif



/* unhandled exception "handler" */

static asm void unhandled_interrupt(void)
{
	/* retrieve VAB field from ICTL register */
	move.l  #ArchIO.Intc.ictl,R0
	move.w  X:(R0),X0
	asrr.w  #6,X0
	bfclr   #0xff80,X0

	/* Interrupt number is stored in X0 register */	
	debughlt
	
	/* Loop forever */
	bra *+0
}

/* illegal instruction interrupt ($04) */

#ifndef INT_VECTOR_ADDR_2
#define INT_VECTOR_ADDR_2 ill_op

static asm void ill_op(void)
{
	debughlt	
	nop
	rti
}

#endif

/* hardware stack overflow interrupt ($08) */

#ifndef INT_VECTOR_ADDR_4
#define INT_VECTOR_ADDR_4 hws_overflow

static asm void hws_overflow(void)
{
	debughlt	
	nop
	rti
}

#endif
	
/* misaligned long word access interrupt ($0A) */

#ifndef INT_VECTOR_ADDR_5
#define INT_VECTOR_ADDR_5 misalign

static asm void misalign(void)
{
	debughlt	
	nop
	rti
}

#endif

/* PLL lost of lock interrupt ($20) */

#ifndef INT_VECTOR_ADDR_15
#define INT_VECTOR_ADDR_15 pll_losslock

static asm void pll_losslock(void)
{
	debughlt
	nop
	rti
}

#endif

/***************************
 *
 * Interrupt vectors table
 *
 ***************************/

#define boot_start		0x0000F400

#pragma define_section interrupt_vectors "interrupt_vectors.text"  RX
#pragma section interrupt_vectors begin

static asm void int_vec(void)
{
	jmp >Start;           	/* Reset */
	//jmp boot_start;
	
#ifdef INT_VECTOR_ADDR_1	
	jmp >INT_VECTOR_ADDR_1	/* 0x02: COP Reset */							
#else
	jmp >Start;
	//jmp boot_start;
#endif
	
#ifdef INT_VECTOR_ADDR_2
	jsr >INT_VECTOR_ADDR_2
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_3
	jsr >INT_VECTOR_ADDR_3
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_4
	jsr >INT_VECTOR_ADDR_4
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_5
	jsr >INT_VECTOR_ADDR_5
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_6
	jsr >INT_VECTOR_ADDR_6
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_7
	jsr >INT_VECTOR_ADDR_7
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_8
	jsr >INT_VECTOR_ADDR_8
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_9
	jsr >INT_VECTOR_ADDR_9
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_10
	jsr >INT_VECTOR_ADDR_10
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_11
	jsr >INT_VECTOR_ADDR_11
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_12
	jsr >INT_VECTOR_ADDR_12
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_13
	jsr >INT_VECTOR_ADDR_13
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_14
	jsr >INT_VECTOR_ADDR_14
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_15
	jsr >INT_VECTOR_ADDR_15
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_16
	jsr >INT_VECTOR_ADDR_16
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_17
	jsr >INT_VECTOR_ADDR_17
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_18
	jsr >INT_VECTOR_ADDR_18
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_19
	jsr >INT_VECTOR_ADDR_19
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_20
	jsr >INT_VECTOR_ADDR_20
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_21
	jsr >INT_VECTOR_ADDR_21
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_22
	jsr >INT_VECTOR_ADDR_22
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_23
	jsr >INT_VECTOR_ADDR_23
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_24
	jsr >INT_VECTOR_ADDR_24
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_25
	jsr >INT_VECTOR_ADDR_25
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_26
	jsr >INT_VECTOR_ADDR_26
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_27
	jsr >INT_VECTOR_ADDR_27
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_28
	jsr >INT_VECTOR_ADDR_28
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_29
	jsr >INT_VECTOR_ADDR_29
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_30
	jsr >INT_VECTOR_ADDR_30
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_31
	jsr >INT_VECTOR_ADDR_31
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_32
	jsr >INT_VECTOR_ADDR_32
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_33
	jsr >INT_VECTOR_ADDR_33
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_34
	jsr >INT_VECTOR_ADDR_34
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_35
	jsr >INT_VECTOR_ADDR_35
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_36
	jsr >INT_VECTOR_ADDR_36
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_37
	jsr >INT_VECTOR_ADDR_37
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_38
	jsr >INT_VECTOR_ADDR_38
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_39
	jsr >INT_VECTOR_ADDR_39
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_40
	jsr >INT_VECTOR_ADDR_40
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_41
	jsr >INT_VECTOR_ADDR_41
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_42
	jsr >INT_VECTOR_ADDR_42
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_43
	jsr >INT_VECTOR_ADDR_43
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_44
	jsr >INT_VECTOR_ADDR_44
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_45
	jsr >INT_VECTOR_ADDR_45
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_46
	jsr >INT_VECTOR_ADDR_46
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_47
	jsr >INT_VECTOR_ADDR_47
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_48
	jsr >INT_VECTOR_ADDR_48
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_49
	jsr >INT_VECTOR_ADDR_49
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_50
	jsr >INT_VECTOR_ADDR_50
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_51
	jsr >INT_VECTOR_ADDR_51
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_52
	jsr >INT_VECTOR_ADDR_52
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_53
	jsr >INT_VECTOR_ADDR_53
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_54
	jsr >INT_VECTOR_ADDR_54
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_55
	jsr >INT_VECTOR_ADDR_55
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_56
	jsr >INT_VECTOR_ADDR_56
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_57
	jsr >INT_VECTOR_ADDR_57
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_58
	jsr >INT_VECTOR_ADDR_58
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_59
	jsr >INT_VECTOR_ADDR_59
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_60
	jsr >INT_VECTOR_ADDR_60
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_61
	jsr >INT_VECTOR_ADDR_61
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_62
	jsr >INT_VECTOR_ADDR_62
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_63
	jsr >INT_VECTOR_ADDR_63
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_64
	jsr >INT_VECTOR_ADDR_64
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_65
	jsr >INT_VECTOR_ADDR_65
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_66
	jsr >INT_VECTOR_ADDR_66
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_67
	jsr >INT_VECTOR_ADDR_67
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_68
	jsr >INT_VECTOR_ADDR_68
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_69
	jsr >INT_VECTOR_ADDR_69
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_70
	jsr >INT_VECTOR_ADDR_70
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_71
	jsr >INT_VECTOR_ADDR_71
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_72
	jsr >INT_VECTOR_ADDR_72
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_73
	jsr >INT_VECTOR_ADDR_73
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_74
	jsr >INT_VECTOR_ADDR_74
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_75
	jsr >INT_VECTOR_ADDR_75
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_76
	jsr >INT_VECTOR_ADDR_76
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_77
	jsr >INT_VECTOR_ADDR_77
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_78
	jsr >INT_VECTOR_ADDR_78
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_79
	jsr >INT_VECTOR_ADDR_79
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_80
	jsr >INT_VECTOR_ADDR_80
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_81
	jsr >INT_VECTOR_ADDR_81
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_82
	jsr >INT_VECTOR_ADDR_82
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_83
	jsr >INT_VECTOR_ADDR_83
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_84
	jsr >INT_VECTOR_ADDR_84
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_85
	jsr >INT_VECTOR_ADDR_85
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_86
	jsr >INT_VECTOR_ADDR_86
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_87
	jsr >INT_VECTOR_ADDR_87
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_88
	jsr >INT_VECTOR_ADDR_88
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_89
	jsr >INT_VECTOR_ADDR_89
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_90
	jsr >INT_VECTOR_ADDR_90
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_91
	jsr >INT_VECTOR_ADDR_91
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_92
	jsr >INT_VECTOR_ADDR_92
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_93
	jsr >INT_VECTOR_ADDR_93
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_94
	jsr >INT_VECTOR_ADDR_94
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_95
	jsr >INT_VECTOR_ADDR_95
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_96
	jsr >INT_VECTOR_ADDR_96
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_97
	jsr >INT_VECTOR_ADDR_97
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_98
	jsr >INT_VECTOR_ADDR_98
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_99
	jsr >INT_VECTOR_ADDR_99
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_100
	jsr >INT_VECTOR_ADDR_100
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_101
	jsr >INT_VECTOR_ADDR_101
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_102
	jsr >INT_VECTOR_ADDR_102
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_103
	jsr >INT_VECTOR_ADDR_103
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_104
	jsr >INT_VECTOR_ADDR_104
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_105
	jsr >INT_VECTOR_ADDR_105
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_106
	jsr >INT_VECTOR_ADDR_106
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_107
	jsr >INT_VECTOR_ADDR_107
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_108
	jsr >INT_VECTOR_ADDR_108
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_109
	jsr >INT_VECTOR_ADDR_109
#else
	jsr >unhandled_interrupt
#endif
#ifdef INT_VECTOR_ADDR_110
	jsr >INT_VECTOR_ADDR_110
#else
	jsr >unhandled_interrupt
#endif


}

#pragma section interrupt_vectors end

#ifdef __cplusplus
}
#endif

