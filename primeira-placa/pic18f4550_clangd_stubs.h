/*
 * clangd_stubs.h - v2 (Complete)
 *
 * This file contains mock 'structs' for the PIC18F4550's '...bits' registers
 * so that clangd (or any standard C language server) can understand the
 * Object.Member syntax (e.g., TRISBbits.TRISB0) without errors.
 *
 * This file is ONLY for the language server. The real XC8 compiler
 * will use its own official headers to generate the correct code.
 *
 * Generated based on pic18f4550.h from 05/12/2024.
 */

#ifndef __CLANGD_STUBS_H__
#define __CLANGD_STUBS_H__

// Default crystal frequency for delay functions
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 20000000
#endif

// Prototypes for common compiler built-ins
void __delay_ms(unsigned long);
void __delay_us(unsigned long);

/*=========================================================================
 * SFRs (SPECIAL FUNCTION REGISTERS)
 *=========================================================================
 */

// --- USB Registers ---

extern volatile union {
  struct {
    unsigned FRM : 3;
  };
  struct {
    unsigned FRM8 : 1;
    unsigned FRM9 : 1;
    unsigned FRM10 : 1;
  };
} UFRMHbits;

extern volatile union {
  struct {
    unsigned URSTIF : 1;
    unsigned UERRIF : 1;
    unsigned ACTVIF : 1;
    unsigned TRNIF : 1;
    unsigned IDLEIF : 1;
    unsigned STALLIF : 1;
    unsigned SOFIF : 1;
  };
} UIRbits;

extern volatile union {
  struct {
    unsigned URSTIE : 1;
    unsigned UERRIE : 1;
    unsigned ACTVIE : 1;
    unsigned TRNIE : 1;
    unsigned IDLEIE : 1;
    unsigned STALLIE : 1;
    unsigned SOFIE : 1;
  };
} UIEbits;

extern volatile union {
  struct {
    unsigned PIDEF : 1;
    unsigned CRC5EF : 1;
    unsigned CRC16EF : 1;
    unsigned DFN8EF : 1;
    unsigned BTOEF : 1;
    unsigned : 2;
    unsigned BTSEF : 1;
  };
} UEIRbits;

extern volatile union {
  struct {
    unsigned PIDEE : 1;
    unsigned CRC5EE : 1;
    unsigned CRC16EE : 1;
    unsigned DFN8EE : 1;
    unsigned BTOEE : 1;
    unsigned : 2;
    unsigned BTSEE : 1;
  };
} UEIEbits;

extern volatile union {
  struct {
    unsigned : 1;
    unsigned PPBI : 1;
    unsigned DIR : 1;
    unsigned ENDP : 4;
  };
  struct {
    unsigned : 3;
    unsigned ENDP0 : 1;
    unsigned ENDP1 : 1;
    unsigned ENDP2 : 1;
    unsigned ENDP3 : 1;
  };
} USTATbits;

extern volatile union {
  struct {
    unsigned : 1;
    unsigned SUSPND : 1;
    unsigned RESUME : 1;
    unsigned USBEN : 1;
    unsigned PKTDIS : 1;
    unsigned SE0 : 1;
    unsigned PPBRST : 1;
  };
} UCONbits;

extern volatile union {
  struct {
    unsigned ADDR : 7;
  };
  struct {
    unsigned ADDR0 : 1;
    unsigned ADDR1 : 1;
    unsigned ADDR2 : 1;
    unsigned ADDR3 : 1;
    unsigned ADDR4 : 1;
    unsigned ADDR5 : 1;
    unsigned ADDR6 : 1;
  };
} UADDRbits;

extern volatile union {
  struct {
    unsigned PPB : 2;
    unsigned FSEN : 1;
    unsigned UTRDIS : 1;
    unsigned UPUEN : 1;
    unsigned : 1;
    unsigned UOEMON : 1;
    unsigned UTEYE : 1;
  };
  struct {
    unsigned PPB0 : 1;
    unsigned PPB1 : 1;
  };
} UCFGbits;

extern volatile union {
  struct {
    unsigned EPSTALL : 1;
    unsigned EPINEN : 1;
    unsigned EPOUTEN : 1;
    unsigned EPCONDIS : 1;
    unsigned EPHSHK : 1;
  };
} UEP0bits, UEP1bits, UEP2bits, UEP3bits, UEP4bits, UEP5bits, UEP6bits,
    UEP7bits, UEP8bits, UEP9bits, UEP10bits, UEP11bits, UEP12bits, UEP13bits,
    UEP14bits, UEP15bits;

// --- PORT & LATCH & TRIS ---

extern volatile union {
  struct {
    unsigned RA0 : 1;
    unsigned RA1 : 1;
    unsigned RA2 : 1;
    unsigned RA3 : 1;
    unsigned RA4 : 1;
    unsigned RA5 : 1;
    unsigned RA6 : 1;
  };
  struct {
    unsigned AN0 : 1;
    unsigned AN1 : 1;
    unsigned AN2 : 1;
    unsigned AN3 : 1;
    unsigned T0CKI : 1;
    unsigned AN4 : 1;
    unsigned OSC2 : 1;
  };
} PORTAbits;

extern volatile union {
  struct {
    unsigned RB0 : 1;
    unsigned RB1 : 1;
    unsigned RB2 : 1;
    unsigned RB3 : 1;
    unsigned RB4 : 1;
    unsigned RB5 : 1;
    unsigned RB6 : 1;
    unsigned RB7 : 1;
  };
  struct {
    unsigned INT0 : 1;
    unsigned INT1 : 1;
    unsigned INT2 : 1;
  };
} PORTBbits;

extern volatile union {
  struct {
    unsigned RC0 : 1;
    unsigned RC1 : 1;
    unsigned RC2 : 1;
    unsigned : 1;
    unsigned RC4 : 1;
    unsigned RC5 : 1;
    unsigned RC6 : 1;
    unsigned RC7 : 1;
  };
} PORTCbits;

extern volatile union {
  struct {
    unsigned RD0 : 1;
    unsigned RD1 : 1;
    unsigned RD2 : 1;
    unsigned RD3 : 1;
    unsigned RD4 : 1;
    unsigned RD5 : 1;
    unsigned RD6 : 1;
    unsigned RD7 : 1;
  };
} PORTDbits;

extern volatile union {
  struct {
    unsigned RE0 : 1;
    unsigned RE1 : 1;
    unsigned RE2 : 1;
    unsigned RE3 : 1;
    unsigned : 3;
    unsigned RDPU : 1;
  };
} PORTEbits;

extern volatile union {
  struct {
    unsigned LATA0 : 1, LATA1 : 1, LATA2 : 1, LATA3 : 1, LATA4 : 1, LATA5 : 1,
        LATA6 : 1;
  };
} LATAbits;
extern volatile union {
  struct {
    unsigned LATB0 : 1, LATB1 : 1, LATB2 : 1, LATB3 : 1, LATB4 : 1, LATB5 : 1,
        LATB6 : 1, LATB7 : 1;
  };
} LATBbits;
extern volatile union {
  struct {
    unsigned LATC0 : 1, LATC1 : 1, LATC2 : 1, : 3, LATC6 : 1, LATC7 : 1;
  };
} LATCbits;
extern volatile union {
  struct {
    unsigned LATD0 : 1, LATD1 : 1, LATD2 : 1, LATD3 : 1, LATD4 : 1, LATD5 : 1,
        LATD6 : 1, LATD7 : 1;
  };
} LATDbits;
extern volatile union {
  struct {
    unsigned LATE0 : 1, LATE1 : 1, LATE2 : 1;
  };
} LATEbits;

extern volatile union {
  struct {
    unsigned TRISA0 : 1, TRISA1 : 1, TRISA2 : 1, TRISA3 : 1, TRISA4 : 1,
        TRISA5 : 1, TRISA6 : 1;
  };
} TRISAbits;
extern volatile union {
  struct {
    unsigned TRISB0 : 1, TRISB1 : 1, TRISB2 : 1, TRISB3 : 1, TRISB4 : 1,
        TRISB5 : 1, TRISB6 : 1, TRISB7 : 1;
  };
} TRISBbits;
extern volatile union {
  struct {
    unsigned TRISC0 : 1, TRISC1 : 1, TRISC2 : 1, : 3, TRISC6 : 1, TRISC7 : 1;
  };
} TRISCbits;
extern volatile union {
  struct {
    unsigned TRISD0 : 1, TRISD1 : 1, TRISD2 : 1, TRISD3 : 1, TRISD4 : 1,
        TRISD5 : 1, TRISD6 : 1, TRISD7 : 1;
  };
} TRISDbits;
extern volatile union {
  struct {
    unsigned TRISE0 : 1, TRISE1 : 1, TRISE2 : 1;
  };
} TRISEbits;

// --- INTERRUPT & CORE REGISTERS ---

extern volatile union {
  struct {
    unsigned nBOR : 1;
    unsigned nPOR : 1;
    unsigned nPD : 1;
    unsigned nTO : 1;
    unsigned nRI : 1;
    unsigned : 1;
    unsigned SBOREN : 1;
    unsigned IPEN : 1;
  };
  struct {
    unsigned BOR : 1;
    unsigned POR : 1;
    unsigned PD : 1;
    unsigned TO : 1;
    unsigned RI : 1;
    unsigned : 2;
    unsigned nIPEN : 1;
  };
} RCONbits;

extern volatile union {
  struct {
    unsigned SWDTEN : 1;
  };
  struct {
    unsigned SWDTE : 1;
  };
} WDTCONbits;

extern volatile union {
  struct {
    unsigned HLVDL : 4;
    unsigned HLVDEN : 1;
    unsigned IRVST : 1;
    unsigned : 1;
    unsigned VDIRMAG : 1;
  };
  struct {
    unsigned LVDL : 4;
    unsigned LVDEN : 1;
    unsigned IVRST : 1;
  };
  struct {
    unsigned LVV : 4;
  };
} HLVDCONbits, LVDCONbits;

extern volatile union {
  struct {
    unsigned RBIF : 1;
    unsigned INT0IF : 1;
    unsigned TMR0IF : 1;
    unsigned RBIE : 1;
    unsigned INT0IE : 1;
    unsigned TMR0IE : 1;
    unsigned PEIE_GIEL : 1;
    unsigned GIE_GIEH : 1;
  };
  struct {
    unsigned : 1;
    unsigned INT0F : 1;
    unsigned T0IF : 1;
    unsigned : 1;
    unsigned INT0E : 1;
    unsigned T0IE : 1;
    unsigned PEIE : 1;
    unsigned GIE : 1;
  };
} INTCONbits;

extern volatile union {
  struct {
    unsigned RBIP : 1;
    unsigned : 1;
    unsigned TMR0IP : 1;
    unsigned : 1;
    unsigned INTEDG2 : 1;
    unsigned INTEDG1 : 1;
    unsigned INTEDG0 : 1;
    unsigned nRBPU : 1;
  };
  struct {
    unsigned : 2;
    unsigned T0IP : 1;
    unsigned : 4;
    unsigned RBPU : 1;
  };
} INTCON2bits;

extern volatile union {
  struct {
    unsigned INT1IF : 1;
    unsigned INT2IF : 1;
    unsigned : 1;
    unsigned INT1IE : 1;
    unsigned INT2IE : 1;
    unsigned : 1;
    unsigned INT1IP : 1;
    unsigned INT2IP : 1;
  };
  struct {
    unsigned INT1F : 1;
    unsigned INT2F : 1;
    unsigned : 1;
    unsigned INT1E : 1;
    unsigned INT2E : 1;
    unsigned : 1;
    unsigned INT1P : 1;
    unsigned INT2P : 1;
  };
} INTCON3bits;

extern volatile union {
  struct {
    unsigned TMR1IE : 1;
    unsigned TMR2IE : 1;
    unsigned CCP1IE : 1;
    unsigned SSPIE : 1;
    unsigned TXIE : 1;
    unsigned RCIE : 1;
    unsigned ADIE : 1;
    unsigned SPPIE : 1;
  };
  struct {
    unsigned : 7;
    unsigned PSPIE : 1;
  };
} PIE1bits;

extern volatile union {
  struct {
    unsigned TMR1IF : 1;
    unsigned TMR2IF : 1;
    unsigned CCP1IF : 1;
    unsigned SSPIF : 1;
    unsigned TXIF : 1;
    unsigned RCIF : 1;
    unsigned ADIF : 1;
    unsigned SPPIF : 1;
  };
  struct {
    unsigned : 7;
    unsigned PSPIF : 1;
  };
} PIR1bits;

extern volatile union {
  struct {
    unsigned TMR1IP : 1;
    unsigned TMR2IP : 1;
    unsigned CCP1IP : 1;
    unsigned SSPIP : 1;
    unsigned TXIP : 1;
    unsigned RCIP : 1;
    unsigned ADIP : 1;
    unsigned SPPIP : 1;
  };
  struct {
    unsigned : 7;
    unsigned PSPIP : 1;
  };
} IPR1bits;

extern volatile union {
  struct {
    unsigned CCP2IE : 1;
    unsigned TMR3IE : 1;
    unsigned HLVDIE : 1;
    unsigned BCLIE : 1;
    unsigned EEIE : 1;
    unsigned USBIE : 1;
    unsigned CMIE : 1;
    unsigned OSCFIE : 1;
  };
  struct {
    unsigned : 2;
    unsigned LVDIE : 1;
  };
} PIE2bits;

extern volatile union {
  struct {
    unsigned CCP2IF : 1;
    unsigned TMR3IF : 1;
    unsigned HLVDIF : 1;
    unsigned BCLIF : 1;
    unsigned EEIF : 1;
    unsigned USBIF : 1;
    unsigned CMIF : 1;
    unsigned OSCFIF : 1;
  };
  struct {
    unsigned : 2;
    unsigned LVDIF : 1;
  };
} PIR2bits;

extern volatile union {
  struct {
    unsigned CCP2IP : 1;
    unsigned TMR3IP : 1;
    unsigned HLVDIP : 1;
    unsigned BCLIP : 1;
    unsigned EEIP : 1;
    unsigned USBIP : 1;
    unsigned CMIP : 1;
    unsigned OSCFIP : 1;
  };
  struct {
    unsigned : 2;
    unsigned LVDIP : 1;
  };
} IPR2bits;

// --- TIMERS & CCP & ADC & OTHER PERIPHERALS ---

extern volatile union {
  struct {
    unsigned T0PS : 3, PSA : 1, T0SE : 1, T0CS : 1, T08BIT : 1, TMR0ON : 1;
  };
} T0CONbits;
extern volatile union {
  struct {
    unsigned TMR1ON : 1, TMR1CS : 1, nT1SYNC : 1, T1OSCEN : 1, T1CKPS : 2,
        T1RUN : 1, RD16 : 1;
  };
} T1CONbits;
extern volatile union {
  struct {
    unsigned T2CKPS : 2, TMR2ON : 1, TOUTPS : 4;
  };
} T2CONbits;
extern volatile union {
  struct {
    unsigned TMR3ON : 1, TMR3CS : 1, nT3SYNC : 1, T3CCP1 : 1, T3CKPS : 2,
        T3CCP2 : 1, RD16 : 1;
  };
} T3CONbits;

extern volatile union {
  struct {
    unsigned ADCS : 3;
    unsigned ACQT : 3;
    unsigned : 1;
    unsigned ADFM : 1;
  };
} ADCON2bits;

extern volatile union {
  struct {
    unsigned PCFG : 4;
    unsigned VCFG : 2;
  };
} ADCON1bits;

extern volatile union {
  struct {
    unsigned ADON : 1;
    unsigned GO_nDONE : 1;
    unsigned CHS : 4;
  };
  struct {
    unsigned : 1;
    unsigned GO : 1;
  };
  struct {
    unsigned : 1;
    unsigned DONE : 1;
  };
} ADCON0bits;

extern volatile union {
  struct {
    unsigned CCP1M : 4;
    unsigned DC1B : 2;
    unsigned P1M : 2;
  };
} CCP1CONbits, ECCP1CONbits;

extern volatile union {
  struct {
    unsigned CCP2M : 4;
    unsigned DC2B : 2;
  };
} CCP2CONbits;

extern volatile union {
  struct {
    unsigned PSSBD : 2;
    unsigned PSSAC : 2;
    unsigned ECCPAS : 3;
    unsigned ECCPASE : 1;
  };
} ECCP1ASbits, CCP1ASbits;

extern volatile union {
  struct {
    unsigned PDC : 7;
    unsigned PRSEN : 1;
  };
} ECCP1DELbits, CCP1DELbits;

extern volatile union {
  struct {
    unsigned RX9D : 1;
    unsigned OERR : 1;
    unsigned FERR : 1;
    unsigned ADDEN : 1;
    unsigned CREN : 1;
    unsigned SREN : 1;
    unsigned RX9 : 1;
    unsigned SPEN : 1;
  };
} RCSTAbits, RCSTA1bits;

extern volatile union {
  struct {
    unsigned TX9D : 1;
    unsigned TRMT : 1;
    unsigned BRGH : 1;
    unsigned SENDB : 1;
    unsigned SYNC : 1;
    unsigned TXEN : 1;
    unsigned TX9 : 1;
    unsigned CSRC : 1;
  };
} TXSTAbits, TXSTA1bits;

extern volatile union {
  struct {
    unsigned ABDEN : 1;
    unsigned WUE : 1;
    unsigned : 1;
    unsigned BRG16 : 1;
    unsigned TXCKP : 1;
    unsigned RXDTP : 1;
    unsigned RCIDL : 1;
    unsigned ABDOVF : 1;
  };
  struct {
    unsigned : 4;
    unsigned SCKP : 1;
    unsigned : 1;
    unsigned RCMT : 1;
  };
} BAUDCONbits, BAUDCTLbits;

extern volatile union {
  struct {
    unsigned CM : 3;
    unsigned CIS : 1;
    unsigned C1INV : 1;
    unsigned C2INV : 1;
    unsigned C1OUT : 1;
    unsigned C2OUT : 1;
  };
} CMCONbits;

extern volatile union {
  struct {
    unsigned CVR : 4;
    unsigned CVRSS : 1;
    unsigned CVRR : 1;
    unsigned CVROE : 1;
    unsigned CVREN : 1;
  };
} CVRCONbits;

extern volatile union {
  struct {
    unsigned SSPM : 4;
    unsigned CKP : 1;
    unsigned SSPEN : 1;
    unsigned SSPOV : 1;
    unsigned WCOL : 1;
  };
} SSPCON1bits;

extern volatile union {
  struct {
    unsigned SEN : 1;
    unsigned RSEN : 1;
    unsigned PEN : 1;
    unsigned RCEN : 1;
    unsigned ACKEN : 1;
    unsigned ACKDT : 1;
    unsigned ACKSTAT : 1;
    unsigned GCEN : 1;
  };
} SSPCON2bits;

extern volatile union {
  struct {
    unsigned BF : 1;
    unsigned UA : 1;
    unsigned R_nW : 1;
    unsigned S : 1;
    unsigned P : 1;
    unsigned D_nA : 1;
    unsigned CKE : 1;
    unsigned SMP : 1;
  };
  struct {
    unsigned : 2;
    unsigned I2C_READ : 1;
    unsigned I2C_START : 1;
    unsigned I2C_STOP : 1;
    unsigned I2C_DAT : 1;
  };
} SSPSTATbits;

extern volatile union {
  struct {
    unsigned RD : 1;
    unsigned WR : 1;
    unsigned WREN : 1;
    unsigned WRERR : 1;
    unsigned FREE : 1;
    unsigned : 1;
    unsigned CFGS : 1;
    unsigned EEPGD : 1;
  };
} EECON1bits;

extern volatile union {
  struct {
    unsigned C : 1;
    unsigned DC : 1;
    unsigned Z : 1;
    unsigned OV : 1;
    unsigned N : 1;
  };
  struct {
    unsigned CARRY : 1;
    unsigned : 1;
    unsigned ZERO : 1;
    unsigned OVERFLOW : 1;
    unsigned NEGATIVE : 1;
  };
} STATUSbits;

extern volatile union {
  struct {
    unsigned STKPTR : 5;
    unsigned : 1;
    unsigned STKUNF : 1;
    unsigned STKFUL : 1;
  };
  struct {
    unsigned : 7;
    unsigned STKOVF : 1;
  };
} STKPTRbits;

#endif // __CLANGD_STUBS_H__
