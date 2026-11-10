/* GPL-2.0-or-later; register definitions copied from Rockbox firmware/export/s5l87xx.h.
 * Names are retained for source compatibility; see the M2 audit for observed meanings. */
#define REG32_PTR_T volatile uint32_t *
#define FMC_BASE 0x38a00000u
#define FMCTRL0                 (*(REG32_PTR_T)(FMC_BASE + 0x0000))     /* Control Register0 */
#define FMCTRL1                 (*(REG32_PTR_T)(FMC_BASE + 0x0004))     /* Control Register1 */
#define FMCMD                   (*(REG32_PTR_T)(FMC_BASE + 0x0008))     /* Command Register */
#define FMADDR0                 (*(REG32_PTR_T)(FMC_BASE + 0x000C))     /* Address Register0 */
#define FMADDR1                 (*(REG32_PTR_T)(FMC_BASE + 0x0010))     /* Address Register1 */
#define FMADDR2                 (*(REG32_PTR_T)(FMC_BASE + 0x0014))     /* Address Register2 */
#define FMADDR3                 (*(REG32_PTR_T)(FMC_BASE + 0x0018))     /* Address Register3 */
#define FMADDR4                 (*(REG32_PTR_T)(FMC_BASE + 0x001C))     /* Address Register4 */
#define FMADDR5                 (*(REG32_PTR_T)(FMC_BASE + 0x0020))     /* Address Register5 */
#define FMADDR6                 (*(REG32_PTR_T)(FMC_BASE + 0x0024))     /* Address Register6 */
#define FMADDR7                 (*(REG32_PTR_T)(FMC_BASE + 0x0028))     /* Address Register7 */
#define FMANUM                  (*(REG32_PTR_T)(FMC_BASE + 0x002C))     /* Address Counter Register */
#define FMDNUM                  (*(REG32_PTR_T)(FMC_BASE + 0x0030))     /* Data Counter Register */
#define FMDATAW0                (*(REG32_PTR_T)(FMC_BASE + 0x0034))     /* Write Data Register0 */
#define FMDATAW1                (*(REG32_PTR_T)(FMC_BASE + 0x0038))     /* Write Data Register1 */
#define FMDATAW2                (*(REG32_PTR_T)(FMC_BASE + 0x003C))     /* Write Data Register2 */
#define FMDATAW3                (*(REG32_PTR_T)(FMC_BASE + 0x0040))     /* Write Data Register3 */
#define FMCSTAT                 (*(REG32_PTR_T)(FMC_BASE + 0x0048))     /* Status Register */
#define FMSYND0                 (*(REG32_PTR_T)(FMC_BASE + 0x004C))     /* Hamming Syndrome0 */
#define FMSYND1                 (*(REG32_PTR_T)(FMC_BASE + 0x0050))     /* Hamming Syndrome1 */
#define FMSYND2                 (*(REG32_PTR_T)(FMC_BASE + 0x0054))     /* Hamming Syndrome2 */
#define FMSYND3                 (*(REG32_PTR_T)(FMC_BASE + 0x0058))     /* Hamming Syndrome3 */
#define FMSYND4                 (*(REG32_PTR_T)(FMC_BASE + 0x005C))     /* Hamming Syndrome4 */
#define FMSYND5                 (*(REG32_PTR_T)(FMC_BASE + 0x0060))     /* Hamming Syndrome5 */
#define FMSYND6                 (*(REG32_PTR_T)(FMC_BASE + 0x0064))     /* Hamming Syndrome6 */
#define FMSYND7                 (*(REG32_PTR_T)(FMC_BASE + 0x0068))     /* Hamming Syndrome7 */
#define FMFIFO                  (*(REG32_PTR_T)(FMC_BASE + 0x0080))     /* WRITE/READ FIFO FIXME */
#define FMCTRL0_CE(bank)        (1 << ((bank) + 1))  /* chip enable bits 1-8 */
#define FMCTRL0_CE_MASK         (0xff << 1)
#define FMCTRL0_ENABLEDMA       (1 << 10)
#define FMCTRL0_UNK1            (1 << 11)
#define FMCTRL0_AUTOXFER        (1 << 24)       /* S5L8702: write the chunk to the address in FMDATAW0 */
#define FMCTRL1_DOTRANSADDR     (1 << 0)
#define FMCTRL1_DOREADDATA      (1 << 1)
#define FMCTRL1_DOWRITEDATA     (1 << 2)
#define FMCTRL1_CLEARWFIFO      (1 << 6)
#define FMCTRL1_CLEARRFIFO      (1 << 7)
#define FMCSTAT_RBBDONE         (1 << 0)
#define FMCSTAT_CMDDONE         (1 << 1)
#define FMCSTAT_ADDRDONE        (1 << 2)
#define FMCSTAT_TRANSDONE       (1 << 3)
#define FMCSTAT_BANK0READY      (1 << 4)
#define FMCSTAT_BANK1READY      (1 << 5)
#define FMCSTAT_BANK2READY      (1 << 6)
#define FMCSTAT_BANK3READY      (1 << 7)
#define FMCSTAT_UNK20           (1 << 20)       /* S5L8702: chunk written to memory */
#define FMCSTAT_STATUSREADY     (1 << 23)       /* S5L8702: set after READ STATUS; not a ready bit */
#define FMCSTAT_UNK27           (1 << 27)       /* S5L8702: chunk has ECC errors (inferred) */
