"""Verify M1 chapter 4.3 register roles on M2; predictions are not M2 facts.

All role matrices run resident. The PC-as-system-address question remains a
separate final family; --isolate-diagnostics optionally collects it per case.
"""
import struct
from .protocol import DISCOVERY
from fmss import probe as P

SEED = [0x11001111 * (i+1) for i in range(8)]
M1_REFERENCE = 'M1 register-file roles'
ALU = (0xa,0xb,0xc,0xd,0x13,0x14)


def seed(values=SEED):
    return ''.join('movi r%d,#0x%x\n'%(i,v) for i,v in enumerate(values))


def raw(op,r,a=0,imm=0):
    return '.insn 0x%x,%d,0x%x,%s\n'%(op,r,a,imm if isinstance(imm,str) else hex(imm))


def case(t,name,body,want,**options):
    from .followup_experiments import Case
    source=t.prologue(c58=0)+body+'end\n'+t.trap()
    expected={'why':'END','cs_irq':1,**want}
    return Case(name,source,dict(flags=DISCOVERY|P.WAIT_END,timeout_us=20000,**options),
                dict(mode='confirm',expected=expected,basis=M1_REFERENCE,
                     qualification='M1 prediction; only a physical M2 match qualifies the tested role/operands'))


def destinations(s,t):
    for n in range(256):
        if n==8:continue
        body='';after={}
        for index,(op,value) in enumerate(((5,0x5eedd00d),(6,SEED[1]),(0xc,(SEED[1]+1)&0xffffffff))):
            body+=seed()+raw(op,n,0 if op==5 else 1,0x5eedd00d if op==5 else 1 if op==0xc else 0)
            regs=([value]*8 if n==10 else [value if i==n else v for i,v in enumerate(SEED)])
            for i,v in enumerate(regs):
                offset=0xd00+32*index+4*i
                body+='st [0x%x],r%d\n'%(offset,i);after[hex(offset)]=v
        yield case(t,'destination/%03d'%n,body,{'after':after})
    for op in (5,6,0xc):
        body=seed()
        if op!=5:
            body+='movi r1,#landing\n'
            if op==0xc:body+='sub r1,r1,#1\n'
        body+=raw(op,8,0 if op==5 else 1,'landing' if op==5 else 1 if op==0xc else 0)
        body+='wr [0xd70],#0xbad0bad0\nend\nlanding:\nwr [0xd70],#0x600d600d\n'
        yield case(t,'PC-destination/op%02x'%op,body,{'after':{'0xd70':0x600d600d}},params=[(0xd70,0)])


def alu(op,a,b):
    return {0xa:lambda:a&b,0xb:lambda:a|b,0xc:lambda:a+b,0xd:lambda:a-b,
            0x13:lambda:(a<<b) if b<32 else 0,0x14:lambda:(a>>b) if b<32 else 0}[op]()&0xffffffff


def sources(s,t):
    roles=[(6,0)]+[(op,imm) for op in ALU for imm in (1,0)]
    for n in range(256):
        body=''
        for index,(op,imm) in enumerate(roles):
            body+=seed()+raw(op,1,n,imm)+'st [0x%x],r1\n'%(0xd00+4*index)
        c=case(t,'argument-source/%03d'%n,body,{})
        code=P.assemble(t,c.source)
        positions=[i for i,(w,v) in enumerate(struct.iter_unpack('<II',code)) if w>>24 in (6,*ALU) and (w>>16)&255==1 and w&65535==n]
        assert len(positions)==len(roles)
        after,m1={},{}
        for index,((op,imm),pos) in enumerate(zip(roles,positions)):
            for table,rule in ((after,'m2'),(m1,'m1')):
                # p16/p35: on M2 source 10 is the destination register (r1) in every
                # form; M1's rule applied it to MOV/immediate forms only.
                ten=n==10 and (rule=='m2' or op==6 or imm)
                src=(pos*8 if n==8 else SEED[1] if ten else SEED[n if n<8 else 0])
                table[hex(0xd00+4*index)]=src if op==6 else alu(op,src,imm) if imm else alu(op,SEED[1],src)
        c.conditions['expected']['after']=after
        if m1!=after:
            c.conditions['m1_prediction']={'after':m1,'basis':'M1 source 10 = destination for MOV/immediate forms only; '
                                           'M2 measured p16 and p35: every form (M2-141)'}
        yield c


def branches(s,t):
    for n in range(256):
        body='';after={}
        for i,(op,zero) in enumerate(((0xe,False),(0xe,True),(0x17,False),(0x17,True))):
            values=[0 if zero else v for v in SEED]
            body+=seed(values)+raw(op,n,0,'taken%d'%i)
            body+='wr [0x%x],#0xbad0bad0\nmovi r7,#1\nbnz r7,join%d\ntaken%d:\nwr [0x%x],#0x600d600d\njoin%d:\n'%(0xd00+4*i,i,i,0xd00+4*i,i)
            taken=n<8 and ((op==0xe and not zero) or (op==0x17 and zero))
            after[hex(0xd00+4*i)]=0x600d600d if taken else 0xbad0bad0
        yield case(t,'branch-register/%03d'%n,body,{'after':after})


def addresses(s,t):
    base=t.scratch_addr(s)
    for n in range(256):
        if n==8:continue  # separate bus applicability experiment, last
        body=seed([base+4*i for i in range(8)])+raw(3,1,n)+'st [0xd00],r1\n'
        body+=seed([0xd40+4*i for i in range(8)])+raw(0x18,1,n)+'st [0xd04],r1\n'
        index=n if n<8 else 0
        yield case(t,'address-register/%03d'%n,body,{'after':{'0xd00':0xcafe0000+index,'0xd04':0xd00d0000+index}},
            scratch=struct.pack('<8I',*(0xcafe0000+i for i in range(8))),scratch_read=32,
            params=[(0xd40+4*i,0xd00d0000+i) for i in range(8)])


def stores(s,t):
    base=t.scratch_addr(s)
    predecessors=[('seed-r7','',SEED[7]),('movi-r0','movi r0,#0xa0a0a0a0\n',0xa0a0a0a0),
        ('movi-r3','movi r3,#0x33333333\n',0x33333333),('movi-r7','movi r7,#0x77777777\n',0x77777777),
        ('add','add r2,r2,#1\n',(SEED[2]+1)&0xffffffff),('mov','mov r4,r1\n',SEED[1]),
        ('ld','ld r5,[0xd74]\n',0x5d5d5d5d),('pld','movi r0,#0xd78\n'+raw(0x18,6,0),0x6e6e6e6e),
        ('ldi','movi r0,#0x%x\nldi r1,[r0]\n'%base,0xcafe0001),
        ('wr','wr [0xd7c],#0x7c7c7c7c\n',SEED[0]),('st','st [0xd7c],r5\n',SEED[5]),
        ('wait','wait 11\n',SEED[7]),('nop',raw(8,0),SEED[0])]
    selections=[(n,predecessors[0]) for n in range(8,256)]
    selections += [(n,p) for p in predecessors for n in (9,14,200)]
    for i,(n,(label,body,value)) in enumerate(selections):
        yield case(t,'register-bus/%03d/%03d/%s'%(i,n,label),seed()+body+raw(2,n,0xd70),{'after':{'0xd70':value}},
            params=[(0xd70,0xeeeeeeee),(0xd74,0x5d5d5d5d),(0xd78,0x6e6e6e6e)],scratch=struct.pack('<I',0xcafe0001),scratch_read=4)


def jumps(s,t):
    for n in range(256):
        body=''.join('movi r%d,#land%d\n'%(i,i) for i in range(8))+raw(0xf,0,n)
        body+='wr [0xd70],#0xbad0bad0\nend\n'
        for i in range(8):body+='land%d:\nwr [0xd70],#0x%x\nend\n'%(i,0x600d0000+i)
        want={'after':{'0xd70':0x600d0000+(n if n<8 else 0)}}
        if n==8:want={'why':'TIMEOUT','cs_irq':0,'after':{'0xd70':0}}
        yield case(t,'jump-source/%03d'%n,body,want,params=[(0xd70,0)])


def source10(s,t):
    """Distinguish p11's self-source result from M1's r0 rule, including shifts.

    Each case runs MOV and both ALU forms with selectors 0 and 10. Reset all
    GPRs before every operation; save each result before the next reset.
    Destination r0 is deliberately a non-discriminating control.
    """
    pairs=((1,3),(3,1),(0,5),(5,0),(31,2),(2,31),(32,1),(0x80000001,0x55aa55aa))
    roles=[(6,0)]+[(op,imm) for op in ALU for imm in (1,0)]
    for dest in range(8):
        for k,(r0,value) in enumerate(pairs):
            body='';expected={};m1={};fields={}
            values=list(SEED);values[0]=r0;values[dest]=value
            for src in (0,10):
                for op,imm in roles:
                    off=hex(0xd00+4*len(expected))
                    body+=seed(values)+raw(op,dest,src,imm)+'st [%s],r%d\n'%(off,dest)
                    a=values[dest];b=values[0]
                    self_src=a if src==10 else b
                    old_src=a if src==10 and (op==6 or imm) else b
                    def result(v):
                        return v if op==6 else alu(op,v,imm) if imm else alu(op,a,v)
                    expected[off]=result(self_src);m1[off]=result(old_src)
                    fields[off]=dict(opcode=op,destination=dest,source=src,immediate=imm)
            c=case(t,'source10/r%d/values%02d'%(dest,k),body,{'after':expected})
            c.conditions.update(basis='p11 AND/OR/ADD/SUB result plus M1 chapter 4.3 comparison',
                qualification='p12 physical match for all eight GPR destinations and these value pairs; M1 alternative retained',
                initial_registers=values,fields=fields,alternative_m1_after=m1,
                discriminating_offsets=[off for off in expected if expected[off]!=m1[off]])
            yield c


def reset_pram(s,t):
    """Isolate reset writes instead of assigning p11's clearing to one phase.

    Seed all PRAM words after preflight, keep a bounded local loop running,
    apply each CPU write set at 50 us, then capture before normal cleanup.
    No expected clearing mask is invented for the unmeasured phases.
    """
    from .followup_experiments import Case
    stop=[(0xc00,8)]
    reset=[(0xc00,0x80000)]
    fmc=[(4,0x800),(4,0x80000000)]
    buffers=[(0xc60,1),(0xc60,0)]
    ecc=[(0x81c,1)]
    variants=[('running-control',[]),('stop',stop),('stop-abort',stop+[(0xc00,2)]),
              ('stop-buffers',stop+buffers),('stop-sequencer-reset',stop+reset),
              ('stop-fmc-reset',stop+fmc),('stop-ecc-reset',stop+ecc),
              ('full-reset',stop+buffers+reset+fmc+ecc)]
    params=[(off,0xcafe0000+off) for off in range(0xd00,0xd80,4)]
    params += [(0xc18+4*i,0x11001111*(i+1)) for i in range(8)]
    for name,writes in variants:
        yield Case(name,t.prologue(c58=0)+'loop:\nbnz r6,loop\nend\n'+t.trap(),
            dict(flags=DISCOVERY|P.WAIT_END,params=params,late=writes,late_us=50,timeout_us=500),
            dict(question='Which reset phase clears GPRs/PRAM? Compare before/after; capture precedes cleanup.',
                 basis='p11 untouched PRAM clears across the complete native reset boundary',
                 qualification='p12: STOP then CTRL=0x80000 clears GPRs/PRAM/C10; separate buffer/FMC/ECC steps preserve GPRs/PRAM; other states remain open',
                 cpu_writes=writes,initial_pram={hex(off):value for off,value in params if off>=0xd00}))


def pc_bus(s,t):
    # This measures a distinct SoC bus mapping, not an assumed M1 ROM word.
    from .followup_experiments import Case
    yield case(t,'PC-read-control',raw(6,2,8),{'r':{'2':32}})
    for op in (0x18,3):
        yield Case('PC-address/op%02x'%op,t.prologue(c58=0)+seed([t.scratch_addr(s)]*8)+raw(op,1,8)+'end\n'+t.trap(),
            dict(flags=DISCOVERY|P.WAIT_END,timeout_us=20000,scratch=bytes(32),scratch_read=32),
            dict(basis=M1_REFERENCE,question='PC used as window/system address; raw value depends on M2 bus map',
                 qualification='applicability measurement; no M1 ROM word assumed on M2'))


SPECS={
 'm2_regdest':(destinations,'regnum,r8,regsweep','M1 destination roles: all 256 selectors, MOVI/MOV/ADD, local PC targets'),
 'm2_regsrc':(sources,'regsweep','M1 argument sources: all 256 selectors, MOV and every ALU/immediate-zero form'),
 'm2_regbranch':(branches,'regsweep','M1 branch-register roles: all 256 selectors, BZ/BNZ, zero/nonzero controls'),
 'm2_regaddr':(addresses,'regsweep','M1 LDI/PLD address roles with mapped fallback pointers; PC bus role separate'),
 'm2_regst':(stores,'stprev,regsweep','M1 last-register-bus-value rule: all extended selectors plus predecessor contrasts'),
 'm2_regjr':(jumps,'r8,regsweep','M1 JR source roles: all 256 selectors, mapped labels and PC self-loop'),
 'm2_reg10':(source10,'regsweep','P11 source 10 difference: all eight GPR destinations, distinct small shift counts, zero/large controls'),
 'm2_resetpram':(reset_pram,'restore,ctrlfx','Isolate which CPU reset writes clear PRAM/GPRs; paired bounded-loop controls'),
 'm2_regbus':(pc_bus,'r8,regsweep','Isolated PC-as-address applicability; run after collected register comparisons'),
}
