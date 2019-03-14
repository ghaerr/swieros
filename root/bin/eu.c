// eu -- user mode cpu emulator
//
// Usage:  eu [-v] file ...
//
// Description:
//
// Written by Robert Swierczek

#include <u.h>
#include <libc.h>
#include <libm.h>
#include <net.h>

enum { STACKSZ = 8*1024*1024 }; // user stack size (8M)

int verbose;
char *cmd;
unsigned char *mem;
uint memsz;

char ops[] = 
  "HALT,ENT ,LEV ,JMP ,JMPI,JSR ,JSRA,LEA ,LEAG,CYC ,MCPY,MCMP,MCHR,MSET," // system
  "LL  ,LLS ,LLH ,LLC ,LLB ,LLD ,LLF ,LG  ,LGS ,LGH ,LGC ,LGB ,LGD ,LGF ," // load a
  "LX  ,LXS ,LXH ,LXC ,LXB ,LXD ,LXF ,LI  ,LHI ,LIF ,"
  "LBL ,LBLS,LBLH,LBLC,LBLB,LBLD,LBLF,LBG ,LBGS,LBGH,LBGC,LBGB,LBGD,LBGF," // load b
  "LBX ,LBXS,LBXH,LBXC,LBXB,LBXD,LBXF,LBI ,LBHI,LBIF,LBA ,LBAD,"
  "SL  ,SLH ,SLB ,SLD ,SLF ,SG  ,SGH ,SGB ,SGD ,SGF ,"                     // store
  "SX  ,SXH ,SXB ,SXD ,SXF ,"
  "ADDF,SUBF,MULF,DIVF,"                                                   // arithmetic
  "ADD ,ADDI,ADDL,SUB ,SUBI,SUBL,MUL ,MULI,MULL,DIV ,DIVI,DIVL,"
  "DVU ,DVUI,DVUL,MOD ,MODI,MODL,MDU ,MDUI,MDUL,AND ,ANDI,ANDL,"
  "OR  ,ORI ,ORL ,XOR ,XORI,XORL,SHL ,SHLI,SHLL,SHR ,SHRI,SHRL,"
  "SRU ,SRUI,SRUL,EQ  ,EQF ,NE  ,NEF ,LT  ,LTU ,LTF ,GE  ,GEU ,GEF ,"      // logical  
  "BZ  ,BZF ,BNZ ,BNZF,BE  ,BEF ,BNE ,BNEF,BLT ,BLTU,BLTF,BGE ,BGEU,BGEF," // conditional
  "CID ,CUD ,CDI ,CDU ,"                                                   // conversion
  "CLI ,STI ,RTI ,BIN ,BOUT,NOP ,SSP ,PSHA,PSHI,PSHF,PSHB,POPB,POPF,POPA," // misc
  "IVEC,PDIR,SPAG,TIME,LVAD,TRAP,LUSP,SUSP,LCL ,LCA ,PSHC,POPC,MSIZ,"
  "PSHG,POPG,NET1,NET2,NET3,NET4,NET5,NET6,NET7,NET8,NET9,"
  "POW ,ATN2,FABS,ATAN,LOG ,LOGT,EXP ,FLOR,CEIL,HYPO,SIN ,COS ,TAN ,ASIN," // math
  "ACOS,SINH,COSH,TANH,SQRT,FMOD,"
  "IDLE,";
  
int cpu(uint pc, int argc, char **argv)
{
  uint a, b, c, sp, cycle = 0;
  int ir, n;
  double f, g;
  unsigned char *p;
  uint av;

  sp = (memsz + STACKSZ) & -8;
  av = sp -= ((argc+1)*4+7) & -8;			// argv
  
  // copy arguments to 32-bit stack
  *(uint *)(mem+av + argc*4) = 0;		// NULL
  for(ir=argc-1; ir>=0; ir--) {
  	n = strlen(argv[ir]) + 1;
	sp -= (n+7) & -8;
	memcpy(mem+sp, argv[ir], n);
  	*(uint *)(mem+av + ir*4) = sp;		// av[i]
  }

  sp -= 8; *(uint *)(mem+sp) = TRAP | (S_exit<<8); // call exit if main returns
  sp -= 8; *(uint *)(mem+sp) = av;
  sp -= 8; *(uint *)(mem+sp) = argc;
  sp -= 8; *(uint *)(mem+sp) = sp + 24;

  for (;;) {   
    if (sp & 7) { dprintf(2,"stack pointer not a multiple of 8! sp = %u\n", sp); return -1; }
    cycle++;    
    ir = *(int *)(mem+pc);
if (verbose) dprintf(2,"%08x  %08x%6.4s\n", pc, ir, &ops[(ir&0xff)*5]);
    pc += 4;
    switch ((uchar)ir) {
    case HALT: dprintf(2,"halted! a = %d cycle = %u\n", a, cycle); return -1; // XXX supervisor mode

    // memory
    case MCPY: memcpy(mem+a, mem+b, c); a += c; b += c; c = 0; continue;
    case MCMP: a = memcmp(mem+a, mem+b, c); b += c; c = 0; continue;
    case MCHR: p = memchr(mem+a, b, c); a = p? p-mem: 0; c = 0; continue;
    case MSET: memset(mem+a, b, c); a += c; c = 0; continue;

    // math
    case POW:  f = pow(f,g); continue;
    case ATN2: f = atan2(f,g); continue;
    case FABS: f = fabs(f); continue;
    case ATAN: f = atan(f); continue;
    case LOG:  f = log(f); continue;
    case LOGT: f = log10(f); continue;
    case EXP:  f = exp(f); continue;
    case FLOR: f = floor(f); continue;
    case CEIL: f = ceil(f); continue;
    case HYPO: f = hypot(f,g); continue;
    case SIN:  f = sin(f); continue;
    case COS:  f = cos(f); continue;
    case TAN:  f = tan(f); continue;
    case ASIN: f = asin(f); continue;
    case ACOS: f = acos(f); continue;
    case SINH: f = sinh(f); continue;
    case COSH: f = cosh(f); continue;
    case TANH: f = tanh(f); continue;
    case SQRT: f = sqrt(f); continue;
    case FMOD: f = fmod(f,g); continue;

    // procedure linkage
    case ENT:  sp += ir>>8; continue;
    case LEV:  sp += ir>>8; pc = *(uint *)(mem+sp); sp += 8; continue;

    // jump
    case JMP:  pc += ir>>8; continue;
    case JMPI: pc += ((uint *)(mem+pc + (ir>>8)))[a]; continue;
    case JSR:  sp -= 8; *(uint *)(mem+sp) = pc; pc += ir>>8; continue;
    case JSRA: sp -= 8; *(uint *)(mem+sp) = pc; pc = a; continue;

    // stack
    case PSHA: sp -= 8; *(uint *)(mem+sp) = a; continue;
    case PSHB: sp -= 8; *(uint *)(mem+sp) = b; continue;
    case PSHC: sp -= 8; *(uint *)(mem+sp) = c; continue;
    case PSHF: sp -= 8; *(double *)(mem+sp) = f; continue;
    case PSHG: sp -= 8; *(double *)(mem+sp) = g; continue;
    case PSHI: sp -= 8; *(uint *)(mem+sp) = ir>>8; continue;

    case POPA: a = *(uint *)(mem+sp); sp += 8; continue;
    case POPB: b = *(uint *)(mem+sp); sp += 8; continue;
    case POPC: c = *(uint *)(mem+sp); sp += 8; continue;
    case POPF: f = *(double *)(mem+sp); sp += 8; continue;
    case POPG: g = *(double *)(mem+sp); sp += 8; continue;

    // load effective address
    case LEA:  a = sp + (ir>>8); continue; 
    case LEAG: a = pc + (ir>>8); continue;

    // load a local
    case LL:   a = *(uint *)   (mem+sp + (ir>>8)); continue;
    case LLS:  a = *(short *)  (mem+sp + (ir>>8)); continue;
    case LLH:  a = *(ushort *) (mem+sp + (ir>>8)); continue;
    case LLC:  a = *(char *)   (mem+sp + (ir>>8)); continue;
    case LLB:  a = *(uchar *)  (mem+sp + (ir>>8)); continue;
    case LLD:  f = *(double *) (mem+sp + (ir>>8)); continue;
    case LLF:  f = *(float *)  (mem+sp + (ir>>8)); continue;

    // load a global
    case LG:   a = *(uint *)   (mem+pc + (ir>>8)); continue;
    case LGS:  a = *(short *)  (mem+pc + (ir>>8)); continue;
    case LGH:  a = *(ushort *) (mem+pc + (ir>>8)); continue;
    case LGC:  a = *(char *)   (mem+pc + (ir>>8)); continue;
    case LGB:  a = *(uchar *)  (mem+pc + (ir>>8)); continue;
    case LGD:  f = *(double *) (mem+pc + (ir>>8)); continue;
    case LGF:  f = *(float *)  (mem+pc + (ir>>8)); continue;

    // load a indexed
    case LX:   a = *(uint *)   (mem+a + (ir>>8)); continue;
    case LXS:  a = *(short *)  (mem+a + (ir>>8)); continue;
    case LXH:  a = *(ushort *) (mem+a + (ir>>8)); continue;
    case LXC:  a = *(char *)   (mem+a + (ir>>8)); continue;
    case LXB:  a = *(uchar *)  (mem+a + (ir>>8)); continue;
    case LXD:  f = *(double *) (mem+a + (ir>>8)); continue;
    case LXF:  f = *(float *)  (mem+a + (ir>>8)); continue;

    // load a immediate
    case LI:   a = ir>>8; continue;
    case LHI:  a = a<<24 | (uint)ir>>8; continue;
    case LIF:  f = (ir>>8)/256.0; continue;

    // load b local
    case LBL:  b = *(uint *)   (mem+sp + (ir>>8)); continue;
    case LBLS: b = *(short *)  (mem+sp + (ir>>8)); continue;
    case LBLH: b = *(ushort *) (mem+sp + (ir>>8)); continue;
    case LBLC: b = *(char *)   (mem+sp + (ir>>8)); continue;
    case LBLB: b = *(uchar *)  (mem+sp + (ir>>8)); continue;
    case LBLD: g = *(double *) (mem+sp + (ir>>8)); continue;
    case LBLF: g = *(float *)  (mem+sp + (ir>>8)); continue;

    // load b global
    case LBG:  b = *(uint *)   (mem+pc + (ir>>8)); continue;
    case LBGS: b = *(short *)  (mem+pc + (ir>>8)); continue;
    case LBGH: b = *(ushort *) (mem+pc + (ir>>8)); continue;
    case LBGC: b = *(char *)   (mem+pc + (ir>>8)); continue;
    case LBGB: b = *(uchar *)  (mem+pc + (ir>>8)); continue;
    case LBGD: g = *(double *) (mem+pc + (ir>>8)); continue;
    case LBGF: g = *(float *)  (mem+pc + (ir>>8)); continue;

    // load b indexed
    case LBX:  b = *(uint *)   (mem+b + (ir>>8)); continue;
    case LBXS: b = *(short *)  (mem+b + (ir>>8)); continue;
    case LBXH: b = *(ushort *) (mem+b + (ir>>8)); continue;
    case LBXC: b = *(char *)   (mem+b + (ir>>8)); continue;
    case LBXB: b = *(uchar *)  (mem+b + (ir>>8)); continue;
    case LBXD: g = *(double *) (mem+b + (ir>>8)); continue;
    case LBXF: g = *(float *)  (mem+b + (ir>>8)); continue;

    // load b immediate
    case LBI:  b = ir>>8; continue;
    case LBHI: b = b<<24 | (uint)ir>>8; continue;
    case LBIF: g = (ir>>8)/256.0; continue;

    // misc transfer
    case LCL:  c = *(uint *)(mem+sp + (ir>>8)); continue;
    case LBA:  b = a; continue;
    case LCA:  c = a; continue;
    case LBAD: g = f; continue;

    // store a local
    case SL:   *(uint *)   (mem+sp + (ir>>8)) = a; continue;
    case SLH:  *(ushort *) (mem+sp + (ir>>8)) = a; continue;
    case SLB:  *(uchar *)  (mem+sp + (ir>>8)) = a; continue;
    case SLD:  *(double *) (mem+sp + (ir>>8)) = f; continue;
    case SLF:  *(float *)  (mem+sp + (ir>>8)) = f; continue;

    // store a global
    case SG:   *(uint *)   (mem+pc + (ir>>8)) = a; continue;
    case SGH:  *(ushort *) (mem+pc + (ir>>8)) = a; continue;
    case SGB:  *(uchar *)  (mem+pc + (ir>>8)) = a; continue;
    case SGD:  *(double *) (mem+pc + (ir>>8)) = f; continue;
    case SGF:  *(float *)  (mem+pc + (ir>>8)) = f; continue;

    // store a indexed
    case SX:   *(uint *)   (mem+b + (ir>>8)) = a; continue;
    case SXH:  *(ushort *) (mem+b + (ir>>8)) = a; continue;
    case SXB:  *(uchar *)  (mem+b + (ir>>8)) = a; continue;
    case SXD:  *(double *) (mem+b + (ir>>8)) = f; continue;
    case SXF:  *(float *)  (mem+b + (ir>>8)) = f; continue;
      
    // arithmetic
    case ADDF: f += g; continue;
    case SUBF: f -= g; continue;
    case MULF: f *= g; continue;
    case DIVF: f /= g; continue;

    case ADD:  a += b; continue;
    case ADDI: a += ir>>8; continue;
    case ADDL: a += *(uint *)(mem+sp + (ir>>8)); continue;

    case SUB:  a -= b; continue;
    case SUBI: a -= ir>>8; continue;
    case SUBL: a -= *(uint *)(mem+sp + (ir>>8)); continue;

    case MUL:  a *= b; continue;
    case MULI: a *= ir>>8; continue;
    case MULL: a *= *(uint *)(mem+sp + (ir>>8)); continue;

    case DIV:  a = (int)a / (int)b; continue;
    case DIVI: a = (int)a / (ir>>8); continue;
    case DIVL: a = (int)a / *(int *)(mem+sp + (ir>>8)); continue;

    case DVU:  a /= b; continue;
    case DVUI: a /= ir>>8; continue;
    case DVUL: a /= *(uint *)(mem+sp + (ir>>8)); continue;

    case MOD:  a = (int)a % (int)b; continue;
    case MODI: a = (int)a % (ir>>8); continue;
    case MODL: a = (int)a % *(int *)(mem+sp + (ir>>8)); continue;

    case MDU:  a %= b; continue;
    case MDUI: a %= ir>>8; continue;
    case MDUL: a %= *(uint *)(mem+sp + (ir>>8)); continue;

    case AND:  a &= b; continue;
    case ANDI: a &= ir>>8; continue;
    case ANDL: a &= *(uint *)(mem+sp + (ir>>8)); continue;

    case OR:   a |= b; continue;
    case ORI:  a |= ir>>8; continue;
    case ORL:  a |= *(uint *)(mem+sp + (ir>>8)); continue;

    case XOR:  a ^= b; continue;
    case XORI: a ^= ir>>8; continue;
    case XORL: a ^= *(uint *)(mem+sp + (ir>>8)); continue;

    case SHL:  a <<= b; continue;
    case SHLI: a <<= ir>>8; continue;
    case SHLL: a <<= *(uint *)(mem+sp + (ir>>8)); continue;

    case SHR:  a = (int)a >> (int)b; continue;
    case SHRI: a = (int)a >> (ir>>8); continue;
    case SHRL: a = (int)a >> *(int *)(mem+sp + (ir>>8)); continue;

    case SRU:  a >>= b; continue;
    case SRUI: a >>= ir>>8; continue;
    case SRUL: a >>= *(uint *)(mem+sp + (ir>>8)); continue;

    // logical
    case EQ:   a = a == b; continue;
    case EQF:  a = f == g; continue;
    case NE:   a = a != b; continue;
    case NEF:  a = f != g; continue;
    case LT:   a = (int)a < (int)b; continue;
    case LTU:  a = a < b; continue;
    case LTF:  a = f < g; continue;
    case GE:   a = (int)a >= (int)b; continue;
    case GEU:  a = a >= b; continue;
    case GEF:  a = f >= g; continue;

    // branch
    case BZ:   if (!a)               pc += ir>>8; continue;
    case BZF:  if (!f)               pc += ir>>8; continue;
    case BNZ:  if (a)                pc += ir>>8; continue;
    case BNZF: if (f)                pc += ir>>8; continue;
    case BE:   if (a == b)           pc += ir>>8; continue;
    case BEF:  if (f == g)           pc += ir>>8; continue;
    case BNE:  if (a != b)           pc += ir>>8; continue;
    case BNEF: if (f != g)           pc += ir>>8; continue;
    case BLT:  if ((int)a < (int)b)  pc += ir>>8; continue;
    case BLTU: if (a < b)            pc += ir>>8; continue;
    case BLTF: if (f < g)            pc += ir>>8; continue;
    case BGE:  if ((int)a >= (int)b) pc += ir>>8; continue;
    case BGEU: if (a >= b)           pc += ir>>8; continue;
    case BGEF: if (f >= g)           pc += ir>>8; continue;
    
    // conversion
    case CID:  f = (int)a; continue;
    case CUD:  f = a; continue;
    case CDI:  a = (int)f; continue;
    case CDU:  a = f; continue;

    // misc
    case SSP:  sp = a; continue;
    case NOP:  continue;
    case CYC:  a = cycle; continue;

    case TRAP:
      switch (ir>>8) {
      case S_fork:    a = fork();                                  continue; // fork()
      case S_exit:    if (verbose) dprintf(2,"exit(%d) cycle = %u\n", a, cycle); return a; // exit(rc)
      case S_wait:    a = wait();                                  continue; // wait()
      case S_pipe:    a = pipe((void *)mem+a);                     continue; // pipe(&fd)
      case S_write:   a = write(a, (void *)mem+b, c);              continue; // write(fd, p, n)
      case S_read:    a = read(a, (void *)mem+b, c);               continue; // read(fd, p, n)
      case S_close:   a = close(a);                                continue; // close(fd)
      case S_kill:    a = kill(a);                                 continue; // kill(pid)
      case S_exec:    a = exec((void *)mem+a, (void *)mem+b);      continue; // exec(path, argv)
      case S_open:    a = open((void *)mem+a, b);                  continue; // open(path, mode)
      case S_mknod:   a = mknod((void *)mem+a, b, c);              continue; // mknod(path, mode, dev)
      case S_unlink:  a = unlink((void *)mem+a);                   continue; // unlink(path)
      case S_fstat:   a = fstat(a, (void *)mem+b);                 continue; // fstat(fd, p)
      case S_link:    a = link((void *)mem+a, (void *)mem+b);      continue; // link(old, new)
      case S_mkdir:   a = mkdir((void *)mem+a);                    continue; // mkdir(path);
      case S_chdir:   a = chdir((void *)mem+a);                    continue; // chdir(path)
      case S_dup2:    a = dup2(a, b);                              continue; // dup2(fd, nfd)
      case S_getpid:  a = getpid();                                continue; // getpid()
      case S_sbrk:    p = sbrk(a); a = (p==(void *)-1)? -1: p-mem; continue; // sbrk(size)
      case S_sleep:   a = sleep(a);                                continue; // sleep(msec) XXX msec vs sec
      case S_uptime:  a = uptime();                                continue; // uptime()
      case S_lseek:   a = lseek(a, b, c);                          continue; // lseek(fd, pos, whence)
      case S_mount:   a = mount((void *)mem+a, (void *)mem+b, c);  continue; // mount(spec, dir, rwflag)
      case S_umount:  a = umount((void *)mem+a);                   continue; // umount(spec)
      case S_socket:  a = socket(a, b, c);                         continue; // socket(family, type, protocol)
      case S_bind:    a = bind(a, (void *)mem+b, c);               continue; // bind(fd, addr, addrlen)
      case S_listen:  a = listen(a, b);                            continue; // listen(fd, len)
      case S_poll:    a = poll((void *)mem+a, b, c);               continue; // poll(pfd, n, msec)
      case S_accept:  a = accept(a, (void *)mem+b, (void *)mem+c); continue; // accept(fd, addr, addrlen)
      case S_connect: a = connect(a, (void *)mem+b, c);            continue; // connect(fd, addr, addrlen)

//      case S_shutdown:
//      case S_getsockopt:
//      case S_setsockopt:
//      case S_getpeername:
//      case S_getsockname:
      
      default: dprintf(2,"unsupported trap cycle = %u pc = %08x ir = %08x a = %d b = %d c = %d", cycle, pc, ir, a, b, c); return -1;
      }
    default:   dprintf(2,"unknown instruction cycle = %u pc = %08x ir = %08x\n", cycle, pc, ir); return -1;
    }
  }
}

void usage()
{
  dprintf(2,"%s : usage: %s [-v] file ...\n", cmd, cmd);
  exit(-1);
}

int main(int argc, char *argv[])
{
  int f, rc;
  struct { uint magic, bss, entry, flags; } hdr;
  char *file;
  struct stat st;

  cmd = *argv;
  if (argc < 2) usage();
  file = *++argv;
  verbose = 0;
  while (--argc && *file == '-') {
    switch(file[1]) {
    case 'v': verbose = 1; break;
    default: usage();
    }
    file = *++argv;
  }

  if ((f = open(file, O_RDONLY)) < 0) { dprintf(2,"%s : couldn't open %s\n", cmd, file); return -1; }
  if (fstat(f, &st)) { dprintf(2,"%s : couldn't stat file %s\n", cmd, file); return -1; }

  read(f, &hdr, sizeof(hdr));
  if (hdr.magic != 0xC0DEF00D) { dprintf(2,"%s : bad hdr.magic\n", cmd); return -1; }
  
  memsz = (uint)st.st_size - sizeof(hdr) + hdr.bss;
  mem = sbrk(memsz + STACKSZ);

  if (verbose) dprintf(2, "mem %x-%x, memsz %x, stacksz %x, bss %x, entry %x, flags %x\n", mem, mem+memsz+STACKSZ, memsz, STACKSZ, hdr.bss, hdr.entry, hdr.flags);
  if (read(f, mem, st.st_size - sizeof(hdr)) != (uint)st.st_size - sizeof(hdr)) { dprintf(2,"%s : read error\n"); return -1; }
  close(f);

  if (verbose) dprintf(2,"%s : emulating %s\n", cmd, file);
  rc = cpu(hdr.entry, argc, argv);
  if (verbose) dprintf(2,"%s : %s returned %d.\n", cmd, file, rc);
  return rc;
}
