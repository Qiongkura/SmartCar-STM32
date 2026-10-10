"""Run actual geometry and app corner/gap C on emulated Cortex-M3.

No MCU connection and no physical motor commands. HAL sensors and actuators
are test boundaries; line_map.c and app.c are the actual firmware sources.
"""
from pathlib import Path
import json, subprocess, sys, tempfile

ROOT=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path(__file__).resolve().parents[1]
for deps in (ROOT.parent/'mode2-test-deps',Path('C:/Users/Administrator/Documents/Codex/2026-09-26/new-chat/work/mode2-test-deps')):
    if deps.is_dir():
        sys.path.insert(0,str(deps));break
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_R0,UC_ARM_REG_PC
from elftools.elf.elffile import ELFFile

gcc=Path.home()/'.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc.exe'
framework=Path.home()/'.platformio/packages/framework-stm32cubef1'
includes=[ROOT/'src',framework/'Drivers/CMSIS/Include',
          framework/'Drivers/CMSIS/Device/ST/STM32F1xx/Include',
          framework/'Drivers/STM32F1xx_HAL_Driver/Inc',
          framework/'Drivers/STM32F1xx_HAL_Driver/Src']
control=(ROOT/'src/control.c').read_text('utf-8')
pid=control[control.index('float pid_step('):control.index('void control_init(')]

code=r'''#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "app.c"
static float test_yaw,test_rate,test_dist,test_lin,test_rot,test_vl,test_vr;
static uint32_t test_tick;
static uint8_t test_bits,test_prev_count;
volatile float check_diag[8];
float dist_l_cm,dist_r_cm;
uint8_t line_prev_count;
void *memset(void *p,int v,size_t n){unsigned char *q=p;for(size_t i=0;i<n;++i)q[i]=(unsigned char)v;return p;}
uint32_t HAL_GetTick(void){return test_tick;}
float imu_yaw(void){return test_yaw;}
float imu_yaw_rate(void){return test_rate;}
float imu_gyro_z(void){return test_rate;}
uint16_t motor_raw_cnt(int side){return 0;}
float control_dist_cm(void){return test_dist;}
float control_speed_l(void){return test_vl;}
float control_speed_r(void){return test_vr;}
float control_lin(void){return test_lin;}
void control_brake(void){test_lin=test_rot=0;}
void control_brake_fast(void){test_lin=test_rot=0;}
void control_set_cmd(float v,float w){test_lin=v;test_rot=w;}
void control_set_spin_ex(float w,float gain){test_lin=0;test_rot=w;}
int line_count(void){int n=0;for(int i=0;i<8;++i)if(test_bits&(1u<<i))++n;return n;}
void line_update(void){line_prev_count=test_prev_count;test_prev_count=line_count();}
uint8_t line_bits(void){return test_bits;}
uint8_t line_read_raw(void){return test_bits;}
bool line_lost(void){return test_bits==0;}
float line_error(void){int n=0,s=0;for(int i=0;i<8;++i)if(test_bits&(1u<<i)){++n;s+=i;}return n?((float)s/n-3.5f)/3.5f:0;}
PID_CODE
#define CHECK(x,n) do{if(!(x))return n;}while(0)
static void setup(float yaw){
 test_yaw=yaw;test_rate=test_dist=test_lin=test_rot=test_vl=test_vr=0;
 test_tick=1000;test_bits=0x18;test_prev_count=line_prev_count=0;
 running=true;finished=aborted=false;mode=MODE_LINE;
 line_corner_active=line_settle_active=line_gap_active=line_pull=false;
 line_corner_advanced=line_corner_stopped=false;line_corner_phase=line_corner_trims=0;
 line_track_seen=line_prev_valid=line_end_pending=false;
 line_straight_yaw=line_hold_yaw=yaw;line_straight_valid=true;
 line_corners_done=0;line_seg_start_cm=line_gap_start_cm=0;
 line_align_stable_ms=lost_ms=lost_err=line_derivative=est_e=0;
 line_seen_cnt=line_exit_seen=line_junc_hist=0;line_junc_side=0;
 line_crossing=false;line_wide_side=0;line_wide_cm=-999;
 line_prev_e=line_e_lpf=0;line_map_align_limited=0;ev_n=0;
 line_map_reset(&track_map,yaw,0,4.8f,0,1.1f);line_edge_reset();pid_reset(&pid_head);
}
static void sample(uint8_t bits){test_bits=bits;test_tick+=10;line_follow(.01f,30);}
static uint8_t straight_mask(const line_map_t *m){
 float s,c;line_map_sincos(m->yaw,&s,&c);uint8_t b=0;
 for(int i=0;i<8;++i){float lateral=((float)i-3.5f)*m->pitch_cm;
  float y=m->y+m->forward_cm*s+lateral*c;if(f_abs(y)<1)b|=(1u<<i);
 }return b;
}

volatile float object_trace[120];
static void warm(int m){
 setup(0);mode=m;test_lin=0;acc_yaw_hold(.01f);
 for(int i=0;i<40;i++){test_lin=30;sample(0x18);}
}
int test_main(void){
 for(int m=5;m<=6;m++){
  warm(m);float peak=0,delta=0,energy=0,last=0;
  for(int i=0;i<120;i++){
   test_lin=30;sample(((i/3)&1)?0x08:0x10);
   float v=f_abs(test_rot);if(v>peak)peak=v;
   v=f_abs(test_rot-last);if(v>delta)delta=v;
   energy+=test_rot*test_rot;last=test_rot;
   if(m==6)object_trace[i]=test_rot;
  }
  int k=m==5?0:4;check_diag[k]=peak;check_diag[k+1]=delta;check_diag[k+2]=energy/120;
 }
 warm(5);
 for(int i=0;i<80;i++){test_lin=30;sample(0x20);}
 check_diag[3]=test_rot;CHECK(test_rot>10&&test_rot<90,1);
 warm(5);float last=0,delta=0;
 for(int i=0;i<100;i++){
  test_lin=((i/3)&1)?32:30;test_rate=(i&1)?6:-6;sample(0x18);
  float d=f_abs(test_rot-last);if(d>delta)delta=d;last=test_rot;
 }
 check_diag[7]=delta;
 return 0;
}
'''.replace('PID_CODE',pid)

with tempfile.TemporaryDirectory(prefix='car-line-map-') as tmp:
    folder=Path(tmp);source=folder/'test.c';elf=folder/'test.elf'
    source.write_text(code,'utf-8')
    cmd=[str(gcc),'-mcpu=cortex-m3','-mthumb','-mfloat-abi=soft','-Os',
         '-ffreestanding','-fno-builtin','-ffunction-sections','-fdata-sections','-nostdlib',
         '-DSTM32F103xB','-DUSE_HAL_DRIVER',*['-I'+str(x) for x in includes],
         str(source),str(ROOT/'src/line_map.c'),'-Ttext=0x10000','-Tdata=0x40000',
         '-Wl,--gc-sections,--entry=test_main','-lgcc','-o',str(elf)]
    r=subprocess.run(cmd,capture_output=True,text=True)
    assert r.returncode==0,r.stderr
    u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    u.mem_map(0,0x1000000);u.mem_map(0x20000000,0x100000);u.mem_map(0x40000000,0x30000)
    with elf.open('rb') as f:
        e=ELFFile(f)
        for segment in e.iter_segments():
            if segment['p_type']=='PT_LOAD':u.mem_write(segment['p_vaddr'],segment.data())
        sym=e.get_section_by_name('.symtab')
        entry=sym.get_symbol_by_name('test_main')[0]['st_value']
        diag=sym.get_symbol_by_name('check_diag')[0]['st_value']
        trace=sym.get_symbol_by_name('object_trace')[0]['st_value']
    u.reg_write(UC_ARM_REG_SP,0x200ff000);u.reg_write(UC_ARM_REG_LR,0x900001)
    u.emu_start(entry|1,0x900000,count=50000000)
    assert u.reg_read(UC_ARM_REG_PC)==0x900000,'instruction limit'
    failure=u.reg_read(UC_ARM_REG_R0)
    if failure:
        import struct
        print('geometry snapshot:',struct.unpack('<8f',u.mem_read(diag,32)))
    assert failure==0,f'Actual line-map/app C check {failure} failed'

    import struct,math
    values=list(struct.unpack('<8f',u.mem_read(diag,32)))
    result=dict(mode5_peak=values[0],mode5_step=values[1],mode5_rms=math.sqrt(values[2]),
                mode5_persistent_offset_command=values[3],mode6_peak=values[4],
                mode6_step=values[5],mode6_rms=math.sqrt(values[6]),mode5_acc_step=values[7],
                mode6_trace=list(struct.unpack('<120f',u.mem_read(trace,480))))
    if 'MODE5_ROT_SLEW' in (ROOT/'src/app.c').read_text('utf8'):
        assert result['mode5_step']<=2.401 and result['mode5_acc_step']<=2.401,result
    if len(sys.argv)>2:Path(sys.argv[2]).write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k!='mode6_trace'}))
