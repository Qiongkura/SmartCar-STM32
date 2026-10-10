"""Run actual geometry and app corner/gap C on emulated Cortex-M3.

No MCU connection and no physical motor commands. HAL sensors and actuators
are test boundaries; line_map.c and app.c are the actual firmware sources.
"""
from pathlib import Path
import json, subprocess, sys, tempfile

ROOT=Path(__file__).resolve().parents[1]
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

code=r'''
#include <stdint.h>
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
int test_main(void){
 float s,c;line_map_t m;
 line_map_sincos(90,&s,&c);CHECK(s>.999f&&f_abs(c)<.001f,1);
 line_map_sincos(-90,&s,&c);CHECK(s<-.999f&&f_abs(c)<.001f,2);
 CHECK(line_map_angle_diff(721,359)==2,3);
 line_map_reset(&m,0,0,4.8f,0,1.1f);line_map_update(&m,10,0);
 CHECK(f_abs(m.x-0)<.001f,4); /* reject impossible >5cm single-tick jump */
 line_map_reset(&m,0,0,4.8f,0,1.1f);
 for(int i=1;i<=20;++i)line_map_update(&m,.5f*i,0);
 CHECK(f_abs(m.x-10)<.01f&&f_abs(m.y)<.01f,5);
 line_map_update(&m,9.5f,0);CHECK(f_abs(m.x-9.5f)<.01f,6);
 /* Spatial observations recover the true line despite a 5-degree approach. */
 line_map_reset(&m,5,0,4.8f,0,1.1f);
 for(int i=1;i<=65;++i){line_map_update(&m,.5f*i,5);line_map_observe(&m,straight_mask(&m));}
 CHECK(m.axis_valid&&m.position_valid&&f_abs(m.heading)<2,7);
 check_diag[0]=m.heading;check_diag[1]=m.offset;check_diag[2]=m.x;check_diag[3]=m.y;
 /* Offset is measured in the fitted normal, not the original world Y axis. */
 CHECK(f_abs(line_map_cross_error(&m)+m.y)<.8f,8);
 float heading=m.heading;
 CHECK(!line_map_observe(&m,0xff)&&!line_map_observe(&m,0x81),9);
 CHECK(m.heading==heading,10);
 m.axis_yaw=m.heading=20;m.turns=0;m.track_valid=m.position_valid=true;
 line_map_begin_corner(&m,1,25);CHECK(f_abs(m.heading-110)<.01f,11);
 line_map_finish_corner(&m);line_map_begin_corner(&m,1,160);
 CHECK(f_abs(m.heading-200)<.01f,12); /* unknown new line cannot rotate square axes */
 for(int i=0;i<18;++i){line_map_finish_corner(&m);line_map_begin_corner(&m,-1,123);}
 CHECK(m.heading==20+90*m.turns,13);
 /* Two separated OUTER probes constrain corner position with known 90deg. */
 for(int dir=-1;dir<=1;dir+=2){
  line_map_reset(&m,0,20,4.8f,0,1.1f);m.x=-4.8f;
  m.axis_valid=m.track_valid=m.position_valid=true;
  line_map_begin_corner(&m,dir,0);m.x=-.8f;m.dist=24;
  float ds[8],ys[8];for(int i=0;i<8;++i){ds[i]=20;ys[i]=0;}
  uint8_t outer=dir<0?0x03:0xc0;
  CHECK(line_map_corner_edges(&m,dir,outer,ds,ys),14);
  CHECK(m.position_valid&&f_abs(m.corner_x)<.02f,15);
  CHECK(f_abs(line_map_advance_remaining(&m)-.8f)<.03f,44);
  CHECK(!line_map_corner_edges(&m,dir,0x18,ds,ys),16);
  ds[dir<0?1:7]+=2;
  CHECK(!line_map_corner_edges(&m,dir,outer,ds,ys),17);
  ds[dir<0?1:7]=0;
  CHECK(!line_map_corner_edges(&m,dir,outer,ds,ys),18);
 }
 /* Locked gap heading does not learn from noise; steering is rate limited. */
 line_map_reset(&m,0,0,4.8f,0,1.1f);m.position_valid=true;m.offset=1;
 line_map_lock_gap(&m);CHECK(!line_map_observe(&m,0x18)&&m.heading==0,19);
 float w=line_map_steer(&m,0,0,.01f,true);CHECK(w>0&&w<=1.801f,20);
 for(int i=0;i<100;++i)w=line_map_steer(&m,-20,-100,.01f,true);
 CHECK(w<=35.001f&&w>=-35.001f&&m.integral<=10,21);
 CHECK(!line_map_aligned(&m,0,0),22);m.offset=0;
 CHECK(line_map_aligned(&m,0,0)&&!line_map_aligned(&m,6,0),23);
 /* 60cm gap with small initial lateral offset: heading/position feedback. */
 line_map_reset(&m,0,0,4.8f,0,1.1f);m.position_valid=true;m.offset=0;m.y=1;
 float y=0,rate=0,dist=0;
 for(int i=0;i<334;++i){
  w=line_map_steer(&m,y,rate,.01f,false);rate+=.15f*(w-rate);y+=rate*.01f;
  dist+=.18f;line_map_update(&m,dist,y);
 }
 CHECK(f_abs(m.y)<.8f&&f_abs(y)<4&&m.heading==0,24);
 /* Existing single-side confirmation remains; both-side black is ignored. */
 setup(0);for(int i=0;i<8;++i)sample(0x18);
 for(int i=0;i<8;++i)sample(0xff);CHECK(!line_corner_active,25);
 for(int dir=-1;dir<=1;dir+=2){
  setup(25);track_map.axis_yaw=track_map.heading=20;track_map.axis_valid=true;
  track_map.position_valid=true;line_straight_yaw=20;
  for(int i=0;i<8;++i)sample(0x18);
  for(int i=0;i<4;++i)sample(dir<0?0x0f:0xf0);
  CHECK(line_corner_active&&line_corner_dir==dir,26);
  CHECK(f_abs(line_corner_target-(20+90*dir))<.01f,27);
  test_dist=4;sample(0);CHECK(line_corner_advanced,28);
  test_tick+=150;sample(0);CHECK(line_corner_stopped,29);
  test_yaw=line_corner_target;test_rate=0;sample(0);
  CHECK(line_corner_phase==1&&test_lin==0,30);
  test_tick+=130;sample(0);CHECK(!line_corner_active&&line_settle_active,31);
  for(int i=0;i<15;++i){test_dist+=.18f;sample(0);}
  CHECK(!line_settle_active&&line_gap_active&&running,32);
 }
 /* Persistent 10cm disagreement at alignment limit must stop. */
 setup(0);line_settle_active=true;line_settle_t0=test_tick;
 track_map.position_valid=true;track_map.offset=10;
 for(int i=0;i<180&&running;++i)sample(0);
 CHECK(!running&&aborted&&test_lin==0&&test_rot==0&&line_map_align_limited==2,33);
 /* Large residual cannot bypass turn validation. */
 setup(0);line_begin_corner(1);line_corner_advanced=line_corner_stopped=true;
 line_corner_phase=1;line_corner_t0=test_tick-200;
 test_yaw=60;sample(0);CHECK(!running&&aborted&&test_rot==0,34);
 /* Limited correction is slow and cannot repeat indefinitely. */
 setup(0);line_begin_corner(1);line_corner_advanced=line_corner_stopped=true;
 line_corner_phase=1;line_corner_t0=test_tick-200;test_yaw=86;
 sample(0);CHECK(line_corner_trims==1&&line_corner_phase==0,35);
 sample(0);CHECK(test_rot>0&&test_rot<=18.001f,36);
 line_corner_phase=1;line_corner_trims=2;line_corner_t0=test_tick-200;
 sample(0);CHECK(!running&&aborted,37);
 /* Gap length guard, frozen heading, and no unintended corner on white. */
 setup(0);track_map.axis_valid=track_map.position_valid=true;track_map.confidence=3;
 for(int i=0;i<300;++i){test_dist=i*.2f;sample(0);}
 CHECK(line_gap_active&&!line_corner_active&&track_map.heading==0,38);
 CHECK(!line_search_failed(30)&&test_lin==30&&f_abs(test_rot)<.01f,39);
 test_dist=81;CHECK(line_search_failed(30),40);
 /* Reacquisition resets stale derivative and unlocks spatial learning. */
 test_dist=60;sample(0x18);CHECK(!line_gap_active&&!track_map.frozen,41);
 /* Disjoint black points cannot establish an outgoing line position. */
 setup(0);line_settle_active=true;line_settle_t0=test_tick;
 for(int i=0;i<4;++i)sample(0x81);
 CHECK(!track_map.position_valid&&line_exit_seen==0,42);
 /* Fallback at 20cm/s must not inherit a timeout calculated for 60cm/s. */
 setup(0);sample(0);lost_ms=3500;test_dist=60;
 CHECK(!line_search_failed(60),43);
 for(int mask=0;mask<256;++mask){
  setup(0);for(int i=0;i<8;++i)sample(0x18);
  for(int i=0;i<5;++i)sample(mask);
  int left=0,right=0;
  for(int i=0;i<8;++i)if(mask&(1u<<i)){if(i<4)++left;else ++right;}
  bool expected=(left==4&&right<=2)||(right==4&&left<=2);
  CHECK(line_corner_active==expected,45);
 }
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
    u.reg_write(UC_ARM_REG_SP,0x200ff000);u.reg_write(UC_ARM_REG_LR,0x900001)
    u.emu_start(entry|1,0x900000,count=50000000)
    assert u.reg_read(UC_ARM_REG_PC)==0x900000,'instruction limit'
    failure=u.reg_read(UC_ARM_REG_R0)
    if failure:
        import struct
        print('geometry snapshot:',struct.unpack('<8f',u.mem_read(diag,32)))
    assert failure==0,f'Actual line-map/app C check {failure} failed'
    print('Line map + app: PASS (45 checks including all 256 line masks, actual Cortex-M3 C).')
