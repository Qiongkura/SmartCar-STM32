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
#include <stdbool.h>
#include "app.c"
static float test_dist,test_yaw,test_speed,test_rot;
static uint32_t test_tick;
static int test_brakes;
volatile float check_diag[8];
uint32_t HAL_GetTick(void){return test_tick;}
float imu_yaw(void){return test_yaw;}
float control_dist_cm(void){return test_dist;}
void control_brake(void){test_brakes++;test_speed=test_rot=0;}
void control_set_cmd(float v,float r){test_speed=v;test_rot=r;}
PID_CODE
#define CHECK(x,n) do {if(!(x))return n;} while(0)
static void object_reset(void){
 obj_num=0;obj_cur=0;obj_state=OBJ_IDLE;test_tick=1000;test_dist=0;
 sonar_new=false;obj_last_seen_ms=0;obj_last_seen_dist=0;
}
static void echo(float d){sonar_cm=d;sonar_new=true;obj_update();}
static void distance_reset(int target,float dist){
 running=true;finished=aborted=enc_fault=false;
 p_dist_cm=target;p_cruise=47;test_dist=dist;test_yaw=yaw_ref=0;
 test_tick=1100;run_start_ms=1000;test_brakes=0;test_speed=test_rot=0;
 pid_head.kp=5;pid_head.ki=pid_head.kd=0;pid_reset(&pid_head);
}
int test_main(void){
 const float displayed[4]={14.1f,21.6f,12.4f,25.8f};
 const float actual[4]={12.8f,20.0f,11.0f,24.0f};
 const float expected[4]={12.72121f,19.97057f,11.07802f,24.03020f};
 for(int i=0;i<4;i++){
  object_reset();echo(displayed[i]);
  check_diag[i]=obj_dist[0];
  CHECK(obj_num==1&&obj_state==OBJ_TRACKING,1);
  CHECK(f_abs(obj_dist[0]-expected[i])<.002f,2);
  CHECK(f_abs(obj_dist[0]-actual[i])<.08f,3);
  CHECK(sonar_cm==displayed[i],4); /* Raw sensor value unchanged. */
 }
 CHECK(obj_track_distance_cm(0)<0&&obj_track_distance_cm(-1)<0,5);
 object_reset();echo(0);echo(-1);echo(100);
 CHECK(obj_num==0&&obj_state==OBJ_IDLE,6);
 echo(17.6f);float nearest=obj_dist[0];
 test_tick+=50;echo(20);CHECK(obj_num==1&&obj_dist[0]==nearest,7);
 test_tick+=50;echo(16);CHECK(obj_num==1&&obj_dist[0]<nearest,8);
 test_tick+=249;obj_update();CHECK(obj_state==OBJ_TRACKING,9);
 test_tick++;obj_update();CHECK(obj_state==OBJ_IDLE,10);
 echo(20);CHECK(obj_num==2,11);
 test_dist=12.01f;obj_update();CHECK(obj_state==OBJ_IDLE,12);
 for(int target=50;target<=100;target+=10){
  distance_reset(target,target-.60f);ctrl_dist(.01f);
  CHECK(running&&!test_brakes&&test_speed>=10,13);
  distance_reset(target,target-.50f);ctrl_dist(.01f);
  CHECK(!running&&finished&&!aborted&&test_brakes==1,14);
 }
 distance_reset(50,0);test_tick=2601;ctrl_dist(.01f);
 CHECK(!running&&finished&&aborted&&enc_fault&&test_brakes==1,15);
 distance_reset(50,45);test_yaw=10;ctrl_dist(.01f);
 CHECK(running&&test_rot<0&&test_speed>=10,16);
 running=false;int brakes=test_brakes;ctrl_dist(.01f);
 CHECK(test_brakes==brakes,17);
 return 0;
}
'''.replace('PID_CODE',pid)

with tempfile.TemporaryDirectory(prefix='car-mode-calibration-') as tmp:
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
    print('Mode 3/6 calibration: PASS (actual Cortex-M3 app.c, four live-display samples, braking and counting).')
