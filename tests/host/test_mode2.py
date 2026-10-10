"""Emulate actual C turn/control logic. Requires unicorn and pyelftools."""
from pathlib import Path
import re, shutil, subprocess, tempfile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC
from elftools.elf.elffile import ELFFile
ROOT=Path(__file__).resolve().parents[1]
GCC=Path.home()/'.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc.exe'
a=(ROOT/'src/app.c').read_text('utf8')
def run_c(folder,name,code,extras=()):
 src=folder/(name+'.c');src.write_text(code,encoding='utf8');elf=folder/(name+'.elf')
 r=subprocess.run([str(GCC),'-mcpu=cortex-m3','-mthumb','-mfloat-abi=soft','-O1','-ffreestanding','-fno-builtin','-nostdlib','-I',str(folder),str(src),*map(str,extras),'-Ttext=0x10000','-Tdata=0x40000','-Wl,--entry=test_main','-lm','-lgcc','-o',str(elf)],capture_output=True,text=True)
 assert r.returncode==0,r.stderr
 u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.mem_map(0,0x1000000);u.mem_map(0x20000000,0x100000)
 with elf.open('rb') as f:
  e=ELFFile(f)
  for s in e.iter_segments():
   if s['p_type']=='PT_LOAD':u.mem_write(s['p_vaddr'],s.data())
  entry=e.get_section_by_name('.symtab').get_symbol_by_name('test_main')[0]['st_value']
 u.reg_write(UC_ARM_REG_SP,0x200ff000);u.reg_write(UC_ARM_REG_LR,0x900001)
 u.emu_start(entry|1,0x900000,count=20000000)
 assert u.reg_read(UC_ARM_REG_PC)==0x900000,name+' instruction limit'
 assert u.reg_read(UC_ARM_REG_R0)==0,f'{name}: check {u.reg_read(UC_ARM_REG_R0)} failed'
 print(name+': PASS')
params=a[a.index('#define TURN_RIGHT_RAW_RATIO'):a.index('/* Mode 3 only:')]
states=a[a.index('enum { TURN_DRIVE'):a.index('static float lost_ms')]
logic=a[a.index('static float turn_command_angle'):a.index('/* 模式三:')]
i=a.index('  turn_phase = TURN_DRIVE;',a.index('static void start_run'))
init=a[i:a.index('  line_corner_active',i)]
pre=r'''
#include <stdbool.h>
#include <stdint.h>
static bool running,finished,aborted;
static int p_turn_angle,p_turn_dir;
static float yaw_ref,yaw,rate,command,vl,vr;
static uint32_t tick,run_start_ms,run_end_ms;
static int brakes;
static float f_abs(float x){return x<0?-x:x;}
static float f_clamp(float x,float l,float h){return x<l?l:(x>h?h:x);}
uint32_t HAL_GetTick(void){return tick;}
float imu_yaw(void){return yaw;}
float imu_gyro_z(void){return rate;}
float control_speed_l(void){return vl;}
float control_speed_r(void){return vr;}
void control_brake(void){command=0;brakes++;}
void control_set_spin(float r){command=r;}
#define CHECK(x,n) if(!(x))return n
'''
setup='''static void setup(int angle,int dir){
 tick=100;run_start_ms=tick;run_end_ms=0;running=true;finished=aborted=false;
 p_turn_angle=angle;p_turn_dir=dir;yaw_ref=yaw=1000.0f;
 rate=command=0;vl=vr=2;brakes=0;
'''+init+'''
 if(angle==0)turn_finish(TURN_ERR_NONE);
}
'''
tests=r'''
int test_main(void){
 setup(360,1);CHECK(f_abs(turn_command_angle()-363.02521f)<.0001f,15);
 setup(90,1);CHECK(f_abs(turn_command_angle()-90.75630f)<.0001f,16);
 setup(360,-1);CHECK(f_abs(turn_command_angle()-363.02521f)<.0001f,17);
 setup(90,-1);CHECK(f_abs(turn_command_angle()-90.75630f)<.0001f,19);
 setup(0,-1);CHECK(turn_command_angle()==0.0f&&!running&&brakes,20);
 setup(0,1);CHECK(turn_command_angle()==0.0f,18);
 for(int dir=-1;dir<=1;dir+=2){
  for(int angle=0;angle<=360;angle+=30){
   setup(angle,dir);
   if(angle==0){CHECK(!running&&!aborted&&brakes,1);continue;}
   for(int i=0;i<2400&&running;i++){
    tick+=10;ctrl_turn(0.01f);
    rate+=(command-rate)*(turn_phase==TURN_BRAKING?0.25f:0.08f);
    yaw+=rate*0.01f;
   }
   CHECK(!running&&!aborted,2);
   float physical=f_abs(yaw-1000)*(357.0f/360.0f);
   CHECK(f_abs(physical-angle)<=TURN_TOL_DEG,3);
   CHECK(f_abs(turn_display_angle()-physical)<.001f,14);
   CHECK(brakes>0&&run_end_ms>0,4);
  }
 }
 setup(360,1);yaw=1358.02521f;ctrl_turn(.01f);yaw=1367.02521f;rate=70;
 tick+=10;ctrl_turn(.01f);CHECK(turn_phase==TURN_BRAKING&&command==0,5);
 tick+=10;ctrl_turn(.01f);CHECK(command==0,6);
 rate=0;tick+=10;ctrl_turn(.01f);tick+=260;ctrl_turn(.01f);
 CHECK(turn_phase==TURN_DRIVE&&turn_trims==1,7);
 tick+=10;ctrl_turn(.01f);CHECK(command<0&&command>=-TURN_TRIM_DPS,8);
 for(int side=0;side<2;side++){
  setup(360,1);if(side==0)vl=0;else vr=0;
  for(int i=0;i<200&&running;i++){tick+=10;ctrl_turn(.01f);}
  CHECK(!running&&aborted&&turn_fault==(side?TURN_ERR_RIGHT:TURN_ERR_LEFT),9);
 }
 setup(360,1);yaw=984;ctrl_turn(.01f);CHECK(turn_fault==TURN_ERR_DIRECTION,10);
 setup(360,1);tick+=20001;ctrl_turn(.01f);CHECK(turn_fault==TURN_ERR_TIMEOUT,11);
 setup(90,1);yaw=1090.75630f;rate=50;ctrl_turn(.01f);
 tick+=1801;ctrl_turn(.01f);CHECK(turn_fault==TURN_ERR_SETTLE,12);
 setup(90,1);turn_phase=TURN_BRAKING;turn_brake_ms=tick;turn_trims=TURN_MAX_TRIMS;
 yaw=1080;rate=0;ctrl_turn(.01f);tick+=260;ctrl_turn(.01f);
 CHECK(turn_fault==TURN_ERR_SETTLE&&aborted,13);
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='smartcar-mode2-') as d:
 f=Path(d);run_c(f,'turn_state',pre+params+states+logic+setup+tests)
 shutil.copy2(ROOT/'src/control.c',f/'control.c');shutil.copy2(ROOT/'src/control.h',f/'control.h')
 board=(ROOT/'src/board.h').read_text('utf8')
 defs=[re.search(r'^#define\s+'+n+r'\s+\S+',board,re.M).group() for n in ['PWM_MAX','WHEEL_BASE_CM','ENCODER_CM_L','ENCODER_CM_R']]
 (f/'board.h').write_text('\n'.join(defs))
 (f/'motor.h').write_text('''#include <stdint.h>
#define MOT_LEFT 0
#define MOT_RIGHT 1
void motor_init(void);void motor_set_duty(int,float);void motor_coast(int);
void motor_brake(int);int16_t motor_read_delta(int);
''')
 (f/'line.h').write_text('float line_error(void);int line_count(void);')
 (f/'stm32f1xx_hal.h').write_text('/* Runtime Flash logging is disabled in this build. */')
 run_c(f,'wheel_control',r'''
#include "control.h"
#include "motor.h"
static float duty[2];static int16_t delta[2];
void motor_init(void){}
void motor_set_duty(int s,float d){duty[s]=d;}
void motor_coast(int s){duty[s]=0;}
void motor_brake(int s){duty[s]=2;}
void app_flush_logs(void){}
float line_error(void){return 0;}
int line_count(void){return 0;}
int16_t motor_read_delta(int s){int16_t d=delta[s];delta[s]=0;return d;}
#define CHECK(x,n) if(!(x))return n
int test_main(void){
 control_init();control_reset_dist();control_set_spin(90);control_update(.01f);
 CHECK(duty[0]>0&&duty[1]<0,1);
 for(int i=0;i<300;i++){control_set_spin(90);control_update(.01f);
 CHECK(duty[0]<=.65001f&&duty[1]>=-.65001f,2);}
 control_brake();control_update(.01f);CHECK(duty[0]==2&&duty[1]==2,3);
 control_set_spin(-20);control_update(.01f);CHECK(duty[0]<0&&duty[1]>0,4);
 control_brake();delta[0]=30;delta[1]=30;control_update(.01f);
 CHECK(duty[0]==2&&duty[1]==2,5);
 control_reset();control_set_cmd(10,0);control_update(.01f);
 CHECK(duty[0]>0&&duty[1]>0,6);
 control_stop();control_update(.01f);CHECK(duty[0]==0&&duty[1]==0,7);
 control_reset();control_set_pivot(180);control_update(.01f);
 CHECK(duty[0]>0&&duty[1]<0&&duty[0]+duty[1]<.001f&&duty[0]+duty[1]>-.001f,8);
 return 0;
}
''',extras=[f/'control.c'])
print('26 angle/direction runs plus overshoot, trim, stall, timeout and brake/control checks passed.')
