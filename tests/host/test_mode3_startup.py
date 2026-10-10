"""Compile actual wheel control and app acceleration selection for Cortex-M3."""
from pathlib import Path
import argparse, re, shutil, subprocess, tempfile
import sys
sys.path.insert(0, str(Path(r'C:/Users/Administrator/Documents/Codex/2026-09-26/new-chat/work/mode2-test-deps')))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC
from elftools.elf.elffile import ELFFile

arg=argparse.ArgumentParser();arg.add_argument('--project',type=Path,default=Path(__file__).resolve().parents[1])
root=arg.parse_args().project
gcc=Path.home()/'.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc.exe'
app=(root/'src/app.c').read_text('utf-8')
selection=re.search(r'  acc_slew_l = .*?;',app,re.S).group()
test=r'''
#include <stdint.h>
#include "app.h"
static float duty[2];static int16_t pulses[2];
void motor_init(void){}
void motor_set_duty(int s,float d){duty[s]=d;}
void motor_coast(int s){duty[s]=0;}
void motor_brake(int s){duty[s]=2;}
int16_t motor_read_delta(int s){int16_t d=pulses[s];pulses[s]=0;return d;}
void app_flush_logs(void){}
float line_error(void){return 0;}
int line_count(void){return 0;}
#include "control.c"
static void select_profile(car_mode_t mode){
SELECTION
}
#define CHECK(x,n) do{if(!(x))return (n);}while(0)
static void reset(void){
 control_brake_fast();pulses[0]=pulses[1]=0;
}
static int targets_equal(void){
 int i=(bb_wr+BB_N-1)%BB_N;
 return bb_buf[i*BB_FLD+2]==bb_buf[i*BB_FLD+3];
}
int test_main(void){
 control_init();select_profile(MODE_RC);
 reset();
 /* Actual output targets must remain equal throughout straight acceleration. */
 for(int i=0;i<80;i++){
  control_set_cmd(60,0);control_update(.01f);CHECK(targets_equal(),1);
 }
 CHECK(ramp_l>59.99f&&ramp_l<60.01f&&ramp_r>59.99f&&ramp_r<60.01f,2);
 for(int i=0;i<80;i++){
  control_set_cmd(15,0);control_update(.01f);CHECK(targets_equal(),3);
 }
 reset();
 for(int i=0;i<80;i++){
  control_set_cmd(-60,0);control_update(.01f);CHECK(targets_equal(),4);
 }
 CHECK(ramp_l< -59.99f&&ramp_r< -59.99f,5);
 for(int i=0;i<140;i++){
  control_set_cmd(60,0);control_update(.01f);CHECK(targets_equal(),6);
 }
 /* Different control tick periods must preserve left/right target symmetry. */
 float ticks[]={.005f,.015f,.02f,.05f};
 for(int j=0;j<4;j++){
  reset();for(int i=0;i<140;i++){
   control_set_cmd(60,0);control_update(ticks[j]);CHECK(targets_equal(),7);
  }
 }
 /* TANK straight commands use the same acceleration profile in both directions. */
 for(int direction=-1;direction<=1;direction+=2){
  reset();for(int i=0;i<60;i++){
   control_set_wheels(direction*40,direction*40);control_update(.01f);
   CHECK(targets_equal(),8);
  }
 }
 /* Intentionally different wheel speeds / steering must remain available. */
 reset();for(int i=0;i<60;i++){
  control_set_wheels(40,20);control_update(.01f);
 }
 CHECK(ramp_l>39.99f&&ramp_r<20.01f&&!targets_equal(),9);
 reset();control_set_cmd(30,30);control_update(.01f);
 CHECK(bb_buf[2]>bb_buf[3],10);
 reset();control_set_spin(90);control_update(.01f);
 CHECK(duty[0]>0&&duty[1]<0&&duty[0]<=.65001f&&duty[1]>=-.65001f,11);
 /* Feed actual encoder deltas with the left wheel lagging: PI must increase
  * left drive relative to right, rather than achieving equality by fixed PWM. */
 reset();for(int i=0;i<30;i++){
  control_set_cmd(30,0);control_update(.01f);
 }
 for(int i=0;i<8;i++){
  pulses[0]=16;pulses[1]=19;
  control_set_cmd(30,0);control_update(.01f);
 }
 CHECK(meas_l<meas_r&&duty[0]>duty[1],12);
 CHECK(sync_trim<0,13);
 /* Existing fast stop stays latched even when an encoder reports movement. */
 control_brake_fast();pulses[0]=12;pulses[1]=12;control_update(.01f);
 CHECK(duty[0]==2&&duty[1]==2,14);
 /* Selection of the profile for other modes is unchanged. */
 select_profile(MODE_LINE);CHECK(acc_slew_l==ACC_SLEW_L,15);
 select_profile(MODE_OBJECT);CHECK(acc_slew_l==ACC_SLEW_L,16);
 select_profile(MODE_DIST);CHECK(acc_slew_l==1.0f,17);
 reset();for(int i=0;i<80;i++){
  control_set_cmd(47,0);control_update(.01f);
  CHECK(targets_equal()&&ramp_l==ramp_r,19);
 }
 CHECK(ramp_l==47.0f,20);
 select_profile(MODE_VOFA);CHECK(acc_slew_l==ACC_SLEW_L_OTHER,18);
 return 0;
}
'''.replace('SELECTION',selection)
with tempfile.TemporaryDirectory(prefix='rc-accel-') as d:
    folder=Path(d)
    for name in ('control.c','control.h','app.h'):
        shutil.copy2(root/'src'/name,folder/name)
    board=(root/'src/board.h').read_text('utf-8')
    names=['PWM_MAX','WHEEL_BASE_CM','ENCODER_CM_L','ENCODER_CM_R']
    defs=[re.search(r'^#define\s+'+n+r'\s+\S+',board,re.M).group() for n in names]
    (folder/'board.h').write_text('\n'.join(defs),'utf-8')
    (folder/'motor.h').write_text('''#include <stdint.h>
#define MOT_LEFT 0
#define MOT_RIGHT 1
void motor_init(void);void motor_set_duty(int,float);void motor_coast(int);
void motor_brake(int);int16_t motor_read_delta(int);
''')
    (folder/'line.h').write_text('float line_error(void);int line_count(void);')
    (folder/'stm32f1xx_hal.h').write_text('''#include <stdint.h>
typedef struct {uint32_t TypeErase,PageAddress,NbPages;} FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_PAGES 0
#define FLASH_TYPEPROGRAM_HALFWORD 0
#define HAL_OK 0
static inline void HAL_FLASH_Unlock(void){}
static inline void HAL_FLASH_Lock(void){}
static inline int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *er,uint32_t *err){return 1;}
static inline int HAL_FLASH_Program(int t,uint32_t a,uint16_t v){return 0;}
''')
    source=folder/'test.c';source.write_text(test,'utf-8');elf=folder/'test.elf'
    cmd=[str(gcc),'-mcpu=cortex-m3','-mthumb','-mfloat-abi=soft','-O1',
         '-ffreestanding','-nostdlib','-I',str(folder),str(source),
         '-Ttext=0x10000','-Tdata=0x40000','-Wl,--entry=test_main','-lm','-lgcc','-o',str(elf)]
    r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
    u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    u.mem_map(0,0x1000000);u.mem_map(0x20000000,0x100000)
    with elf.open('rb') as f:
        e=ELFFile(f)
        for segment in e.iter_segments():
            if segment['p_type']=='PT_LOAD':u.mem_write(segment['p_vaddr'],segment.data())
        entry=e.get_section_by_name('.symtab').get_symbol_by_name('test_main')[0]['st_value']
    u.reg_write(UC_ARM_REG_SP,0x200ff000);u.reg_write(UC_ARM_REG_LR,0x900001)
    u.emu_start(entry|1,0x900000,count=20000000)
    assert u.reg_read(UC_ARM_REG_PC)==0x900000,'instruction limit'
    failure=u.reg_read(UC_ARM_REG_R0)
    assert failure==0,f'RC acceleration/control check {failure} failed'
    print('Mode 3 startup + existing RC/control: PASS (20 checks, actual control C)')
