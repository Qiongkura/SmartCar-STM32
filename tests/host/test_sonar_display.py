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

code=r'''
#include <stdint.h>
#include <stdbool.h>
#include "app.c"
static char captured[4][40];
static uint32_t tick=1000;
volatile float check_diag[8];
int16_t gyro_raw[3];
uint8_t yaw_axis;
uint32_t HAL_GetTick(void){return tick;}
float imu_yaw(void){return 0;}
float imu_gyro_z(void){return 0;}
float control_dist_cm(void){return 0;}
float control_lin(void){return 0;}
float control_speed_l(void){return 0;}
float control_speed_r(void){return 0;}
uint16_t motor_raw_cnt(int s){return 0;}
uint8_t line_bits(void){return 0;}
void OLED_ClearLine(uint8_t y){}
void OLED_DrawString(uint8_t x,uint8_t y,const char *text){strncpy(captured[y/16],text,39);captured[y/16][39]=0;}
void OLED_Refresh(void){}
#define CHECK(x,n) do{if(!(x))return n;}while(0)
int test_main(void){
 const float old_live[4]={14.1f,21.6f,12.4f,25.8f};
 const char *expected[4]={"12.7","20.0","11.1","24.0"};
 for(int i=0;i<4;i++){
  char row[40];mode=MODE_OBJECT;imu_fault=false;finished=running=false;
  sonar_cm=sonar_disp=old_live[i];obj_state=OBJ_IDLE;obj_num=0;obj_page=0;
  ui_draw();snprintf(row,sizeof(row),"到轨 %s cm",expected[i]);
  CHECK(strstr(captured[2],expected[i])!=0,1); CHECK(strcmp(captured[2],row)==0,8); /* Catches the old idle-display bypass. */
  running=true;ui_draw();CHECK(strcmp(captured[3],row)==0,2);
  sonar_new=true;obj_update();running=false;finished=true;ui_draw();
  snprintf(row,sizeof(row),"1:%s ",expected[i]);CHECK(strcmp(captured[1],row)==0,3);
  mode=MODE_DEBUG;dbg_page=1;finished=false;ui_draw();
  snprintf(row,sizeof(row),"到轨 %s cm",expected[i]);CHECK(strcmp(captured[1],row)==0,4);
 }
 mode=MODE_OBJECT;running=finished=false;sonar_cm=sonar_disp=-1;
 ui_draw();CHECK(strcmp(captured[2],"到轨 无回波")==0,5);
 running=true;ui_draw();CHECK(strcmp(captured[3],"到轨 -- cm")==0,6);
 mode=MODE_DEBUG;dbg_page=1;ui_draw();CHECK(strcmp(captured[1],"到轨 -- cm")==0,7);
 return 0;
}
'''

with tempfile.TemporaryDirectory(prefix='car-line-map-') as tmp:
    folder=Path(tmp);source=folder/'test.c';elf=folder/'test.elf'
    source.write_text(code,'utf-8')
    cmd=[str(gcc),'-mcpu=cortex-m3','-mthumb','-mfloat-abi=soft','-Os',
         '-ffreestanding','-fno-builtin','-ffunction-sections','-fdata-sections','-nostdlib',
         '-DSTM32F103xB','-DUSE_HAL_DRIVER',*['-I'+str(x) for x in includes],
         str(source),str(ROOT/'src/line_map.c'),'-Ttext=0x10000','-Tdata=0x40000',
         '-Wl,--gc-sections,--entry=test_main','-Wl,--defsym=end=0x60000','-Wl,--start-group','-lc','-lnosys','-lgcc','-Wl,--end-group','-o',str(elf)]
    r=subprocess.run(cmd,capture_output=True,text=True,encoding="utf8",errors="replace")
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
    print('OLED measurement paths: PASS (actual app.c UI, idle/running/results/debug/no echo).')
