from pathlib import Path
import re
root = Path(__file__).resolve().parents[1]
arduino = root / 'components' / 'arduino'
manifest = arduino / 'idf_component.yml'
backup = root / 'tools' / 'arduino_idf_component.original.yml'
if not backup.exists():
    backup.write_bytes(manifest.read_bytes())
# This application uses Arduino core + HardwareSerial only, no Arduino libraries.
# All their optional cloud, Zigbee and WiFi dependencies can be omitted together.
manifest.write_text('dependencies:\n  idf: ">=5.3,<5.5"\n', encoding='utf-8')
cmake = arduino / 'CMakeLists.txt'
text = cmake.read_text()
# Arduino.h always includes IPAddress.h even with optional Network disabled.
text = text.replace('set(requires spi_flash ', 'set(requires esp_netif spi_flash ')
cmake.write_text(text, encoding='utf-8')
cfg = root / 'sdkconfig.defaults'
text = cfg.read_text()
text = text.split('\n# CAR_BRIDGE_MINIMAL_CORE')[0]
options = re.findall(r'^config (ARDUINO_SELECTIVE_\w+)$', (arduino/'Kconfig.projbuild').read_text(), re.M)
text += '\n# CAR_BRIDGE_MINIMAL_CORE\n'
for option in options:
    if option != 'ARDUINO_SELECTIVE_COMPILATION':
        text += f'CONFIG_{option}=n\n'
cfg.write_text(text, encoding='utf-8')
active = root / 'sdkconfig.esp32dev'
if active.exists():
    text = active.read_text()
    for option in options:
        if option != 'ARDUINO_SELECTIVE_COMPILATION':
            text = re.sub(rf'^CONFIG_{option}=.*$', f'# CONFIG_{option} is not set', text, flags=re.M)
    active.write_text(text, encoding='utf-8')
ini = root / 'platformio.ini'
text = ini.read_text()
text = re.sub(r'board_build.embed_txtfiles =\n(?:[ \t]+[^\n]*\n)+', '', text)
ini.write_text(text, encoding='utf-8')
print('Configured Arduino core-only build; optional library dependencies omitted')
