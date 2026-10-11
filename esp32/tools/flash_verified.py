from pathlib import Path
import subprocess, json, hashlib, os, sys
root = Path(__file__).resolve().parents[1]
build = root / '.pio' / 'build' / 'esp32dev'
python = root / '.tools' / 'flash-env' / 'Scripts' / 'python.exe'
port = sys.argv[1] if len(sys.argv) > 1 else 'COM13'
env = dict(os.environ, PYTHONIOENCODING='utf-8')
def run(args, name):
    p = subprocess.run([str(python), '-m', 'esptool', '--chip', 'esp32', '--port', port,
                        '--baud', '460800', *args], capture_output=True, text=True, encoding='utf-8', env=env)
    (root/name).write_text(p.stdout+p.stderr, encoding='utf-8')
    if p.returncode:
        print((p.stdout+p.stderr)[-2000:])
        raise RuntimeError(name+' failed')
    return p.stdout+p.stderr
backup = root / 'esp32_original_383e51c04f00.bin'
assert backup.stat().st_size == 0x400000, 'Original flash backup missing'
assert hashlib.sha256(backup.read_bytes()).hexdigest() == '5659a2c833ac56794fa71d69e4b5f3fc0e6e7b8d46667bcb85cd7529ff6abe22'
identity = run(['read-mac'], 'identity.log')
assert '38:3e:51:c0:4f:00' in identity.lower(), 'Wrong ESP32; no flash written'
config = json.loads((build/'flasher_args.json').read_text())
files = []
hashes = {}
for offset, name in config['flash_files'].items():
    file = build/name
    assert file.is_file() and file.stat().st_size > 0, name+' missing'
    files += [offset, str(file)]
    hashes[name] = hashlib.sha256(file.read_bytes()).hexdigest()
settings = config['write_flash_args']
run(['write-flash', *settings, *files], 'upload.log')
run(['verify-flash', *files], 'verify.log')
record = {'port':port, 'mac':'38:3e:51:c0:4f:00', 'chip':'ESP32-D0WD-V3',
          'flash_size':4194304, 'original_backup_sha256':hashlib.sha256(backup.read_bytes()).hexdigest(),
          'uploaded_files_sha256':hashes, 'write_and_verify_passed':True,
          'stm32_firmware_written':False, 'controller_pairing_verified':False}
(root/'verification.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
print('ESP32 written and verified; MAC 38:3e:51:c0:4f:00; STM32 not flashed')
