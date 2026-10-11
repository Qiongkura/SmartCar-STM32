from pathlib import Path
import zipfile
root = Path(__file__).resolve().parents[1]
commit = 'ac961f671abd5ae1da0a15fd4bee71ed807c2cf3'
dest = root / 'components' / 'arduino'
with zipfile.ZipFile(root / 'arduino-pinned.zip') as archive:
    prefix = f'arduino-esp32-{commit}/'
    for item in archive.infolist():
        if not item.filename.startswith(prefix):
            raise ValueError('Unexpected archive root')
        rel = item.filename[len(prefix):]
        if not rel or item.is_dir():
            continue
        target = (dest / rel).resolve()
        if not target.is_relative_to(dest.resolve()):
            raise ValueError('Archive path traversal')
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(archive.read(item))
(root / 'arduino-source-commit.txt').write_text(commit + '\n')
print('Pinned Arduino source extracted:', commit)
