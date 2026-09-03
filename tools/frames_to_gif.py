"""프레임 PNG 들을 README 에 넣을 만한 크기의 GIF 로 묶는다.

    python -m tools.frames_to_gif out/drive/frames_007_ORD_02 chase --out docs/img/drive_chase.gif --width 480 --max-frames 180

drive_isaac 이 끝날 때 자동으로 부르지만, Isaac 을 다시 돌리지 않고 폭·프레임 수만
바꿔 다시 뽑을 때 직접 쓴다. 프레임이 max_frames 를 넘으면 균등하게 건너뛴다.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image


def make_gif(frames_dir: Path, prefix: str, out: Path, *, width: int = 400, fps: int = 10, max_frames: int = 120, colors: int = 128) -> Path | None:
    files = sorted(Path(frames_dir).glob(f"{prefix}_*.png"))
    if not files:
        return None
    if len(files) > max_frames:
        step = len(files) / max_frames
        files = [files[int(i * step)] for i in range(max_frames)]
    imgs = []
    for f in files:
        im = Image.open(f).convert("RGB")
        h = round(im.height * width / im.width)
        imgs.append(im.resize((width, h), Image.LANCZOS).convert("P", palette=Image.ADAPTIVE, colors=colors))
    out.parent.mkdir(parents=True, exist_ok=True)
    imgs[0].save(out, save_all=True, append_images=imgs[1:], duration=int(1000 / fps), loop=0, optimize=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames_dir")
    ap.add_argument("prefix", help="chase 또는 fpv")
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=400)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--max-frames", type=int, default=120)
    args = ap.parse_args()
    out = make_gif(Path(args.frames_dir), args.prefix, Path(args.out), width=args.width, fps=args.fps, max_frames=args.max_frames)
    if out is None:
        raise SystemExit(f"프레임 없음: {args.frames_dir}/{args.prefix}_*.png")
    print(f"저장: {out}  ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
