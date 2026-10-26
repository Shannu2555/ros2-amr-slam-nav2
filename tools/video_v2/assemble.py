import subprocess, json
FPS = 30; XF = 0.5
def enc_frames(d, out):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "30", "-i", f"frames/{d}/f%04d.png", "-c:v", "libx264",
                    "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p", "-r", "30", out], check=True)
def enc_still(png, dur, out):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-loop", "1", "-t", str(dur), "-framerate", "30", "-i", png, "-c:v", "libx264",
                    "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p", "-r", "30", out], check=True)
def dur(f):
    return float(subprocess.check_output(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", f]).decode())
segs = []
enc_still("cards/title.png", 4.0, "seg/0.mp4"); segs.append("seg/0.mp4")
enc_frames("map", "seg/1.mp4"); segs.append("seg/1.mp4")
enc_frames("nav", "seg/2.mp4"); segs.append("seg/2.mp4")
for i, (p, d) in enumerate([("table", 6.5), ("bars", 5.5), ("pergoal", 6.0), ("outro", 4.5)]):
    o = f"seg/{3 + i}.mp4"; enc_still(f"cards/{p}.png", d, o); segs.append(o)
ds = [dur(s) for s in segs]
inp = sum([["-i", s] for s in segs], [])
fc = []; prev = "[0:v]"; off = 0
for i in range(1, len(segs)):
    off += ds[i - 1] - XF; lab = f"[v{i}]"
    fc.append(f"{prev}[{i}:v]xfade=transition=fade:duration={XF}:offset={off:.3f}{lab}"); prev = lab; off = off  # cumulative
    # offset accumulates relative to output timeline
fc_s = ";".join(fc)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *inp, "-filter_complex", fc_s, "-map", prev, "-c:v", "libx264", "-preset", "slow",
                "-crf", "20", "-pix_fmt", "yuv420p", "-r", "30", "-profile:v", "high", "-movflags", "+faststart", "-an",
                "/workspace/linkedin_post3_v2.mp4"], check=True)
print(ds, dur("/workspace/linkedin_post3_v2.mp4"))
