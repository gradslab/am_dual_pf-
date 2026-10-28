"""Web versions of the rendered animations: results/anim/<log>_<cam>_drake.mp4 -> docs/anim/<name>.mp4 (960 px wide,
H.264) with a poster frame, and two light GIFs for the README. Needs ffmpeg.

    python3 web/anim_web.py
"""
import os, subprocess, json
HERE = os.path.dirname(os.path.abspath(__file__)); SIM = os.path.abspath(os.path.join(HERE, '..'))
ANIM = os.path.join(SIM, 'results', 'anim'); DOCS = os.path.abspath(os.path.join(SIM, '..', 'docs', 'anim'))
os.makedirs(DOCS, exist_ok=True)

# page name, rendered file, poster time in s
CLIPS = [('nominal_orbit', 'D3_nominal_orbit_drake.mp4', 14.0), ('nominal_side', 'D3_nominal_side_drake.mp4', 14.0),
         ('proposed_side', 'D4_relaxed_side_drake.mp4', 20.0), ('proposed_orbit', 'D4_relaxed_orbit_drake.mp4', 20.0),
         ('cascade_side', 'D4_filter_side_drake.mp4', 17.0), ('cascade_orbit', 'D4_filter_orbit_drake.mp4', 17.0),
         ('nominal_switch_side', 'D4_nominal_side_drake.mp4', 16.0)]
GIFS = [('proposed_side', 'D4_relaxed_side_drake.mp4', 13.0, 17.0), ('nominal_side', 'D3_nominal_side_drake.mp4', 0.0, 12.0)]


def run(cmd):
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    sizes = {}
    for name, src, t_poster in CLIPS:
        src = os.path.join(ANIM, src)
        if not os.path.exists(src):
            print('missing', src); continue
        out = os.path.join(DOCS, name + '.mp4'); jpg = os.path.join(DOCS, name + '.jpg')
        run(['ffmpeg', '-y', '-i', src, '-vf', 'scale=960:-2', '-c:v', 'libx264', '-crf', '26', '-preset', 'slow',
             '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-an', out])
        run(['ffmpeg', '-y', '-ss', str(t_poster), '-i', out, '-frames:v', '1', '-q:v', '4', jpg])
        sizes[name + '.mp4'] = round(os.path.getsize(out) / 1e6, 2); print(name, sizes[name + '.mp4'], 'MB')
    for name, src, t0, dur in GIFS:
        src = os.path.join(ANIM, src)
        if not os.path.exists(src): continue
        out = os.path.join(DOCS, name + '.gif')
        flt = 'fps=10,scale=480:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4'
        run(['ffmpeg', '-y', '-ss', str(t0), '-t', str(dur), '-i', src, '-vf', flt, '-loop', '0', out])
        sizes[name + '.gif'] = round(os.path.getsize(out) / 1e6, 2); print(name + '.gif', sizes[name + '.gif'], 'MB')
    json.dump(sizes, open(os.path.join(SIM, 'results', 'web', 'anim_sizes.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
