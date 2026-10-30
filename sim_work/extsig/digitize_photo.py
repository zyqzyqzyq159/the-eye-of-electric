# 实机照片迹线提取 + 谐波分析：定位畸变类型（偶次=单边压缩 / 奇次=对称 / 台阶=码错误）
from PIL import Image
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

im = Image.open(r'C:\Users\hp\Documents\xwechat_files\wxid_n6ehpgmojrzj22_dc68'
                r'\temp\RWTemp\2026-10\9e20f478899dc29eb19741386f9343c8'
                r'\a5ff2c97277f5d8e34a18249c0911569.jpg').convert('RGB')
a = np.asarray(im).astype(int)
R, G, B = a[..., 0], a[..., 1], a[..., 2]
red = (R > 110) & (R - G > 45) & (R - B > 45)

x0, x1 = 371, 2403                      # A 窗主区（跳过左侧红字面板）
trace = np.full(x1 - x0, np.nan)
for i, x in enumerate(range(x0, x1)):
    ys = np.where(red[:, x])[0]
    if len(ys):
        trace[i] = np.median(ys)

k = 9
pad = np.pad(trace, k // 2, mode='edge')
ts = np.nanmedian(sliding_window_view(pad, k), axis=1)
np.save('photo_trace.npy', ts)

ts0 = ts - ts.mean()
ac = np.correlate(ts0, ts0, 'full')[len(ts0) - 1:]
ac = ac / ac[0]
peaks = [(i, ac[i]) for i in range(50, 700) if ac[i] > ac[i - 1] and ac[i] >= ac[i + 1]]
peaks.sort(key=lambda z: -z[1])
print('autocorr peaks:', [(p, round(v, 3)) for p, v in peaks[:6]])
p = peaks[0][0]
print('period =', p, 'px')

n = len(ts) // p
fold = np.zeros(256)
for i in range(n):
    seg = ts[i * p:(i + 1) * p]
    xp = np.linspace(0, 256, len(seg), endpoint=False)
    fold += np.interp(np.arange(256), xp, seg)
fold /= n
np.save('photo_fold.npy', fold)

f = np.abs(np.fft.rfft(fold - fold.mean()))
h = f[1:9] / f[1] * 100
thd = np.sqrt(np.sum((f[2:16] / f[1]) ** 2)) * 100
print('harmonics pct of h1: h2=%.1f h3=%.1f h4=%.1f h5=%.1f h6=%.1f h7=%.1f h8=%.1f'
      % tuple(h))
print('THD(from photo) = %.1f%%   (screen shows 43.0%%)' % thd)
rms = np.sqrt(np.mean((fold - fold.mean()) ** 2))
print('crest = %.3f (sine 1.414)' % (np.abs(fold - fold.mean()).max() / rms))
pk, bt = fold.max(), fold.min()
mid = (pk + bt) / 2
print('peak asymmetry: +%.0f%% / -%.0f%% of pp'
      % ((pk - mid) / (pk - bt) * 100, (mid - bt) / (pk - bt) * 100))

# 各周期一致性
devs = []
for i in range(n):
    seg = ts[i * p:(i + 1) * p]
    xp = np.linspace(0, 256, len(seg), endpoint=False)
    s = np.interp(np.arange(256), xp, seg)
    devs.append(np.sqrt(np.mean((s - fold) ** 2)))
print('period-to-period rms dev: mean %.1f px, max %.1f px (pp=%.0f px) -> %s'
      % (np.mean(devs), np.max(devs), pk - bt,
         'IDENTICAL' if np.mean(devs) < 0.02 * (pk - bt) else 'VARIES'))

print('folded shape (2 periods shown, 80 cols):')
H = 16
row = ''.join('#' for _ in range(80))
grid = [[' '] * 80 for _ in range(H)]
for i in range(80):
    v = fold[(i % 40) * 256 // 40:(i % 40 + 1) * 256 // 40].mean()
    r = int((1 - (v - bt) / (pk - bt)) * (H - 1))
    grid[r][i] = '#'
for g in grid:
    print(''.join(g))
