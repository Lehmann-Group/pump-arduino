import numpy as np
from scipy import stats
import matplotlib.pyplot as plt


######### pH probe calibration #########
# Enter raw data here
x = np.array([217, 400, 572])
y = np.array([4, 7, 10])

p = np.polyfit(x, y, 1)

res = stats.linregress(x, y)
r2 = res.rvalue ** 2

x0 = np.linspace(x.min(), x.max())
y0 = p[0] * x0 + p[1]

intStr = f'{"Intercept":10s}: {p[1]:10.5f}'
slopeStr = f'{"Slope":10s}: {p[0]:10.5f}'
r2Str = f'{"R2":10s}: {r2:10.5f}'

fig, axs = plt.subplots()
axs.plot(x, y, 'o')
axs.plot(x0, y0, '-')
axs.text(0.9, 0.2, intStr, ha='right', transform=axs.transAxes)
axs.text(0.9, 0.15, slopeStr, ha='right', transform=axs.transAxes)
axs.text(0.9, 0.1, r2Str, ha='right', transform=axs.transAxes)

axs.set_xlabel('Probe reading')
axs.set_ylabel('pH')
plt.tight_layout()
plt.savefig('img/pH-probe-calibration.png')
print('pH probe calibration:')
print(intStr)
print(slopeStr)
print(r2Str)

######### Ammonia probe calibration #########
# Enter raw data here
x = np.array([385, 390, 393, 399, 406.3, 413.5])
y = np.array([0.099385,0.993788,9.92186,98.388,955.6803,7114.27301])

logy = np.log(y)
p = np.polyfit(x, logy, 1)

def Cn(x0):
    y0 = p[0] * x0 + p[1]
    return np.exp(y0)

res = stats.linregress(x, logy)
r2 = res.rvalue ** 2

x0 = np.linspace(x.min(), x.max())

intStr = f'{"Intercept":10s}: {p[1]:10.5f}'
slopeStr = f'{"Slope":10s}: {p[0]:10.5f}'
r2Str = f'{"R2":10s}: {r2:10.5f}'

fig, axs = plt.subplots()
axs.plot(x, y, 'o')
axs.plot(x0, Cn(x0), '-')
axs.text(0.9, 0.2, intStr, ha='right', transform=axs.transAxes)
axs.text(0.9, 0.15, slopeStr, ha='right', transform=axs.transAxes)
axs.text(0.9, 0.1, r2Str, ha='right', transform=axs.transAxes)
axs.set_yscale('log')
axs.set_xlabel('Probe reading')
axs.set_ylabel('N Concentration (mol N / L)')
plt.tight_layout()
plt.savefig('img/ammonia-probe-calibration.png')
print('Ammonia probe calibration:')
print(intStr)
print(slopeStr)
print(r2Str)