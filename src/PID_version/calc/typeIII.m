% Generic stuff
k = 1000;
m = 1/k;
p = 1e-12;
n = 1*p*k;

% Sampling params
Ts = 1/130000;
fs = 1/Ts; %Hz
ws = 2*pi*fs;

f = 1:2:fs;

s = j*2*pi*f;

% Circuit components
r1 = 2.2*k;
r2 = 2.2*k;
r3 = 4.02*k;
c1 = 470*n;
c2 = 5.3*n;
c3 = 52*n;

%
g0 = 1/c2;
z0 = 1/(r1*c1);
p0 = 1/(r1*c1) + 1/(r1*c2);
p1 = 1/(r3*c3);
pg = 0.5*ws/(130*k);

g1 = -1/r2;
g2 = (-1/r3).*s./(s + p1);

zf =  (g0./s).*(s + z0)./(s + p0);
zf = (g0./(s + pg)).*(s + z0)./(s + p0);
Hst2 =  g1.*zf;
Hs = g2.*zf + g1.*zf;

subplot (2, 1, 1)
semilogx(f, 20*log10(Hst2));
hold on
semilogx(f, 20*log10(Hs), 'r');
hold off

subplot (2, 1, 2)
semilogx(f, (180/pi)*angle(Hst2));
hold on
semilogx(f, (180/pi)*angle(Hs), 'r');
hold off