% Generic stuff
k = 1000;
m = 1/k;

% Sampling params
Ts = (1/100)*2^9
fs = 1/Ts; %Hz
ws = 2*pi*fs;

f = (0.01/3600):(0.01/3600):fs;
size(f);

s = j*2*pi*f;
kz = 2*fs;
z1 = e.^(-s/fs);
sz = kz*(1 - z1)./(1 + z1);

% Circuit components
r1 = 1*k;
r2 = 1*k;
r3 = 1*k;
c1 = 500*m;
c2 = c1/20;
c3 = c2/2;

% Coefficients
g0 = 1/c2
z0 = 1/(r1*c1)
p0 = 1/(r1*c1) + 1/(r1*c2)
p1 = 1/(r3*c3)
pg = ws/(20*k);

g0chk = r1*(p0-z0)

g1 = -1/r2;
g2 = (-1/r3).*s./(s + p1);

% s-domain
zf = (g0./(s + pg)).*(s + z0)./(s + p0);
Hs = g2.*zf + g1.*zf;

% z-domain

% IIR filter gain
gya = (z0 + kz)./(p0 + kz);
gyb = 1./(pg + kz);
gyc = (-1/r3)*kz./(kz + p1);
gff = (-1/r2);

% IIR Coefficients for feedback impedance filters
a0 = (z0 - kz)./(z0 + kz);
b0 = (kz - p0)./(p0 + kz);
b1 = (kz - pg)./(pg + kz);
b2 = (kz - p1)./(kz + p1);

% IIR filters for the feedback impedance
ya = (1 + a0.*z1)./(1 - b0.*z1);
yb = (1 + z1)./(1 - b1.*z1);

% Result of two cascaded stages with gain
yfb = g0.*gya.*gyb.*ya.*yb;

% Injection impedance IIR filter
yc = (1 - z1)./(1 - b2*z1);

% Final compensator filter, cascade and sum
Hz = gff.*yfb + gyc.*yc.*yfb;

subplot (2, 1, 1)
semilogx(f, 20*log10(Hs));
hold on
semilogx(f, 20*log10(Hz), 'r');
hold off

subplot (2, 1, 2)
semilogx(f, (180/pi)*angle(Hs));
hold on
semilogx(f, (180/pi)*angle(Hz), 'r');
hold off