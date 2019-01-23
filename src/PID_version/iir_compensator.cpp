#include <math.h>
#include <stdlib.h>

#include "iir_compensator.hpp"

//
//  MATLAB Filter prototype definition
//
    // % IIR filter gain
    // gya = (z0 + kz)./(p0 + kz);
    // gyb = 1./(pg + kz);
    // gyc = (-1/r3)*kz./(kz + p1);
    // gff = (-1/r2);
    //
    // % IIR Coefficients for feedback impedance filters
    // a0 = (z0 - kz)./(z0 + kz);
    // b0 = (kz - p0)./(p0 + kz);
    // b1 = (kz - pg)./(pg + kz);
    // b2 = (kz - p1)./(kz + p1);
    //
    // % IIR filters for the feedback impedance
    // ya = (1 + a0.*z1)./(1 - b0.*z1);
    // yb = (1 + z1)./(1 - b1.*z1);
    //
    // % Result of two cascaded stages with gain
    // yfb = g0.*gya.*gyb.*ya.*yb;
    //
    // % Injection impedance IIR filter
    // yc = (1 - z1)./(1 - b2*z1);
    //
    // % Final compensator filter, cascade and sum
    // Hz = gff.*yfb + gyc.*yc.*yfb;

float
run_filter_one_pole(iir_coeffs* cf, float x)
{
    float y0 = cf->a0*x + cf->a1*cf->x1
                        + cf->b0*cf->y1;
    cf->x1 = x;
    cf->y1 = y0;
    return y0;
}

void
compute_filter_coeffs(iir_comp* f)
{
    float kz = f->kz;

    // First-stage IIR
    f->ya.b0 = (kz - f->p0)/(f->p0 + kz);
    f->ya.a0 = 1.0;
    f->ya.a1 = (f->z0 - kz)/(f->z0 + kz);

    // Second-stage IIR
    f->yb.b0 = (kz - f->pg)/(f->pg + kz);
    f->yb.a0 = 1.0;
    f->yb.a1 = 1.0;

    // Third-stage IIR
    f->yc.b0 = (kz - f->p1)/(kz + f->p1);
    f->yc.a0 = 1.0;
    f->yc.a1 = -1.0;


}

void
init_state_variables(iir_comp* f)
{
    // First-stage IIR
    f->ya.y1 = 0.0;
    f->ya.x1 = 0.0;

    // Second-stage IIR
    f->yb.y1 = 0.0;
    f->yb.x1 = 0.0;

    // Third-stage IIR
    f->yc.y1 = 0.0;
    f->yc.x1 = 0.0;
}

void
set_poles_zeros_direct(iir_comp* f, float z0, float p0, float p1, float pg, float gff, float gdt)
{
    f->z0 = z0;
    f->p0 = p0;
    f->p1 = p1;
    f->pg = pg;

    f->gff = 1.0;
    f->gyc = 0.5*f->kz/(f->kz + f->p1);

    f->g0 = (f->p0 - f->z0); //Assumes R1 = 1

    compute_filter_coeffs(f);
}

void
set_circuit_params(iir_comp* f, float r1, float r2, float r3, float c1, float c2, float c3, float pg)
{
    float kz = f->kz;

    f->z0 = 1.0/(r1*c1);
    f->p0 = 1.0/(r1*c1) + 1.0/(r1*c2);
    f->p1 = 1.0/(r3*c3);
    f->pg = pg;

    f->g0 = 1.0/c2;
    f->gya = (f->z0 + kz)/(f->p0 + kz);
    f->gyb = 1.0/(f->pg + kz);
    f->gyc = (-1.0/r3)*kz/(kz + f->p1);
    f->gff = (-1.0/r2);

    compute_filter_coeffs(f);
}

void
init_compensator(iir_comp* f, float fs)
{
    f->fs = fs;
    f->kz = 2.0*fs;

    f->pg = 2.0*M_PI*f->fs/10000.0;
    f->z0 = 2.0*M_PI*f->fs/100.0;;
    f->p0 = f->p0*10.0;
    f->p1 = f->z0/2.0;

    compute_filter_coeffs(f);
    init_state_variables(f);

}

void
set_sampling_params(iir_comp* f, float fs, float warp)
{
    f->fs = fs;
    float w0 = 2.0*M_PI*warp;

    if(warp > 0.0)
    {
        f->kz = w0/tan(w0/(2.0*f->fs));
    }
    else
    {
        f->kz = 2.0*f->fs;
    }

}

float
run_compensator(iir_comp* f, float x)
{
    //yfb = g0.*gya.*gyb.*ya.*yb;
    //Hz = gff.*yfb + gyc.*yc.*yfb;

    float yc = x*f->gff + f->gyc*run_filter_one_pole(&(f->yc), x);
    float ya = f->gya*run_filter_one_pole(&(f->ya), yc);
    float yfb = f->gyb*run_filter_one_pole(&(f->yb), ya);
    yfb *= f->g0;

    return yfb;

}
