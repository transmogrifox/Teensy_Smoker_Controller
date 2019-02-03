#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fftw3.h>

#include "../lib/iir_compensator.hpp"

#define ROWS 64535

    //FFTW variables
    fftw_complex fftw_in[ROWS], fftw_out[ROWS];
    fftw_plan ftPlanForward;

int main(int argc, char** argv)
{
    //Sampling frequency
    float Ts = 0.5;
    float fs = 1.0/Ts;
    float ws = 2.0*M_PI*fs;

    //Compensator
    iir_comp typeIII;
    iir_comp* comp = &typeIII;

    //Circuit params
    float k = 1000.0;
    float m = 1.0/k;
    float r1 = 1*k;
    float r2 = 1*k;
    float r3 = 100;
    float c1 = 500*m;
    float c2 = c1/20.0;
    float c3 = c2*2.5;
    float pg = ws/(520.0*k);
    init_compensator(comp, fs);
    set_circuit_params(comp, r1, r2, r3, c1, c2, c3, pg);


    FILE * pImpulse, *pFFT;
    pImpulse = fopen ("impulse_response.txt","w");
    pFFT = fopen ("frequency_response.txt","w");

    size_t nfftFrameSize = ROWS;
    ftPlanForward = fftw_plan_dft_1d(nfftFrameSize, fftw_in, fftw_out, FFTW_FORWARD, FFTW_MEASURE);

    int kk;
    for(kk=0; kk<nfftFrameSize; kk++)
    {
        fftw_in[kk][0] = 0.0;
        fftw_in[kk][1] = 0.0;
    }

    //output filter pulse responses
    int i,j;
    int b = 0;
    float gstep = 6.0;
    float gp = -6.0;
    float x = 0.0;
    float y = 0.0;
    unsigned int seq = 0;
    float impulse[ROWS+1];
    float fft[ROWS+1];
    float ftfreq[ROWS];
    float real = 0.0;
    float imag = 0.0;
    float fstep = fs/ROWS;

    for(j=0; j<ROWS; j++)
    {
            impulse[j] = 0.0;
    }

    fprintf(pImpulse, "SEQ\tX\n");
    fprintf(pFFT, "SEQ\tdB\tAngle\n");

    for(j=0; j<ROWS; j++)
    {
        if(j == 0 ) x = 1.0;
        else x = 0.0;
        y = run_compensator(comp, x);

        impulse[j] = y;

        fftw_in[j][0] = y;
        fftw_in[j][1] = 0.0;
    }

    fftw_execute(ftPlanForward);
    float angle = 0.0;
    float freqdec = 1.0;
    float decstep = 1.00016924;
    int nfreqdec = 1;
    for(j=0; j<ROWS/2; j++)
    {
        real = fftw_out[j][0];
        imag = fftw_out[j][1];
        fft[j] = 20.0*log10( sqrtf(real*real + imag*imag) );
        angle = (180.0/M_PI)*atan2(imag,real);
        //fprintf(pFFT, "%f\t%f\n", log10(((float) j)*fs/ROWS+1e-6), fft[j]);
        float tmp = freqdec;
        nfreqdec = lrintf(tmp);
        //if(j > nfreqdec)
        //{
            fprintf(pFFT, "%d\t%f\t%f\n", j, fft[j], angle);
        //}
        freqdec *= decstep;

    }

    int DS = 100;
    for(j=0; j<ROWS; j+=DS)
    {
        fprintf(pImpulse, "%d\t%f\n", j, impulse[j]);
    }

    fftw_destroy_plan(ftPlanForward);
    fclose(pImpulse);
    fclose(pFFT);

    return 0;
}
