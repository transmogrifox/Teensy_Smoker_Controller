
#ifndef IIR_COMP_H
#define IIR_COMP_H

typedef struct iir_coeffs_t {
    float b0;
    float a0;
    float a1;

    float y1;
    float x1;

} iir_coeffs;

typedef struct iir_comp_t
{
    // Sampling parameters
    float fs;
    float kz;  //Nominally (2/T), but can be adjusted for pre-warping if desired
    // Analog prototype components

    // Gains, poles and zeros for the compensator
    float g0;   // 1/C2
    float gya;  // IIR first stage cascaded gain
    float gyb;  // IIR second cascaded stage gain
    float gyc;  // RC feed-forward gain (Includes -1/R3)
    float gff;  // -1/R2

    float z0;   //primary system zero
    float p0;   //feedback pole involving series resistor and parallel cap
    float p1;   //conpensator RC injection pole
    float pg;   //low frequency pole of finite-gain integrator
                //relates to compensator DC gain

    // IIR coefficients and state variables
    iir_coeffs ya;
    iir_coeffs yb;
    iir_coeffs yc;


} iir_comp;

//
// Pass a pointer to the struct.
// Populates with default values:
//  pg = 2*pi*fs/10000
//  z0 = 2*pi*fs/100
//  p0 = z0*10
//  p1 = z0/2
//  gyc = 0.5
//  gff = 1.0
//

void init_compensator(iir_comp* f, float fs);

// Directly set the locations of poles and zeros
// In the analog prototype, the following gives guidance for definition
// of where these appear in the circuit.
//  z0  : Primary zero set by series RC in the op amp output to inverting input
//  p0  : Pole formed by opamp output-inverting input network
//  p1  : Pole formed by RC feed network
//  pg  : Low frequency pole, characteristic of amplifier DC gain
//  gff : Feed-forward gain from input feed resistor (ie 1/Rfeed)
//  gdt : Feed-forward gain on differentiator input track
//
//  There is an implicit zero at the origin from the input feed RC.
//
void set_poles_zeros_direct(iir_comp* f, float z0, float p0, float p1, float pg, float gff, float gdt);

// Else you can define the compensator response by entering values directly from
// an analog prototype.
//
//  r1  : series to c1 in op amp feedback (out to minus)
//  c1  : as used with r1
//  c2  : high frequency noise filter (sets the high frequency pole)
//  r2  : input feed resistor from signal input to op amp inverting terminal
//  r3  : input feed resistor for differentiator feed path, in series with c3
//  c3  : forms pole with r3.  r3 + c3 series connected from input to op amp inverting terminal
//

void set_circuit_params(iir_comp* f, float r1, float r2, float r3, float c1, float c2, float c3, float pg);

// Configure sampling params
//
//  fs    :  sampling rate, Hz
//  warp  :  Matching frequency (Hz) for pre-warping frequency response
//           If set to zero, it will default to the "K = 2/T" factor in the BLT
//           For any other non-zero input, pre-warping will be applied by
//           K = w0/tan(w0/(2*fs))
//
void set_sampling_params(iir_comp* f, float fs, float warp);

//
// Main function to run the compensator
//
//  x  :  Input sample
//
// return value is the compensator filter output
//

float run_compensator(iir_comp* f, float x);

#endif //IIR_COMP_H
