//   fft.h - declaration of class
//   of fast Fourier transform - FFT
//
//   The code is property of LIBROW
//   You can use it on your own
//   When utilizing credit LIBROW site

#ifndef FFT_H
#define FFT_H

//#include "complex.h"
#include "complextype.h"
#include <string>

class CFFT
{
	public:
		//Convolution functions
		//NFFT is the FFT size (will be modified if invalid!), nSIG is the size for the input, NFIL is the size of the filter
		//Zero padding is automatically taken care of in the convolution function.
		//T means the function returns result in time domain; F is the result in the frequency domain.
		static Complex* convolutionF(const Complex *input,const Complex *filter, long nSIG, long NFIL, long &NFFT);
		static Complex* convolutionT(const Complex *input,const Complex *filter, long nSIG, long NFIL, long &NFFT);
		static Complex* stereoConvMonoInputF(const Complex *input, const Complex *filterLeft, const Complex *filterRight, long nSIG, long NFILL, long NFILR, long &NFFT);
		static Complex* stereoConvMonoInputT(const Complex *input, const Complex *filterLeft, const Complex *filterRight, long nSIG,long NFILL, long NFILR, long &NFFT);
		static Complex* stereoConvStereoInputF(const Complex *input, const Complex *filterLeft, const Complex *filterRight, long nSIG, long NFILL, long NFILR, long &NFFT);
		static Complex* stereoConvStereoInputT(const Complex *input, const Complex *filterLeft, const Complex *filterRight, long nSIG, long NFILL, long NFILR, long &NFFT);
		
		
		//storing the an array into a text file
		//filename is the file name you want to store the data into
		//datatype represents the data you wanna store: real/real+imag/amplitude
    static void storingData(Complex *data, int NFFT, std::string temp, char datatype);
		
		//   FORWARD FOURIER TRANSFORM
		//     Input  - input data
		//     Output - transform result
		//     N      - length of both input data and result
		static bool Forward(const Complex *const Input, Complex *const Output, const unsigned int N);
	
		//   FORWARD FOURIER TRANSFORM, INPLACE VERSION
		//     Data - both input data and output
		//     N    - length of input data
		static bool Forward(Complex *const Data, const unsigned int N);
	
		//   INVERSE FOURIER TRANSFORM
		//     Input  - input data
		//     Output - transform result
		//     N      - length of both input data and result
		//     Scale  - if to scale result
		static bool Inverse(const Complex *const Input, Complex *const Output, const unsigned int N, const bool Scale = true);
	
		//   INVERSE FOURIER TRANSFORM, INPLACE VERSION
		//     Data  - both input data and output
		//     N     - length of both input data and result
		//     Scale - if to scale result
		static bool Inverse(Complex *const Data, const unsigned int N, const bool Scale = true);

	protected:
		//   Rearrange function and its inplace version
		static void Rearrange(const Complex *const Input, Complex *const Output, const unsigned int N);

		static void Rearrange(Complex *Data, const unsigned int N);

	
	
		//   FFT implementation
		static void Perform(Complex *const Data, const unsigned int N, const bool Inverse = false);
	
		//   Scaling of inverse FFT result
		static void Scale(Complex *const Data, const unsigned int N);
};

#endif
