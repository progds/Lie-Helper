/*
 * sample_code.h
 *
 *  Created on: Dec 4, 2016
 *      Author: progds
 */

#ifndef SAMPLE_CODE_H_
#define SAMPLE_CODE_H_

#include "basiclh.h"

// basicLH
void	sample_Accel_Motion_Capture(sensor_h sensor, sensor_event_s *event, void *data);
int		sample_Audio_Preprocessing(void *data, AUDIO_DATA* audio_seg);
void	sample_Speaker_Recognition(void *data, AUDIO_DATA* audio_seg, int segCnt);


// mfcc
#define MEL_round(fp) (int)((fp) >= 0 ? (fp) + 0.5 : (fp) - 0.5)
#define MEL_FRM_LEN		30				// ms
#define MEL_FRM_SPACE	10				// ms
#define	MEL_FFT_LEN		512
#define MEL_PI			3.1415926536
#define	MEL_FILT_NUM	26
#define MEL_CEPS_NUM	12
typedef struct sttComplex
{
	double real;
	double imaginary;
}COMPLEX;
COMPLEX COMPLEX_new(double real, double imaginary);
COMPLEX COMPLEX_add(COMPLEX a, COMPLEX b);
COMPLEX COMPLEX_sub(COMPLEX a, COMPLEX b);
COMPLEX COMPLEX_multiply(COMPLEX a, COMPLEX b);
int  mfcc(const void *input, unsigned int length, double **result, int *nRes);
void CreateFilt(double (*w)[MEL_FFT_LEN/2+1]);
void zero_fft(double *buffer,COMPLEX *vec);
void FFT(COMPLEX *vec);
int decision(double m0, double m1, double m2, double m3, double m4, double m5, double m6, double m7, double m8, double m9, double m10, double m11);


// kmeans
#define DIMENSIONS 12
#define NUMBER_OF_K
double getDistance(double* a, double* b);
double** getInitialCentroids(int k , double** data, int numTuples);
int* getNearestCentroids(int k, double** centroid, double** data, int numTuples);
double** getNewCentroids(int k, int* nearestMeans, double** data, int numTuples);
int compareNearestCentroids(int* last, int* current, int numTuples);
int* kmeans(int k, double** data, int numTuples);


// saving audio
void write_pcm_to_wavFile(const char* filename, int samplerate, int bitrate, int channel, void* data, int dLen);


#endif /* SAMPLE_CODE_H_ */
